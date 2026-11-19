#!/usr/bin/env python3
"""Go2 execution adapter. Geometry/search/replanning belong to SCAN alone.

This process validates message freshness, arbitrates StopMove acknowledgement,
forwards controller velocities, and finishes goal yaw. It never interprets a
prior map, a reference corridor, planner logs, or inflated occupancy as contact.
"""
import json
import sys
from pathlib import Path as FilePath
sys.path.insert(0,str(FilePath(__file__).resolve().parent))
import math
import threading
import time
from concurrent.futures import Future

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry, Path
from scan_planner.msg import Bspline
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Empty, Header, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker


from start_alignment_policy import StartAlignmentPolicy

class CommandGate:
    def __init__(self):
        self.lock = threading.RLock()
        self.shutting_down = False
        self.frame = rospy.get_param('~map_frame', 'map')
        self.body_height = float(rospy.get_param('~body_height', .4))
        self.xy_tolerance = float(rospy.get_param('~goal_xy_tolerance', .3))
        self.default_xy_tolerance = self.xy_tolerance
        self.z_tolerance = float(rospy.get_param('~goal_z_tolerance', .2))
        self.yaw_tolerance = float(rospy.get_param('~yaw_tolerance', .35))
        self.turn_min = float(rospy.get_param('~turn_min_speed', .15))
        self.turn_max = float(rospy.get_param('~turn_max_speed', .4))
        self.turn_kp = float(rospy.get_param('~turn_kp', 1.5))
        self.turn_progress_timeout = float(rospy.get_param('~turn_progress_timeout', .6))
        self.turn_accel = float(rospy.get_param('~turn_accel', .8))
        self.start_turn_max = float(rospy.get_param('~start_turn_max_speed', .5))
        self.start_turn_accel = float(rospy.get_param('~start_turn_accel', 1.6))
        self.turn_timeout = float(rospy.get_param('~turn_timeout', 30.))
        self.settle_time = float(rospy.get_param('~settle_time', .3))
        self.yaw_stable_time = float(rospy.get_param('~yaw_stable_time', .15))
        self.hold_distance = float(rospy.get_param('~goal_hold_distance', .45))
        self.collision_settle_time = float(rospy.get_param('~collision_settle_time', .3))
        self.stopped_linear = float(rospy.get_param('~stopped_linear_threshold', .03))
        self.stopped_yaw = float(rospy.get_param('~stopped_yaw_threshold', .08))
        self.stop_service_name = rospy.get_param('~stop_service', '/inspection/stop')
        self.stop_service = rospy.ServiceProxy(self.stop_service_name, Trigger)
        self.resume_service_name = str(rospy.get_param('~resume_control_service', ''))
        self.resume_service = rospy.ServiceProxy(self.resume_service_name, Trigger) if self.resume_service_name else None
        self.require_contact_feedback = bool(rospy.get_param('~require_contact_feedback', False))
        self.contact_timeout = float(rospy.get_param('~contact_timeout', .3))
        self.max_execution_replans = int(rospy.get_param('~max_execution_replans', 3))
        values = (self.body_height, self.xy_tolerance, self.z_tolerance, self.yaw_tolerance,
                  self.turn_min, self.turn_max, self.turn_kp, self.turn_accel, self.turn_timeout, self.turn_progress_timeout,
                  self.start_turn_max, self.start_turn_accel,
                  self.settle_time, self.yaw_stable_time, self.hold_distance,
                  self.collision_settle_time, self.stopped_linear, self.stopped_yaw, self.contact_timeout)
        if (not all(math.isfinite(v) and v > 0. for v in values)
                or not self.turn_min <= self.turn_max <= 1.
                or not self.turn_min <= self.start_turn_max <= 1. or self.max_execution_replans < 0):
            raise ValueError('invalid Go2 execution adapter configuration')
        self.geometry = rospy.get_param('/dog_nav/geometry', {})
        self.reference_offset = (float(self.geometry.get('reference_offset_x', 0.)),
                                 float(self.geometry.get('reference_offset_y', 0.)))
        if not all(math.isfinite(v) for v in self.reference_offset):
            raise ValueError('invalid navigation reference offset')
        self.radius = float(rospy.get_param('/scan_planner_node/grid_map/double_cylinder_radius', .225))
        self.cylinder_offset = float(rospy.get_param('/scan_planner_node/grid_map/double_cylinder_offset', .18))
        self.z_up = float(rospy.get_param('/scan_planner_node/grid_map/obstacles_inflation_z_up', .1))
        self.z_down = float(rospy.get_param('/scan_planner_node/grid_map/obstacles_inflation_z_down', .1))
        self.start_alignment = StartAlignmentPolicy(self.start_turn_max, self.turn_min, self.start_turn_accel)
        self.completed_start_alignment = None
        self.phase = 'idle'
        self.goal = self.pose = self.path_stamp = None
        self.tracking_goal = self.tracking_stamp = None
        self.cancel_stamp = self.last_curve_stamp = rospy.Time.now()
        self.blocked_since = rospy.Time()
        self.curve_ready = False
        self.odom_stamp = self.odom_received = self.cloud_stamp = self.cloud_received = None
        self.execution_received = self.command_received = self.curve_received = None
        self.contact_active = False
        self.contact_feedback_received = None
        self.collision_stop_pending = False
        self.collision_reason = ''
        self.localization_pause = None
        self.stop_future = None
        self.stop_retry_at = 0.
        self.stop_feedback_since = None
        self.chassis_feedback_msg = self.chassis_feedback_received = None
        self.execution_stall_count = 0
        self.execution_stall_pose = None
        self.turn_started = self.yaw_stable_since = None
        self.turn_settled_since = self.position_error_since = None
        self.position_recovery_stamp = None
        self.last_turn_speed = 0.
        self.yaw_holding = False
        self.turn_min_effective = self.turn_min
        self.turn_progress_since = self.turn_progress_anchor = None
        self.last_turn_time = time.monotonic()
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.alignment_done_pub=rospy.Publisher('/planning/start_alignment_done',String,queue_size=10)
        rospy.Subscriber('/planning/start_alignment',String,self.start_alignment_request,queue_size=1)
        self.position_pub = rospy.Publisher('/planning/goal_position_reached', Header, queue_size=10)
        self.position_lost_pub = rospy.Publisher('/planning/goal_position_lost', Header, queue_size=10)
        self.reached_pub = rospy.Publisher('/planning/goal_reached', Header, queue_size=10)
        self.failed_pub = rospy.Publisher('/planning/goal_failed', Header, queue_size=10)
        self.safety_pub = rospy.Publisher('/dog_nav/safety_stop', String, queue_size=1, latch=True)
        self.collision_control_pub = rospy.Publisher('/planning/collision_control', String, queue_size=20)
        self.collision_status_pub = rospy.Publisher('/dog_nav/collision_status', String, queue_size=10, latch=True)
        self.footprint_pub = rospy.Publisher('/dog_nav/footprint', Marker, queue_size=5, latch=True)
        rospy.Subscriber('/initial_path', Path, self.path, queue_size=1)
        rospy.Subscriber('/planning/tracking_path', Path, self.tracking_path, queue_size=1)
        rospy.Subscriber('/planning/bspline', Bspline, self.spline, queue_size=10)
        rospy.Subscriber('/planning/scan_cmd_vel', Twist, self.command, queue_size=1)
        rospy.Subscriber('/planning/go2_execution_frozen', Bool, self.execution, queue_size=1)
        rospy.Subscriber('/planning/execution_stall', String, self.execution_stall, queue_size=1)
        rospy.Subscriber('/planning/collision_stop', String, self.collision_event, queue_size=10)
        rospy.Subscriber(rospy.get_param('~contact_topic', '/dog_nav/contact_stop'), Bool, self.contact, queue_size=1)
        rospy.Subscriber('/go2/odom', Odometry, self.chassis_feedback, queue_size=1)
        rospy.Subscriber(rospy.get_param('~odom_topic', '/dog_nav/odometry'), Odometry, self.odom, queue_size=1)
        rospy.Subscriber(rospy.get_param('~cloud_topic', '/navigation/obstacle_points'), PointCloud2, self.cloud, queue_size=1)
        rospy.Subscriber('/map_manager/cancel', Empty, self.cancel, queue_size=1)
        rospy.Subscriber('/plan_map/cancel_scan', Empty, self.cancel, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(.02), self.watchdog)
        rospy.on_shutdown(self.shutdown)

    @staticmethod
    def coordinates(pose):
        p, q = pose.position, pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if not all(math.isfinite(v) for v in values) or norm < 1e-12:
            return None
        yaw = math.atan2(2.*(q.w*q.z+q.x*q.y), norm-2.*(q.y*q.y+q.z*q.z))
        return p.x, p.y, p.z, yaw

    def stop(self):
        try:
            self.publisher.publish(Twist())
        except rospy.ROSException:
            if not self.shutting_down and not rospy.is_shutdown():
                raise

    def shutdown(self):
        with self.lock:
            self.shutting_down = True
            self.stop()

    def cancel(self, _msg):
        with self.lock:
            self.start_alignment.cancel()
            self.completed_start_alignment = None
            self.cancel_stamp = rospy.Time.now()
            self.phase, self.goal, self.path_stamp = 'idle', None, None
            self.position_recovery_stamp = None
            self.localization_pause = None
            self.tracking_goal = self.tracking_stamp = None
            self.curve_ready = False
            self.command_received = None
            # Do not cancel an in-flight StopMove or release its interlock.
            self.stop()

    def path(self, msg):
        with self.lock:
            if not msg.poses:
                self.cancel(msg)
                return
            if (msg.header.stamp <= self.cancel_stamp
                    or (self.path_stamp is not None and msg.header.stamp <= self.path_stamp)):
                return
            goal = self.coordinates(msg.poses[-1].pose)
            if msg.header.frame_id != self.frame or goal is None:
                self.fail('invalid global goal protocol')
                return
            tolerance = float(rospy.get_param('~active_goal_xy_tolerance', self.default_xy_tolerance))
            if not math.isfinite(tolerance) or not .005 <= tolerance <= .5:
                self.fail('invalid goal arrival tolerance')
                return
            self.xy_tolerance = tolerance
            self.start_alignment.cancel()
            self.completed_start_alignment = None
            self.localization_pause = None
            self.path_stamp = msg.header.stamp
            self.goal = goal[0], goal[1], goal[2]+self.body_height, goal[3]
            self.phase = 'blocked' if self.collision_stop_pending else 'tracking'
            self.curve_ready = False
            self.command_received = None
            self.execution_stall_count = 0
            self.execution_stall_pose = None
            self.last_turn_speed = 0.
            self.yaw_holding = False
            self.turn_min_effective = self.turn_min
            self.turn_progress_since = self.turn_progress_anchor = None
            self.yaw_stable_since = None
            self.turn_settled_since = self.position_error_since = None
            self.position_recovery_stamp = None
            self.stop()

    def tracking_path(self, msg):
        with self.lock:
            if msg.header.stamp <= self.cancel_stamp:
                return
            if self.tracking_stamp is not None and msg.header.stamp < self.tracking_stamp:
                return
            if not msg.poses:
                self.tracking_goal = None
                self.tracking_stamp = msg.header.stamp
                return
            p = msg.poses[-1].pose.position
            if msg.header.frame_id != self.frame or not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
                return
            self.tracking_goal = p.x, p.y, p.z+self.body_height
            self.tracking_stamp = msg.header.stamp

    def spline(self, msg):
        with self.lock:
            if (self.path_stamp is None or self.phase in ('idle', 'failed', 'complete', 'settling', 'turning', 'start_aligning')
                    or msg.start_time < self.path_stamp or msg.start_time <= self.last_curve_stamp
                    or msg.start_time < self.blocked_since or self.collision_stop_pending):
                return
            n = len(msg.pos_pts)
            valid = msg.order == 3 and 4 <= n <= 2000 and len(msg.knots) == n+4
            valid = valid and all(math.isfinite(k) and (i == 0 or k > msg.knots[i-1]) for i, k in enumerate(msg.knots))
            valid = valid and all(math.isfinite(v) for p in msg.pos_pts for v in (p.x,p.y,p.z))
            if not valid or not 0. < msg.knots[n]-msg.knots[3] <= 120.:
                self.fail('malformed SCAN trajectory protocol')
                return
            moving = any(abs(p.x-msg.pos_pts[0].x)+abs(p.y-msg.pos_pts[0].y)+abs(p.z-msg.pos_pts[0].z) > 1e-6 for p in msg.pos_pts)
            self.last_curve_stamp = msg.start_time
            if msg.obstacle_hold:
                if moving:
                    self.fail('moving trajectory marked as a stop')
                else:
                    self.request_collision_stop('native SCAN emergency hold')
                return
            if not moving:
                self.curve_ready = False
                self.stop()
                return
            self.curve_ready = True
            self.curve_received = time.monotonic()
            self.phase = 'tracking'
            self.command_received = None

    def execution(self, _msg):
        with self.lock:
            self.execution_received = time.monotonic()

    def odom(self, msg):
        with self.lock:
            self.pose = self.coordinates(msg.pose.pose) if msg.header.frame_id == self.frame else None
            self.odom_stamp, self.odom_received = msg.header.stamp, time.monotonic()
            if self.pose is None:
                self.stop()
                return
            if self.execution_stall_pose is not None and math.dist(self.pose[:3], self.execution_stall_pose) > .3:
                self.execution_stall_count, self.execution_stall_pose = 0, None
            self.publish_footprint(msg)

    def cloud(self, msg):
        with self.lock:
            self.cloud_stamp = msg.header.stamp
            self.cloud_received = time.monotonic() if msg.header.frame_id == self.frame else None

    def fresh_pose(self):
        return (self.pose is not None and self.odom_received is not None
                and time.monotonic()-self.odom_received <= .5
                and -.1 <= (rospy.Time.now()-self.odom_stamp).to_sec() <= .5)

    def has_obstacle_input(self):
        # The planner owns occupancy/route feasibility. Cloud age alone must
        # not reset execution or demand a StopMove acknowledgement cycle.
        return self.cloud_received is not None

    def ready(self):
        return (self.phase == 'tracking' and self.curve_ready and not self.collision_stop_pending
                and not self.contact_active and self.fresh_pose() and self.has_obstacle_input()
                and self.execution_received is not None and time.monotonic()-self.execution_received <= .3
                and (not self.require_contact_feedback or self.contact_fresh()))

    def command(self, msg):
        with self.lock:
            if self.shutting_down:
                return
            if self.phase in ('settling', 'turning', 'start_aligning'):
                return  # The gate's yaw timer exclusively owns these commands.
            if not all(math.isfinite(v) for v in (msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)):
                self.fail('nonfinite controller command')
            elif self.ready():
                self.command_received = time.monotonic()
                self.publisher.publish(msg)
            else:
                self.stop()

    def chassis_feedback(self, msg):
        with self.lock:
            self.chassis_feedback_msg, self.chassis_feedback_received = msg, time.monotonic()

    def collision_status(self, state, reason=''):
        self.collision_status_pub.publish(String(data=json.dumps({
            'stamp': rospy.Time.now().to_sec(), 'state': state, 'reason': reason,
            'contact_active': self.contact_active,
            'goal_stamp': self.path_stamp.to_sec() if self.path_stamp is not None else None})))

    def request_collision_stop(self, reason):
        if self.shutting_down or self.collision_stop_pending:
            return
        self.start_alignment.cancel()
        self.collision_stop_pending, self.collision_reason = True, reason
        self.stop_future = None
        self.stop_retry_at = 0.
        self.stop_feedback_since = None
        self.curve_ready = False
        self.command_received = None
        self.blocked_since = rospy.Time.now()
        if self.phase not in ('idle', 'failed', 'complete'):
            self.phase = 'blocked'
        self.stop()
        self.collision_control_pub.publish(String(data='hold'))
        self.collision_status('STOP_REQUESTED', reason)
        rospy.logwarn('Go2 execution stop requested: %s', reason)

    def collision_event(self, msg):
        with self.lock:
            try:
                event = json.loads(msg.data)
                stamp = float(event['stamp'])
                if not math.isfinite(stamp) or (self.path_stamp is not None and stamp < self.path_stamp.to_sec()):
                    return
                self.request_collision_stop(str(event.get('reason', 'native planner emergency')))
            except (ValueError, TypeError, KeyError):
                rospy.logwarn_throttle(2., 'Malformed planner emergency event ignored')

    def execution_stall(self, msg):
        with self.lock:
            if self.phase != 'tracking' or self.collision_stop_pending or self.path_stamp is None:
                return
            try:
                event = json.loads(msg.data)
                stamp = float(event['stamp'])
                if not math.isfinite(stamp) or stamp < self.path_stamp.to_sec():
                    return
            except (ValueError, TypeError, KeyError):
                return
            self.execution_stall_count += 1
            self.execution_stall_pose = self.pose[:3] if self.pose else None
            self.request_collision_stop('measured execution made no progress')
            if self.execution_stall_count > self.max_execution_replans:
                self.fail('repeated commands without measured motion')

    def contact(self, msg):
        with self.lock:
            self.contact_feedback_received = time.monotonic()
            if self.contact_active != bool(msg.data):
                self.stop_feedback_since = None
            self.contact_active = bool(msg.data)
            if self.contact_active:
                self.request_collision_stop('confirmed external contact')

    def contact_fresh(self):
        return (self.contact_feedback_received is not None
                and time.monotonic()-self.contact_feedback_received <= self.contact_timeout)

    def start_stop_request(self):
        future = Future()
        self.stop_future = future
        def call():
            try:
                rospy.wait_for_service(self.stop_service_name, timeout=1.)
                if self.shutting_down:
                    future.set_result((False, 'navigation shutting down'))
                    return
                reply = self.stop_service()
                future.set_result((bool(reply.success), reply.message))
            except Exception as error:
                future.set_result((False, str(error)))
        threading.Thread(target=call, name='go2_stop_ack', daemon=True).start()

    def restore_control(self):
        if not self.resume_service_name:
            return
        rospy.wait_for_service(self.resume_service_name, timeout=1.)
        reply = self.resume_service()
        if not reply.success:
            raise RuntimeError(reply.message)

    def collision_watchdog(self, now):
        self.stop()
        if self.stop_future is None and now >= self.stop_retry_at:
            self.start_stop_request()
        if self.stop_future is None or not self.stop_future.done():
            self.stop_feedback_since = None
            return
        accepted, detail = self.stop_future.result()
        if not accepted:
            rospy.logwarn_throttle(2., 'StopMove not acknowledged: %s', detail)
            self.stop_future, self.stop_retry_at, self.stop_feedback_since = None, now+.5, None
            return
        msg = self.chassis_feedback_msg
        stationary = False
        if msg is not None and self.chassis_feedback_received is not None:
            v = msg.twist.twist
            speed, yaw = math.hypot(v.linear.x,v.linear.y), abs(v.angular.z)
            stationary = (math.isfinite(speed) and math.isfinite(yaw) and speed <= self.stopped_linear
                          and yaw <= self.stopped_yaw and -.1 <= (rospy.Time.now()-msg.header.stamp).to_sec() <= .3
                          and now-self.chassis_feedback_received <= .3)
        if (self.contact_active or (self.require_contact_feedback and not self.contact_fresh())
                or not stationary or not self.fresh_pose() or not self.has_obstacle_input()):
            self.stop_feedback_since = None
            return
        if self.stop_feedback_since is None:
            self.stop_feedback_since = now
            return
        if now-self.stop_feedback_since < self.collision_settle_time:
            return
        # The integrated arbiter revokes ownership on StopMove. Restore it
        # only for the still-active goal, after the physical stop barrier.
        # Cancellation clears goal/path under the same lock; it must never
        # reacquire control. Standalone hardware keeps its original behavior.
        if self.path_stamp is not None and self.goal is not None and self.phase == 'blocked':
            try:
                self.restore_control()
            except Exception as error:
                self.fail('navigation control recovery failed: '+str(error))
                return
        self.collision_stop_pending, self.stop_future = False, None
        self.blocked_since = self.last_curve_stamp = rospy.Time.now()
        self.curve_ready = False
        self.collision_control_pub.publish(String(data='replan'))
        self.collision_status('STOP_CONFIRMED_REPLANNING', self.collision_reason)
        pause = getattr(self, 'localization_pause', None)
        self.localization_pause = None
        if pause is not None and pause[0] == self.path_stamp and self.goal is not None and self.phase == 'blocked':
            if pause[1] in ('settling', 'turning'):
                self.phase, self.turn_started = 'settling', now
                self.turn_settled_since = self.position_error_since = self.yaw_stable_since = None
                self.last_turn_speed = 0.
                self.yaw_holding = False
            self.collision_status('LOCALIZATION_RECOVERED', 'fresh pose and physical stop confirmed; same goal retained')
        # Recover the SAME original goal only after the physical stop barrier.
        # The global planner bounds retries and stamps the new route; an old
        # goal's recovery event can never replace a new or cancelled goal.
        recovery = self.position_recovery_stamp
        self.position_recovery_stamp = None
        if recovery is not None and recovery == self.path_stamp and self.goal is not None and self.phase == 'blocked':
            self.position_lost_pub.publish(Header(stamp=recovery, frame_id=self.frame))

    def fail(self, reason):
        if self.phase in ('idle', 'failed', 'complete'):
            return
        self.phase = 'failed'
        self.request_collision_stop(reason)
        self.safety_pub.publish(String(data=json.dumps({'stamp':rospy.Time.now().to_sec(),'reason':reason})))
        if self.path_stamp is not None:
            self.failed_pub.publish(Header(stamp=self.path_stamp, frame_id=self.frame))
        rospy.logwarn('Go2 execution adapter stopped: %s', reason)

    def watchdog(self, _event):
        with self.lock:
            if self.shutting_down:
                return
            now = time.monotonic()
            if self.collision_stop_pending:
                self.collision_watchdog(now)
                return
            if self.phase not in ('tracking', 'blocked', 'settling', 'turning', 'start_aligning'):
                self.stop()
                return
            if self.require_contact_feedback and not self.contact_fresh():
                self.request_collision_stop('required contact feedback missing or stale')
                return
            if not self.fresh_pose():
                self.stop()
                if self.curve_ready or self.phase in ('settling', 'turning', 'start_aligning'):
                    self.localization_pause = (self.path_stamp, self.phase)
                    self.request_collision_stop('localization feedback temporarily unavailable')
                    self.collision_status('WAITING_LOCALIZATION', 'holding goal until fresh localization and confirmed stop')
                return
            if not self.has_obstacle_input():
                self.stop()
                if self.curve_ready:
                    self.fail('obstacle input not initialized')
                return
            if self.phase=='start_aligning':
                self.execute_start_alignment(now)
                return
            if self.phase in ('settling', 'turning'):
                if now-self.turn_started > self.turn_timeout:
                    self.fail('final orientation timeout')
                else:
                    self.finish_orientation(now)
                return
            xy, z = self.goal_error()
            capture = False
            if self.tracking_goal is not None and self.tracking_stamp == self.path_stamp:
                # The follower may reach the planned navigation endpoint at a
                # different approach yaw after a detour. Enter final alignment
                # there rather than waiting forever for an unaligned lidar
                # point; final success still checks the actual body goal.
                capture = (math.hypot(self.tracking_goal[0]-self.pose[0], self.tracking_goal[1]-self.pose[1]) <= self.xy_tolerance
                           and abs(self.tracking_goal[2]-self.pose[2]) <= self.z_tolerance)
            if (xy <= self.xy_tolerance and z <= self.z_tolerance) or capture:
                self.phase, self.turn_started = 'settling', now
                self.turn_settled_since = self.position_error_since = None
                self.last_turn_speed = 0.
                self.yaw_holding = False
                self.turn_min_effective = self.turn_min
                self.turn_progress_since = self.turn_progress_anchor = None
                self.curve_ready = False
                self.position_pub.publish(Header(stamp=self.path_stamp, frame_id=self.frame))
                self.stop()
                return
            if self.phase == 'blocked':
                self.stop()
                return
            if self.curve_ready and (self.execution_received is None or now-self.execution_received > .3):
                self.fail('controller execution heartbeat stale')
            elif self.curve_ready and self.command_received is None and self.curve_received is not None and now-self.curve_received > .3:
                self.fail('controller produced no command for accepted trajectory')
            elif self.command_received is not None and now-self.command_received > .3:
                self.fail('controller command stream stale')
            elif not self.ready():
                self.stop()

    def start_alignment_request(self,msg):
        with self.lock:
            if self.shutting_down or self.path_stamp is None or self.goal is None:return
            try:event=json.loads(msg.data)
            except (TypeError,ValueError):return
            if not isinstance(event,dict) or event.get('request_id')==self.completed_start_alignment:return
            if self.collision_stop_pending or self.contact_active:return
            may_start=self.phase in ('tracking','blocked','start_aligning') and self.fresh_pose() and self.has_obstacle_input()
            accepted=self.start_alignment.receive(event,self.path_stamp.to_nsec(),self.frame,self.pose,rospy.Time.now().to_sec(),time.monotonic(),may_start)
            if accepted:
                if self.phase!='start_aligning':
                    self.curve_ready=False;self.command_received=None;self.stop()
                    self.collision_status('START_ALIGNING','planner-certified zero-translation rotation')
                self.phase='start_aligning'
            elif event.get('cancel') and self.phase=='start_aligning' and self.start_alignment.intent is None:
                self.phase='tracking';self.curve_ready=False;self.stop()

    def execute_start_alignment(self,now):
        rate,state=self.start_alignment.step(self.pose,self.stationary(now),rospy.Time.now().to_sec(),now)
        if state in ('position_displaced','pose_reset','timeout'):
            self.request_collision_stop('start alignment '+state);return
        if state=='done':
            request_id=self.start_alignment.intent['request_id']
            self.completed_start_alignment=request_id
            self.stop();self.start_alignment.cancel();self.phase='tracking';self.curve_ready=False;self.command_received=None
            self.alignment_done_pub.publish(String(data=request_id));return
        if rate==0.:self.stop();return
        command=Twist();command.angular.z=rate;self.publisher.publish(command)

    def goal_error(self):
        # Goal and feedback refer to the lidar in legacy mode. Compare their
        # body centers at their OWN headings, not two different lidar poses.
        dx, dy = self.reference_offset
        def body(p):
            c, s = math.cos(p[3]), math.sin(p[3])
            return p[0]+c*dx-s*dy, p[1]+s*dx+c*dy
        gx, gy = body(self.goal)
        px, py = body(self.pose)
        return math.hypot(gx-px, gy-py), abs(self.goal[2]-self.pose[2])

    def stationary(self, now):
        msg = self.chassis_feedback_msg
        if msg is None or self.chassis_feedback_received is None:
            return False
        v = msg.twist.twist
        values = (v.linear.x, v.linear.y, v.angular.z)
        return (all(math.isfinite(x) for x in values)
                and math.hypot(v.linear.x, v.linear.y) <= self.stopped_linear
                and abs(v.angular.z) <= self.stopped_yaw
                and -.1 <= (rospy.Time.now()-msg.header.stamp).to_sec() <= .3
                and now-self.chassis_feedback_received <= .3)

    def recover_position(self, reason):
        self.position_recovery_stamp = self.path_stamp
        self.request_collision_stop(reason)
        rospy.logwarn('Final orientation needs position recovery after stop acknowledgement: %s', reason)

    def finish_orientation(self, now):
        xy, z = self.goal_error()
        if xy > self.hold_distance or z > 2.*self.z_tolerance:
            self.last_turn_speed = 0.
            self.stop()
            if self.position_error_since is None:
                self.position_error_since = now
            elif now-self.position_error_since >= self.settle_time:
                self.recover_position('body position displaced during final orientation (xy=%.3f z=%.3f m)' % (xy, z))
            return
        self.position_error_since = None
        if self.phase == 'settling':
            self.last_turn_time = now
            self.stop()
            if not self.stationary(now):
                self.turn_settled_since = None
                return
            if self.turn_settled_since is None:
                self.turn_settled_since = now
            if now-self.turn_settled_since < self.settle_time:
                return
            self.phase = 'turning'
        error = math.atan2(math.sin(self.goal[3]-self.pose[3]),math.cos(self.goal[3]-self.pose[3]))
        # Acquire inside half the final tolerance, then hold until the full
        # tolerance is exceeded. Stopping on its noisy boundary caused repeated
        # short command ramps and never allowed a quiet goal acknowledgement.
        if self.yaw_holding and abs(error) > self.yaw_tolerance:
            self.yaw_holding = False
            self.yaw_stable_since = None
            self.turn_progress_since = self.turn_progress_anchor = None
        if not self.yaw_holding and abs(error) <= .5*self.yaw_tolerance:
            self.yaw_holding = True
        if self.yaw_holding:
            self.last_turn_speed = 0.
            self.turn_progress_since = self.turn_progress_anchor = None
            self.stop()
            if not self.stationary(now):
                self.yaw_stable_since = None
                return
            if self.yaw_stable_since is None:
                self.yaw_stable_since = now
            elif now-self.yaw_stable_since >= self.yaw_stable_time:
                # Both position and yaw must satisfy the original tolerances.
                if xy > self.xy_tolerance or z > self.z_tolerance:
                    self.recover_position('final yaw reached with residual body position error (xy=%.3f z=%.3f m)' % (xy, z))
                else:
                    self.phase = 'complete'
                    self.reached_pub.publish(Header(stamp=self.path_stamp, frame_id=self.frame))
            return
        self.yaw_stable_since = None
        if self.turn_progress_since is None:
            self.turn_progress_since, self.turn_progress_anchor = now, self.pose[3]
        else:
            moved = math.atan2(math.sin(self.pose[3]-self.turn_progress_anchor),
                               math.cos(self.pose[3]-self.turn_progress_anchor))
            if abs(moved) >= .02 and moved*error > 0.:
                self.turn_progress_since, self.turn_progress_anchor = now, self.pose[3]
            elif now-self.turn_progress_since >= self.turn_progress_timeout:
                # Feedback-driven deadband compensation, still bounded by the
                # existing final-turn speed and acceleration limits.
                self.turn_min_effective = min(self.turn_max, self.turn_min_effective+.05)
                self.turn_progress_since, self.turn_progress_anchor = now, self.pose[3]
        requested = math.copysign(min(self.turn_max,max(self.turn_min_effective,abs(self.turn_kp*error))),error)
        if requested*self.last_turn_speed < 0.:
            self.last_turn_speed = 0.
        change = self.turn_accel*min(.1,max(0.,now-self.last_turn_time))
        self.last_turn_speed += max(-change,min(change,requested-self.last_turn_speed))
        self.last_turn_time = now
        cmd = Twist();cmd.angular.z = self.last_turn_speed
        self.publisher.publish(cmd)

    def publish_footprint(self, msg):
        # Display is descriptive only; it does not veto motion or edit a map.
        g = self.geometry
        cx = float(g.get('center_x',0.))+float(g.get('reference_offset_x',0.))
        cy = float(g.get('center_y',0.))+float(g.get('reference_offset_y',0.))
        x,y,z,yaw = self.pose
        c,s = math.cos(yaw), math.sin(yaw)
        up, down = self.z_up, self.z_down
        for index, offset in enumerate((self.cylinder_offset,-self.cylinder_offset)):
            marker = Marker();marker.header = msg.header;marker.ns = 'scan_native_cylinders';marker.id = index
            marker.type, marker.action = Marker.CYLINDER, Marker.ADD
            marker.pose.orientation.w = 1.
            marker.pose.position.x = x+c*(cx+offset)-s*cy
            marker.pose.position.y = y+s*(cx+offset)+c*cy
            marker.pose.position.z = z+.5*(down-up)
            marker.scale.x = marker.scale.y = 2.*self.radius
            marker.scale.z = max(.001,up+down)
            marker.color.b, marker.color.g, marker.color.a = 1.,.6,.25
            marker.lifetime = rospy.Duration(.5)
            self.footprint_pub.publish(marker)
        hx = .5*float(g.get('length',.7))+float(g.get('clearance',.03))
        hy = .5*float(g.get('width',.35))+float(g.get('clearance',.03))
        marker = Marker();marker.header = msg.header;marker.ns = 'prior_footprint';marker.id = 0
        marker.type, marker.action = Marker.LINE_STRIP, Marker.ADD;marker.pose.orientation.w = 1.
        marker.scale.x = .015;marker.color.g,marker.color.a = 1.,.8
        for dx,dy in ((-hx,-hy),(hx,-hy),(hx,hy),(-hx,hy),(-hx,-hy)):
            marker.points.append(Point(x=x+c*(cx+dx)-s*(cy+dy),y=y+s*(cx+dx)+c*(cy+dy),z=z))
        marker.lifetime = rospy.Duration(.5);self.footprint_pub.publish(marker)


if __name__ == '__main__':
    rospy.init_node('scan_command_gate')
    CommandGate()
    rospy.spin()
