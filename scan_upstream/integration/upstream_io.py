#!/usr/bin/env python3
"""Upstream SCAN transport adapter; leaves collision and tracking to upstream."""
import sys
from pathlib import Path as FilePath
sys.path.insert(0, str(FilePath(__file__).resolve().parent))
import json
import copy
import math
import threading
import time
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, Empty, Header, String
from scan_upstream_planner.msg import Bspline
from execution_policy import (
    ExecutionPolicy, cabin_turn_clear, hit_point_bytes, limit_elevator_exit, limit_near_cabin, world_velocity,
)

class Adapter:
    def __init__(self):
        self.lock = threading.RLock()
        self.policy = ExecutionPolicy()
        self.goal = self.pose = self.route_header = None
        self.short_route = False
        self.lobby_goal = None
        self.ingress_goal = False
        self.mission_state = ""
        self.exit_refresh_origin = None
        self.exit_stopped_at = None
        self.exit_refresh_requested = False
        self.settled_at = None
        self.last_status_at = 0.
        self.last_motion_generation = None
        self.xy_tolerance = float(rospy.get_param('~goal_xy_tolerance', .25))
        self.z_tolerance = float(rospy.get_param('~goal_z_tolerance', .25))
        self.body_height = float(rospy.get_param('~body_height', .4))
        self.lobby_yaw_limit = min(.55, float(rospy.get_param('~max_yaw', .55)))
        if not math.isfinite(self.lobby_yaw_limit) or self.lobby_yaw_limit <= 0.:
            raise ValueError("lobby yaw limit must be finite and positive")
        self.cabin_speed = float(rospy.get_param('~elevator_speed_limit', .20))
        self.ingress_speed = float(rospy.get_param('~elevator_ingress_speed_limit', .30))
        self.corridor_speed = float(rospy.get_param('~max_vel', .75))
        if not math.isfinite(self.corridor_speed) or self.corridor_speed <= 0.:
            raise ValueError("max_vel must be finite and positive")
        self.exit_speed = min(.35, self.corridor_speed)
        if not math.isfinite(self.ingress_speed) or self.ingress_speed <= 0:
            raise ValueError("elevator_ingress_speed_limit must be finite and positive")
        if not math.isfinite(self.cabin_speed) or self.cabin_speed <= 0:
            raise ValueError("elevator_speed_limit must be finite and positive")
        elevator = rospy.get_param('~elevator', {})
        self.align_exit_yaw = bool(elevator.get('align_exit_yaw', False))
        self.exit_stops = {}
        self.exit_floor = ''
        self.alignment_stamp = rospy.Time()
        self.elevator_mode = False
        self.exit_door_seen = {}
        self.aligned_reported_at = float("-inf")
        self.exit_yaw_settled_at = None
        self.cabin_yaw_check = None
        self.cabin_sweep_radius = (float(rospy.get_param('~point_radius', .20))
                                   + .5 * float(rospy.get_param('~point_separation', .36)) + .04)
        self.cabins = []
        self.lobbies = []
        self.lobby_planes = {}
        self.lobby_arrival_tolerance = float(rospy.get_param('~lobby_arrival_tolerance', .45))
        if not self.xy_tolerance <= self.lobby_arrival_tolerance <= .60:
            raise ValueError("lobby_arrival_tolerance must be between goal_xy_tolerance and 0.60 m")
        if elevator.get('configured', False):
            for floor, stop in elevator.get('floors', {}).items():
                x, y, z, yaw = map(float, stop['inside_pose'])
                if not all(math.isfinite(value) for value in (x, y, z)):
                    raise ValueError("elevator cabin pose must be finite")
                self.cabins.append((x, y, z))
                if 'lobby_pose' in stop:
                    lx, ly, lz, _ = map(float, stop['lobby_pose'])
                    if not all(math.isfinite(value) for value in (lx, ly, lz)):
                        raise ValueError("elevator lobby pose must be finite")
                    lobby = (lx, ly, lz)
                    self.lobbies.append(lobby)
                    if 'door_center_xy' in stop:
                        cx, cy = map(float, stop['door_center_xy'])
                        nx, ny = cx - x, cy - y
                        norm = math.hypot(nx, ny)
                        if not math.isfinite(norm) or norm < .05:
                            raise ValueError("invalid elevator door center")
                        nx, ny = nx / norm, ny / norm
                        if (lx - cx) * nx + (ly - cy) * ny < .80 + self.lobby_arrival_tolerance:
                            raise ValueError("lobby setback is too short for its arrival tolerance")
                        self.lobby_planes[lobby] = (cx, cy, nx, ny)
                        half_door = .5 * float(stop.get('door_width', elevator.get('door_width', 1.)))
                        if not math.isfinite(half_door) or half_door <= 0.:
                            raise ValueError("invalid elevator door width")
                        self.exit_stops[floor] = (x, y, z, yaw, cx, cy, nx, ny, half_door)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cloud_pub = rospy.Publisher('/scan_upstream/cloud', PointCloud2, queue_size=1)
        self.body_pub = rospy.Publisher('/scan_upstream/body_odometry', Odometry, queue_size=5)
        self.done = rospy.Publisher('/planning/goal_reached', Header, queue_size=1)
        self.status = rospy.Publisher('/planning/upstream_adapter_status', String, queue_size=1)
        self.refresh = rospy.Publisher('/planning/reference_refresh', Header, queue_size=1)
        rospy.Subscriber('/initial_path', Path, self.route, queue_size=1)
        rospy.Subscriber('/planning/scan_upstream_cmd_vel', Twist, self.command, queue_size=1)
        rospy.Subscriber('/planning/bspline', Bspline, self.curve, queue_size=1)
        rospy.Subscriber('/planning/body_odometry', Odometry, self.odom, queue_size=5)
        rospy.Subscriber('/navigation/obstacle_points', PointCloud2, self.cloud, queue_size=1)
        rospy.Subscriber('/dog_nav/localization_status', String, self.localization, queue_size=1)
        rospy.Subscriber('/elevator_nav/status', String, self.mission_status, queue_size=1)
        if self.align_exit_yaw:
            self.cabin_aligned = rospy.Publisher('/elevator_nav/cabin_aligned', Header, queue_size=1)
            rospy.Subscriber('/elevator/door_status', String, self.exit_door_status, queue_size=1)
            rospy.Subscriber('/elevator_lio/in_elevator', Bool, self.on_elevator_mode, queue_size=1)
        for topic in ('/map_manager/cancel', '/plan_map/cancel_scan'):
            rospy.Subscriber(topic, Empty, self.cancel, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(.02), self.tick)
        rospy.on_shutdown(lambda: self.pub.publish(Twist()))

    def route(self, msg):
        with self.lock:
            if not msg.poses:
                self.policy.cancel()
                self.goal = None
                return
            if self.route_header and msg.header.stamp < self.route_header.stamp:
                return
            self.route_header = msg.header
            self.cabin_yaw_check = None
            self.exit_refresh_origin = self.pose[:2] if self.pose else None
            self.exit_stopped_at = None
            self.exit_refresh_requested = False
            target = msg.poses[-1].pose
            p = target.position
            self.goal = (p.x, p.y, p.z+self.body_height, self.yaw(target.orientation))
            self.lobby_goal = next(
                (lobby for lobby in self.lobbies
                 if math.hypot(p.x - lobby[0], p.y - lobby[1]) <= .02
                 and abs(p.z - lobby[2]) <= .02), None)
            self.ingress_goal = any(
                math.hypot(p.x - x, p.y - y) <= .02 and abs(p.z - z) <= .02
                for x, y, z in self.cabins)
            self.short_route = all(
                math.hypot(p.x - item.pose.position.x, p.y - item.pose.position.y) <= self.xy_tolerance
                for item in msg.poses)
            self.settled_at = None
            # Use ROS receive time: the original goal stamp can predate a
            # canceled plan. Only a newly generated upstream curve may resume.
            self.policy.route(rospy.Time.now().to_sec())

    def curve(self, msg):
        with self.lock:
            if msg.pos_pts and msg.knots:
                self.policy.curve(msg.start_time.to_sec())

    def command(self, msg):
        with self.lock:
            self.policy.submit((msg.linear.x, msg.linear.y, msg.angular.z), time.monotonic())

    @staticmethod
    def yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def odom(self, msg):
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
        values = (p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z)
        if not all(math.isfinite(x) for x in values) or sum(x*x for x in values[3:7]) < 1e-8:
            return
        world = copy.deepcopy(msg)
        velocity = world_velocity((q.x,q.y,q.z,q.w), (v.x,v.y,v.z))
        angular = msg.twist.twist.angular
        omega = world_velocity((q.x,q.y,q.z,q.w), (angular.x,angular.y,angular.z))
        world.twist.twist.linear.x, world.twist.twist.linear.y, world.twist.twist.linear.z = velocity
        world.twist.twist.angular.x, world.twist.twist.angular.y, world.twist.twist.angular.z = omega
        # Legacy upstream consumes world-frame derivatives alongside body pose.
        # This private topic never feeds back into the standard localization odom.
        self.body_pub.publish(world)
        with self.lock:
            self.pose = (p.x, p.y, p.z, math.hypot(v.x,v.y), self.yaw(q), abs(v.z))
            self.policy.odom_at = time.monotonic()
            self.policy.odom_stamp = msg.header.stamp.to_sec()

    def localization(self, msg):
        with self.lock:
            self.policy.tracking = msg.data == 'TRACKING'

    def mission_status(self, msg):
        try:
            status = json.loads(msg.data)
            state = status.get('state', '')
            floor = status.get('alignment_floor', '')
            secs, nsecs = divmod(int(status.get('alignment_stamp_ns', 0)), 10**9)
            stamp = rospy.Time(secs, nsecs)
        except (ValueError, AttributeError, TypeError, OverflowError):
            state, floor, stamp = '', '', rospy.Time()
        with self.lock:
            if state != self.mission_state or floor != self.exit_floor or stamp != self.alignment_stamp:
                self.aligned_reported_at = float("-inf")
                self.exit_yaw_settled_at = None
                self.cabin_yaw_check = None
            self.mission_state, self.exit_floor, self.alignment_stamp = state, floor, stamp

    def on_elevator_mode(self, msg):
        with self.lock:
            self.elevator_mode = msg.data
            if msg.data:
                self.exit_yaw_settled_at = None
                self.cabin_yaw_check = None
                self.pub.publish(Twist())

    def exit_door_status(self, msg):
        try:
            status = json.loads(msg.data)
            floor = status.get('floor', '')
        except (ValueError, AttributeError):
            return
        with self.lock:
            if not floor:
                self.exit_door_seen.clear()
            elif status.get('side') == 'cabin' and status.get('open_confirmed') is True:
                self.exit_door_seen[floor] = time.monotonic()
            else:
                self.exit_door_seen.pop(floor, None)

    @staticmethod
    def exit_door_clearance(pose, stop):
        return (pose[0] - stop[4]) * stop[6] + (pose[1] - stop[5]) * stop[7]

    def cabin_sweep_fits_door(self, pose, stop):
        _, _, _, _, cx, cy, nx, ny, half_door = stop
        depth = -self.exit_door_clearance(pose, stop)
        if depth >= self.cabin_sweep_radius:
            return True
        # An open doorway can contain part of the turning envelope, provided
        # the entire cross-section at the threshold clears both jambs.
        cross_track = abs((pose[0] - cx) * ny - (pose[1] - cy) * nx)
        intersection = math.sqrt(max(0., self.cabin_sweep_radius ** 2 - depth ** 2))
        return cross_track + intersection <= half_door

    def align_before_exit(self, command, state, now, ros_now):
        hold = (0., 0., 0.)
        if (self.elevator_mode or
                self.mission_state in ('WAIT_ELEVATOR_MODE', 'ELEVATOR_TRANSIT', 'WAIT_DOOR_OPEN')):
            return hold, 'waiting_in_elevator'
        if not self.align_exit_yaw or self.mission_state not in ('ALIGN_CABIN', 'ALIGN_EXIT'):
            return command, state
        pose, stop = self.pose, self.exit_stops.get(self.exit_floor)
        if (pose is None or stop is None or not self.policy.tracking
                or not 0. <= now - self.policy.odom_at <= .5
                or not -.1 <= ros_now - self.policy.odom_stamp <= .5
                or abs(pose[2] - self.body_height - stop[2]) > .40 or pose[5] > .12):
            self.exit_yaw_settled_at = None
            return hold, 'exit_yaw_waiting_pose'
        # The center must remain in the cabin. Use the swept footprint and
        # the open doorway to judge clearance at the threshold.
        if (self.exit_door_clearance(pose, stop) >= 0.
                or math.hypot(pose[0] - stop[0], pose[1] - stop[1]) > .45):
            self.exit_yaw_settled_at = None
            return hold, 'cabin_yaw_waiting_inside_pose'
        if now - self.exit_door_seen.get(self.exit_floor, float('-inf')) > .60:
            self.exit_yaw_settled_at = None
            return hold, 'exit_yaw_waiting_open_door'
        heading = math.atan2(math.sin(stop[3] - pose[4]), math.cos(stop[3] - pose[4]))
        if abs(heading) <= .10 and pose[3] < .08:
            if self.exit_yaw_settled_at is None:
                self.exit_yaw_settled_at = now
            elif now - self.exit_yaw_settled_at >= .30:
                if now - self.aligned_reported_at >= .2:
                    self.cabin_aligned.publish(Header(stamp=self.alignment_stamp, frame_id='map'))
                    self.aligned_reported_at = now
                return hold, 'cabin_yaw_aligned'
            return hold, 'exit_yaw_settling'
        self.exit_yaw_settled_at = None
        check = self.cabin_yaw_check
        if (pose[3] >= .08 or abs(heading) <= .10 or not self.cabin_sweep_fits_door(pose, stop)
                or check is None
                or not check[2] or not -.1 <= ros_now - check[0] <= .35
                or math.hypot(pose[0] - check[1][0], pose[1] - check[1][1]) > .03):
            return hold, 'exit_yaw_waiting_clearance'
        direction = 1.0 if abs(heading) > math.pi - .15 else heading
        yaw_rate = math.copysign(min(self.lobby_yaw_limit, max(.45, 1.8 * abs(heading))), direction)
        return (0., 0., yaw_rate), 'aligning_to_cabin_exit'

    def refresh_after_exit_stall(self, command, now, ros_now):
        """Re-anchor a blocked exit after the measured body has moved."""
        if (self.mission_state != 'TRACK_LEG2' or self.exit_refresh_requested
                or not self.policy.active or not self.policy.tracking or self.pose is None
                or not 0. <= now - self.policy.odom_at <= .5
                or not -.1 <= ros_now - self.policy.odom_stamp <= .5
                or not 0. <= now - self.policy.cloud_at <= 1.):
            self.exit_stopped_at = None
            return
        x, y, z, speed, _, _ = self.pose
        if self.exit_refresh_origin is None:
            self.exit_refresh_origin = (x, y)
        if (speed >= .08 or math.hypot(*command[:2]) >= .05
                or math.hypot(x - self.exit_refresh_origin[0], y - self.exit_refresh_origin[1]) < .25
                or not any(abs(z - self.body_height - cz) <= .75
                           and math.hypot(x - cx, y - cy) <= 1.6 for cx, cy, cz in self.cabins)):
            self.exit_stopped_at = None
            return
        if self.exit_stopped_at is None:
            self.exit_stopped_at = now
        elif now - self.exit_stopped_at >= 1.:
            self.exit_refresh_requested = True
            self.refresh.publish(self.route_header)

    def cloud(self, msg):
        if msg.header.frame_id != 'map' or not -.1 <= (rospy.Time.now()-msg.header.stamp).to_sec() <= 1.:
            return
        # The localizer supplies explicit free-ray endpoints for the modified
        # mapper. Original SCAN has no such flag and would mark them occupied.
        flag = next((f for f in msg.fields if f.name == 'free_endpoint'), None)
        if flag:
            if flag.datatype != PointField.FLOAT32 or flag.count != 1:
                rospy.logerr_throttle(2., 'Unsupported free_endpoint field; cloud rejected')
                return
            try:
                data=hit_point_bytes(msg.data,msg.height,msg.width,msg.row_step,msg.point_step,
                    flag.offset,msg.is_bigendian)
            except ValueError as error:
                rospy.logerr_throttle(2., str(error))
                return
            msg = PointCloud2(header=msg.header, height=1, width=len(data)//msg.point_step,
                fields=msg.fields, is_bigendian=msg.is_bigendian, point_step=msg.point_step,
                row_step=len(data), data=bytes(data), is_dense=msg.is_dense)
        with self.lock:
            self.policy.cloud_at = time.monotonic()
            goal, pose, route = self.goal, self.pose, self.route_header
            stop = self.exit_stops.get(self.exit_floor)
            check_cabin = (self.align_exit_yaw and not self.elevator_mode
                           and self.mission_state in ('ALIGN_CABIN', 'ALIGN_EXIT') and stop is not None
                           and pose is not None and self.exit_door_clearance(pose, stop) < 0.)
        self.cloud_pub.publish(msg)
        if check_cabin:
            try:
                points = point_cloud2.read_points(msg, ('x', 'y', 'z'), skip_nans=True)
                clear = cabin_turn_clear(points, pose[:3], self.body_height, self.cabin_sweep_radius)
            except (AssertionError, ValueError):
                clear = False
            with self.lock:
                if self.route_header is route and self.goal is goal:
                    self.cabin_yaw_check = (msg.header.stamp.to_sec(), pose[:2], clear)

    def cancel(self, _):
        with self.lock:
            self.policy.cancel()
            self.goal = None
            self.pub.publish(Twist())

    def tick(self, _):
        now = time.monotonic()
        ros_now = rospy.Time.now().to_sec()
        with self.lock:
            policy_state = self.policy.reason(now, ros_now)
            command = self.policy.output(now, ros_now)
            if self.pose and self.cabins:
                x, y, z = self.pose[:3]
                ground = (x, y, z - self.body_height)
                exit_command = None
                if self.mission_state == 'TRACK_LEG2':
                    exit_command = limit_elevator_exit(
                        command, ground, self.lobby_planes, self.exit_speed,
                        self.corridor_speed, self.lobby_yaw_limit)
                if exit_command is None:
                    ingress = self.mission_state == 'TRACK_INGRESS' and self.ingress_goal
                    speed = self.ingress_speed if ingress else self.cabin_speed
                    command = limit_near_cabin(command, ground, self.cabins, speed, self.lobbies)
                else:
                    command = exit_command
            command, policy_state = self.align_before_exit(command, policy_state, now, ros_now)
            if (self.goal and self.pose and self.policy.active and self.policy.tracking and
                    now-self.policy.odom_at < .5 and not self.elevator_mode
                    and self.mission_state not in ('ALIGN_CABIN', 'ALIGN_EXIT')):
                have_curve = self.policy.curve_stamp >= self.policy.route_received
                heading_error = self.goal[3] - self.pose[4]
                heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
                # A marked lobby is a waiting area. Require a stationary, aligned
                # pose there before acknowledging the approach segment.
                tolerance = self.lobby_arrival_tolerance if self.lobby_goal else self.xy_tolerance
                distance = math.hypot(self.pose[0]-self.goal[0], self.pose[1]-self.goal[1])
                plane = self.lobby_planes.get(self.lobby_goal)
                clear_of_door = (plane is None or
                                 (self.pose[0] - plane[0]) * plane[2] +
                                 (self.pose[1] - plane[1]) * plane[3] >= .80)
                at_position = (distance <= tolerance
                               and abs(self.pose[2] - self.goal[2]) <= self.z_tolerance
                               and clear_of_door)
                stationary = self.pose[3] < .08
                if self.lobby_goal:
                    lobby_ready = self.mission_state == 'TRACK_LEG1' and policy_state == 'forwarding_upstream'
                    can_finish = lobby_ready and abs(heading_error) <= .10
                    if at_position and lobby_ready:
                        # The upstream controller follows spline tangents. At the
                        # marked waiting pose, finish using the published goal yaw.
                        # The Go2 accepted 0.30 rad/s in place but barely
                        # moved in the recorded approach. Use a drive floor
                        # only during this stationary, door-side alignment.
                        yaw_rate = math.copysign(
                            min(self.lobby_yaw_limit, max(.45, 1.8 * abs(heading_error))),
                            heading_error)
                        command = (0., 0., yaw_rate if stationary and not can_finish else 0.)
                        policy_state = 'aligning_to_lobby_goal'
                else:
                    can_finish = have_curve or (self.short_route and abs(heading_error) <= .15)
                close = at_position and stationary and can_finish
                self.settled_at = now if close and self.settled_at is None else self.settled_at if close else None
                if self.settled_at is not None and now-self.settled_at >= .3:
                    self.policy.cancel()
                    self.done.publish(self.route_header)
                    command = (0.,0.,0.)
            else:
                self.settled_at = None
            self.refresh_after_exit_stall(command, now, ros_now)
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = command
            self.pub.publish(msg)
            if now-self.last_status_at >= 1.:
                self.last_status_at = now
                self.status.publish(String(data=json.dumps({
                    'state': policy_state,
                    'goal_generation': self.policy.generation,
                    'forwarded_velocity': command,
                    'backend': 'original_scan',
                })))
            if (any(abs(value) > 1.e-3 for value in command) and
                    self.last_motion_generation != self.policy.generation):
                self.last_motion_generation = self.policy.generation
                rospy.loginfo("SCAN /cmd_vel forwarding started: vx=%.3f vy=%.3f wz=%.3f",
                              *command)

if __name__ == '__main__':
    rospy.init_node('scan_upstream_io')
    Adapter()
    rospy.spin()
