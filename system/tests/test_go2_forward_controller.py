#!/usr/bin/env python3
"""Exercise the compiled forward controller on an isolated ROS master."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import xmlrpc.client


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--devel', type=Path, default=root / 'nav/devel')
    devel = parser.parse_args().devel.resolve()
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
    os.environ['ROS_HOSTNAME'] = '127.0.0.1'
    os.environ.pop('ROS_IP', None)
    os.environ.pop('ROS_NAMESPACE', None)
    logs = Path(tempfile.mkdtemp(prefix='dog-nav-forward-controller-'))
    processes, handles = [], []

    def launch(name, command):
        handle = (logs / (name + '.log')).open('w')
        handles.append(handle)
        proc = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(proc)

    try:
        launch('master', ['roscore', '-p', str(port)])
        master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        deadline = time.monotonic() + 15
        while True:
            try:
                if master.getPid('/forward_test')[0] == 1:
                    break
            except OSError:
                pass
            assert time.monotonic() < deadline
            time.sleep(.05)

        import rospy
        from geometry_msgs.msg import Point, PoseStamped, Twist
        from nav_msgs.msg import Odometry, Path as RosPath
        from scan_planner.msg import Bspline
        from std_msgs.msg import Bool, String
        rospy.init_node('forward_test', disable_signals=True)
        params = dict(time_forward=.8, heading_error_threshold=.65, rotate_exit_threshold=.25,
                      kp_pos=.8, kp_yaw=1.5, max_vx=.2, max_vy=0., max_vyaw=.4, finish_dist=.15)
        rospy.set_param('/closed_loop_controller', params)
        rospy.set_param('/body_pose_topic', '/test/odom')
        commands, states, frozen = [], [], []
        rospy.Subscriber('/test/cmd', Twist, lambda msg: commands.append((time.monotonic(), msg)))
        rospy.Subscriber('/planning/tracking_status', String, lambda msg: states.append(json.loads(msg.data)))
        rospy.Subscriber('/planning/go2_execution_frozen', Bool, lambda msg: frozen.append(msg.data))
        odom = rospy.Publisher('/test/odom', Odometry, queue_size=1)
        paths = rospy.Publisher('/initial_path', RosPath, queue_size=1)
        curves = rospy.Publisher('/planning/bspline', Bspline, queue_size=1)
        launch('controller', [str(devel / 'lib/scan_planner/closed_loop_controller'), 'cmd_vel:=/test/cmd'])
        deadline = time.monotonic() + 10
        while not (odom.get_num_connections() and paths.get_num_connections() and curves.get_num_connections()):
            assert all(proc.poll() is None for proc in processes) and time.monotonic() < deadline
            time.sleep(.02)

        def feed(duration, yaw=0., position=0., position_y=0.):
            end = time.monotonic() + duration
            while time.monotonic() < end:
                msg = Odometry()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = 'map'
                msg.pose.pose.position = Point(position, position_y, .4)
                msg.pose.pose.orientation.z = math.sin(yaw / 2.)
                msg.pose.pose.orientation.w = math.cos(yaw / 2.)
                odom.publish(msg)
                time.sleep(.02)

        def reference(empty=False):
            msg = RosPath()
            msg.header.stamp, msg.header.frame_id = rospy.Time.now(), 'map'
            if not empty:
                for x in (0., 2.):
                    pose = PoseStamped(header=msg.header)
                    pose.pose.position = Point(x, 0., .4)
                    pose.pose.orientation.w = 1.
                    msg.poses.append(pose)
            paths.publish(msg)
            feed(.10)

        def spline(identifier):
            msg = Bspline(order=3, traj_id=identifier, start_time=rospy.Time.now())
            msg.pos_pts = [Point((i - 1) * .2, 0., .4) for i in range(9)]
            msg.knots = [float(i) for i in range(-3, 10)]
            curves.publish(msg)
            return msg

        feed(.3)
        assert commands and all(msg == Twist() for _, msg in commands)
        reference()
        begin = len(commands)
        spline(1)
        feed(3.)
        tracked = commands[begin:]
        assert tracked and max(msg.linear.x for _, msg in tracked) > .19
        assert all(0. <= msg.linear.x <= .2 and msg.linear.y == 0. and abs(msg.angular.z) <= .4 for _, msg in tracked)
        # Deliberately stationary odometry: execution holds at 0.20 m of lead,
        # while bounded forward commands continue instead of pulsing on/off.
        assert states[-1]['frozen'] and states[-1]['mode'] == 'tracking_with_clock_held'
        assert .8 <= states[-1]['execution_time'] <= 1.05, states[-1]
        assert all(msg.linear.x > .19 for _, msg in tracked[-30:])
        for (t0, a), (t1, b) in zip(tracked, tracked[1:]):
            assert abs(a.linear.x - b.linear.x) <= .25 * (t1 - t0 + .035) + 1e-4
        print('PASS: no premature motion, forward-only speed/ramp limits and lagging-body clock hold', flush=True)

        # An accepted replacement segment preserves the existing ramp/output.
        begin = len(commands)
        spline(2)
        feed(.35)
        assert commands[begin:] and all(msg.linear.x > .19 for _, msg in commands[begin:])
        print('PASS: rolling spline handoff preserves motion continuity', flush=True)

        feed(1.4, yaw=.70)
        assert states[-1]['mode'] == 'turning'
        assert commands[-1][1].linear.x == 0. and commands[-1][1].angular.z < -.3
        for yaw in (.64, .66, .64, .66):
            feed(.3, yaw=yaw)
            assert states[-1]['mode'] == 'turning' and commands[-1][1].linear.x == 0.
        feed(.6, yaw=.1)
        assert states[-1]['mode'] != 'turning' and commands[-1][1].linear.x > .01
        print('PASS: heading hysteresis prevents repeated translation/turning switches; no strafing', flush=True)

        time.sleep(.75)
        assert states[-1]['mode'] == 'stale_odometry' and commands[-1][1] == Twist()
        feed(.5)
        assert commands[-1][1].linear.x > .01
        reference(empty=True)
        spline(3)  # A late curve cannot reactivate a cleared goal.
        feed(.4)
        assert commands[-1][1] == Twist()
        reference()
        bad = spline(4)
        feed(.3)
        assert commands[-1][1].linear.x > .01
        bad.knots[4] = bad.knots[3]
        curves.publish(bad)
        feed(.3)
        assert commands[-1][1] == Twist()
        assert all(msg.linear.y == 0. for _, msg in commands)
        print('PASS: stale odometry, canceled goals and malformed splines stop immediately', flush=True)

        # Follow a genuine curved spline with a kinematic forward/yaw body.
        # This catches steering/clock interactions absent from straight tests.
        reference()
        corner = Bspline(order=3, traj_id=5, start_time=rospy.Time.now())
        corner.pos_pts = [Point((i - 1) * .2, 0., .4) for i in range(7)]
        corner.pos_pts += [Point(1., y, .4) for y in (.2, .4, .6, .8)]
        corner.knots = [float(i) for i in range(-3, 12)]
        curves.publish(corner)
        px, py, yaw, max_error = 0., 0., 0., 0.
        previous = time.monotonic()
        deadline = previous + 25.
        while time.monotonic() < deadline:
            now = time.monotonic()
            dt, previous = min(.05, now - previous), now
            cmd = commands[-1][1]
            yaw += cmd.angular.z * dt
            px += cmd.linear.x * math.cos(yaw) * dt
            py += cmd.linear.x * math.sin(yaw) * dt
            # Distance to the two arms of the reference L, in meters.
            horizontal = math.hypot(px - max(0., min(1., px)), py)
            vertical = math.hypot(px - 1., py - max(0., min(.6, py)))
            max_error = max(max_error, min(horizontal, vertical))
            feed(.02, yaw=yaw, position=px, position_y=py)
            if states[-1]['mode'] == 'segment_finished':
                break
        assert states[-1]['mode'] == 'segment_finished', (px, py, yaw, states[-1])
        assert frozen and frozen[-1] is False, 'completed segment left continuation clock frozen'
        assert math.hypot(px - 1., py - .6) < .15 and max_error < .30, (px, py, max_error)
        assert all(msg.linear.y == 0. for _, msg in commands)
        print('PASS: curved spline completes using forward/yaw only; max L-route error %.3f m' % max_error, flush=True)
    finally:
        if 'rospy' in locals():
            rospy.signal_shutdown('isolated controller checks complete')
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=3)
        for handle in handles:
            handle.close()
        print('Logs:', logs, flush=True)


if __name__ == '__main__':
    main()
