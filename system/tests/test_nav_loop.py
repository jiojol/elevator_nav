#!/usr/bin/env python3
"""Exercise opt-in round trips without a ROS master or hardware publishers."""
import copy
from contextlib import ExitStack
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'nav/src/map_for_plan/scripts'))
from global_planner import GlobalPlanner
from plan_graph import FreeGraph


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))


class NavLoopTest(unittest.TestCase):
    def test_global_start_uses_the_same_body_reference_as_local_planning(self):
        self.params['/body_pose_topic'] = '/test/body_reference'
        with patch('global_planner.rospy.Subscriber') as subscriber:
            GlobalPlanner(self.graph, 'map', 2.)
        self.assertIn('/test/body_reference', [call.args[0] for call in subscriber.call_args_list])

    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.now = 100.
        self.params = {'~loop_nav': True, '~loop_pause': .5}
        self.publishers = {}
        self.stack.enter_context(patch('global_planner.time.monotonic', side_effect=lambda: self.now))
        self.stack.enter_context(patch('global_planner.rospy.Time.now', side_effect=lambda: rospy.Time.from_sec(self.now)))
        self.stack.enter_context(patch('global_planner.rospy.get_param', side_effect=lambda name, default=None: self.params.get(name, default)))
        self.stack.enter_context(patch('global_planner.rospy.Publisher', side_effect=self.publisher))
        self.stack.enter_context(patch('global_planner.rospy.Subscriber'))
        self.stack.enter_context(patch('global_planner.rospy.Timer'))
        for name in ('loginfo', 'logwarn', 'logwarn_throttle'):
            self.stack.enter_context(patch('global_planner.rospy.' + name))
        self.graph = FreeGraph(np.array([[.1 + .2 * i, .1, .1] for i in range(6)]), .2)
        self.planner = GlobalPlanner(self.graph, 'map', 2.)
        self.pose = (.1, .1, .5, .25)
        self.odom()

    def publisher(self, topic, *args, **kwargs):
        self.publishers[topic] = Publisher()
        return self.publishers[topic]

    @property
    def paths(self):
        return self.publishers['/plan_map/global_path'].messages

    def odom(self, pose=None, stamp=None):
        if pose is not None:
            self.pose = pose
        x, y, z, yaw = self.pose
        msg = Odometry()
        msg.header = Header(stamp=rospy.Time.from_sec(self.now if stamp is None else stamp), frame_id='map')
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = x, y, z
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.planner._on_odom(msg)

    def goal(self, x=1.1, yaw=1.2):
        msg = PoseStamped(header=Header(stamp=rospy.Time.from_sec(self.now), frame_id='map'))
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, .1, .1
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.planner._on_goal(msg)

    def arrive(self):
        p = self.planner
        self.odom((*p.goal_xyz[:2], p.goal_xyz[2] + .4, p.goal_yaw))
        header = Header(stamp=p.active_path_stamp, frame_id='map')
        p._on_goal_reached(header)
        return header

    def tick(self, dt=.51, fresh=True):
        self.now += dt
        if fresh:
            self.odom()
        self.planner._on_goal_timer(None)

    def test_repeated_round_trips_preserve_endpoints_yaws_and_use_measured_start(self):
        self.goal()
        p = self.planner
        for leg in range(6):
            expected_x, expected_yaw = (1.1, 1.2) if leg % 2 == 0 else (.1, .25)
            self.assertAlmostEqual(p.goal_xyz[0], expected_x)
            self.assertAlmostEqual(p.goal_yaw, expected_yaw)
            previous = self.arrive()
            count = len(self.paths)
            p._on_goal_reached(previous)  # Duplicate completion while dwelling.
            self.tick(.2)
            self.assertEqual(len(self.paths), count)
            self.tick(.31)
            self.assertEqual(len(self.paths), count + 1)
            self.assertNotEqual(p.active_path_stamp, previous.stamp)
            self.assertAlmostEqual(self.paths[-1].poses[0].pose.position.x, expected_x)
            self.assertAlmostEqual(self.paths[-1].poses[-1].pose.position.z, .6)
            p._on_goal_reached(previous)  # Late completion of the previous leg.
            self.assertIsNotNone(p.active_goal)
        np.testing.assert_allclose(p.loop_points[0][0], [.1, .1, .1])
        np.testing.assert_allclose(p.loop_points[1][0], [1.1, .1, .1])

    def test_new_goal_ends_loop_and_all_later_goals_run_once(self):
        for dwelling in (False, True):
            with self.subTest(dwelling=dwelling):
                p = GlobalPlanner(self.graph, 'map', 2.)
                self.planner = p
                self.odom((.1, .1, .5, .25))
                self.goal()
                if dwelling:
                    self.arrive()
                self.now += .1
                self.goal(.7, -.6)
                self.assertIsNone(p.loop_points)
                self.assertIsNone(p.loop_due_at)
                self.assertAlmostEqual(p.goal_yaw, -.6)
                self.arrive()
                count = len(self.paths)
                self.tick(1.)
                self.assertEqual(len(self.paths), count)
                self.goal(.3, .7)
                self.arrive()
                self.tick(1.)
                self.assertIsNone(p.active_goal)
                self.assertIsNone(p.loop_points)

    def test_cancel_cannot_restart_on_fresh_pose_or_late_ack(self):
        for dwelling in (False, True):
            with self.subTest(dwelling=dwelling):
                self.planner = GlobalPlanner(self.graph, 'map', 2.)
                self.odom((.1, .1, .5, .25))
                self.goal()
                old = Header(stamp=self.planner.active_path_stamp, frame_id='map')
                if dwelling:
                    self.arrive()
                self.planner._on_cancel(None)
                count = len(self.paths)
                self.tick(2.)
                self.planner._on_goal_reached(old)
                self.assertEqual(len(self.paths), count)
                self.assertFalse(self.paths[-1].poses)
                self.assertIsNone(self.planner.loop_points)

    def test_stale_localization_waits_for_both_fresh_receipt_and_stamp(self):
        self.goal()
        self.arrive()
        count = len(self.paths)
        self.tick(1., fresh=False)
        self.assertEqual(len(self.paths), count)
        self.odom(stamp=self.now - 2.)
        self.planner._on_goal_timer(None)
        self.assertEqual(len(self.paths), count)
        self.tick(.01)
        self.assertEqual(len(self.paths), count + 1)

    def test_failure_timeout_and_reinitialization_stop_loop(self):
        for action in ('failure', 'timeout', 'initialpose'):
            with self.subTest(action=action):
                self.planner = GlobalPlanner(self.graph, 'map', 2.)
                p = self.planner
                self.odom((.1, .1, .5, .25))
                self.goal()
                if action == 'failure':
                    p._on_goal_failed(Header(stamp=p.active_path_stamp, frame_id='map'))
                elif action == 'timeout':
                    self.tick(601.)
                else:
                    msg = PoseWithCovarianceStamped()
                    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = .1, .1, .1
                    msg.pose.pose.orientation.w = 1.
                    p._on_start(msg)
                count = len(self.paths)
                self.tick(2.)
                self.assertIsNone(p.loop_points)
                self.assertIsNone(p.active_goal)
                self.assertEqual(len(self.paths), count)

    def test_wrong_goal_ack_does_not_schedule_return(self):
        self.goal()
        p = self.planner
        p._on_goal_reached(Header(stamp=rospy.Time(1), frame_id='map'))
        p._on_goal_reached(Header(stamp=p.active_path_stamp, frame_id='other'))
        self.assertIsNone(p.loop_due_at)
        self.assertIsNotNone(p.active_goal)

    def test_pending_first_goal_can_be_preempted_without_arming_new_loop(self):
        self.planner.latest_odom = None
        self.planner.start_xyz = None
        self.goal()
        self.goal(.7)
        self.odom()
        self.assertAlmostEqual(self.planner.goal_xyz[0], .7)
        self.assertIsNone(self.planner.loop_points)

    def test_default_single_goal_and_zero_distance_do_not_repeat(self):
        self.params['~loop_nav'] = False
        self.planner = GlobalPlanner(self.graph, 'map', 2.)
        self.odom()
        self.goal()
        self.arrive()
        count = len(self.paths)
        self.tick(2.)
        self.assertEqual(len(self.paths), count)
        self.params['~loop_nav'] = True
        self.planner = GlobalPlanner(self.graph, 'map', 2.)
        self.odom((.1, .1, .5, .25))
        self.goal(.1, .25)
        self.assertIsNone(self.planner.loop_points)

    def test_invalid_pause_or_planning_only_are_rejected(self):
        for pause in (0., -.1, float('nan')):
            self.params['~loop_pause'] = pause
            with self.assertRaises(ValueError):
                GlobalPlanner(self.graph, 'map', 2.)
        self.params['~loop_pause'], self.params['~planning_only'] = .5, True
        with self.assertRaises(ValueError):
            GlobalPlanner(self.graph, 'map', 2.)

    def test_reference_refresh_preserves_loop_goal_yaw_and_deadline(self):
        self.goal()
        p = self.planner
        endpoints = copy.deepcopy(p.loop_points)
        started = p.goal_started_at
        old = Header(stamp=p.active_path_stamp, frame_id='map')
        self.now += 2.1
        self.odom((.5, .1, .5, -.2))
        p._on_reference_refresh(old)
        self.assertNotEqual(p.active_path_stamp, old.stamp)
        self.assertEqual(p.goal_started_at, started)
        self.assertAlmostEqual(p.goal_yaw, 1.2)
        np.testing.assert_allclose(p.goal_xyz, [1.1, .1, .1])
        self.assertAlmostEqual(self.paths[-1].poses[0].pose.position.x, .5)
        for original, current in zip(endpoints, p.loop_points):
            np.testing.assert_allclose(original[0], current[0])
            self.assertEqual(original[1:], current[1:])
        count = len(self.paths)
        p._on_reference_refresh(old)
        p._on_goal_reached(old)
        self.assertEqual(len(self.paths), count)
        self.assertIsNotNone(p.active_goal)

    def test_start_height_alignment_is_signed_bounded_and_preserves_floor_changes(self):
        for offset, expected in ((-.1, -.1), (-.23, -.2), (.15, .15), (.8, 0.)):
            with self.subTest(offset=offset):
                self.planner = GlobalPlanner(self.graph, 'map', 2.)
                self.odom((.1, .1, .5 + offset, .25))
                self.goal()
                p = self.planner
                self.assertAlmostEqual(p.reference_start_z_bias, expected)
                self.assertAlmostEqual(self.paths[-1].poses[-1].pose.position.z, .6 + expected)
                np.testing.assert_allclose(p.goal_xyz, [1.1, .1, .1])
                np.testing.assert_allclose(self.graph.points[:, 2], .1)
        stair = np.array([[.1 + .2 * i, .1, .1 + .1 * i] for i in range(6)])
        self.graph = FreeGraph(stair, .2)
        self.planner = GlobalPlanner(self.graph, 'map', 2.)
        self.odom((.1, .1, .4, .25))
        msg = PoseStamped()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = stair[-1]
        msg.pose.orientation.w = 1.
        self.planner._on_goal(msg)
        # Smoothing densifies the path; compare its relative height profile,
        # rather than requiring the old number of voxel-centre vertices.
        current = self.planner.path_xyz
        np.testing.assert_allclose(current[:, 2]-current[0, 2],
                                   .5*(current[:, 0]-current[0, 0]), atol=1e-12)
        self.assertAlmostEqual(current[-1, 2]-current[0, 2], .5)
        np.testing.assert_allclose(self.graph.points, stair)

    def test_start_height_alignment_ignores_stale_pose_or_distant_start(self):
        self.odom((.1, .1, .4, .25), stamp=self.now - 1.)
        self.goal()
        self.assertEqual(self.planner.reference_start_z_bias, 0.)
        self.planner.latest_odom.position.y = .6
        self.planner.latest_odom_stamp = rospy.Time.from_sec(self.now)
        self.goal()
        self.assertEqual(self.planner.reference_start_z_bias, 0.)

    def test_refresh_is_bounded_and_rearmed_only_after_measured_progress(self):
        self.goal()
        p = self.planner
        for _ in range(2):
            self.now += 2.1
            self.odom()
            p._on_reference_refresh(Header(stamp=p.active_path_stamp, frame_id='map'))
        self.assertEqual(p.reference_refreshes, 2)
        count = len(self.paths)
        self.now += 2.1
        self.odom()
        p._on_reference_refresh(Header(stamp=p.active_path_stamp, frame_id='map'))
        self.assertEqual(len(self.paths), count)
        self.odom((.5, .1, .5, 0.))
        p._on_reference_refresh(Header(stamp=p.active_path_stamp, frame_id='map'))
        self.assertEqual(p.reference_refreshes, 1)
        self.assertEqual(len(self.paths), count + 1)
        self.goal(.7)
        self.assertEqual(p.reference_refreshes, 0)

    def test_refresh_rejects_stale_wrong_goal_cancelled_and_final_turn_requests(self):
        self.goal()
        p = self.planner
        current = Header(stamp=p.active_path_stamp, frame_id='map')
        count = len(self.paths)
        p._on_reference_refresh(Header(stamp=current.stamp, frame_id='other'))
        p._on_reference_refresh(Header(stamp=rospy.Time(1), frame_id='map'))
        self.now += 2.1
        p._on_reference_refresh(current)
        self.odom(stamp=self.now - 1.)
        p._on_reference_refresh(current)
        self.odom()
        p._on_goal_position_reached(current)
        p._on_reference_refresh(current)
        self.assertTrue(p.finishing_goal)
        self.assertEqual(len(self.paths), count)
        p._on_cancel(None)
        count = len(self.paths)
        p._on_reference_refresh(current)
        self.assertEqual(len(self.paths), count)


if __name__ == '__main__':
    unittest.main()
