#!/usr/bin/env python3
"""Terminal feedback regressions from MuJoCo, without a master or publishers."""
import math
from pathlib import Path
from types import SimpleNamespace
import sys
import threading
import unittest
from unittest.mock import patch

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path as NavPath

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'nav/src/dog_nav_bringup/scripts'))
from scan_command_gate import CommandGate


class TerminalFeedbackTest(unittest.TestCase):
    def setUp(self):
        self.gate = g = CommandGate.__new__(CommandGate)
        from start_alignment_policy import StartAlignmentPolicy
        g.start_alignment=StartAlignmentPolicy()
        g.phase = 'turning'
        g.goal, g.pose = (0., 0., .4, 0.), (0., 0., .4, -.05)
        g.goal_error = lambda: (.01, .01)
        g.hold_distance, g.xy_tolerance, g.z_tolerance = .35, .2, .2
        g.position_error_since = None
        g.yaw_tolerance, g.yaw_stable_time = .12, .15
        g.yaw_holding, g.yaw_stable_since = False, None
        g.turn_min, g.turn_min_effective, g.turn_max = .15, .15, .4
        g.turn_progress_since = g.turn_progress_anchor = None
        g.turn_progress_timeout, g.turn_accel, g.turn_kp = .6, .8, 1.5
        g.last_turn_speed, g.last_turn_time = 0., 0.
        g.stationary = lambda now: True
        self.commands, self.reached, self.recovery = [], [], []
        g.publisher = SimpleNamespace(publish=self.commands.append)
        g.reached_pub = SimpleNamespace(publish=self.reached.append)
        g.stop = lambda: self.commands.append(Twist())
        g.recover_position = self.recovery.append
        g.path_stamp, g.frame = rospy.Time(200), 'map'

    def test_yaw_hysteresis_holds_through_noise_and_releases_outside_tolerance(self):
        g = self.gate
        g.finish_orientation(1.)
        self.assertTrue(g.yaw_holding)
        g.pose = (0., 0., .4, -.11)
        g.finish_orientation(1.1)
        self.assertEqual(self.commands[-1].angular.z, 0.)
        self.assertFalse(self.reached)
        g.pose = (0., 0., .4, -.13)
        g.finish_orientation(1.2)
        self.assertFalse(g.yaw_holding)
        self.assertGreater(self.commands[-1].angular.z, 0.)
        self.assertFalse(self.reached)

    def test_no_yaw_progress_compensates_deadband_with_existing_limits(self):
        g = self.gate
        g.pose = (0., 0., .4, -.09)
        for i in range(1, 151):
            g.finish_orientation(i*.02)
        speeds = [x.angular.z for x in self.commands]
        self.assertGreater(g.turn_min_effective, g.turn_min)
        self.assertLessEqual(max(speeds), g.turn_max)
        self.assertLessEqual(max(abs(b-a) for a, b in zip([0.]+speeds, speeds)), g.turn_accel*.02+1e-9)
        self.assertFalse(self.reached)

    def test_yaw_hold_requires_stationary_feedback_and_original_body_tolerance(self):
        g = self.gate
        g.stationary = lambda now: False
        for now in (1., 2., 3.):
            g.finish_orientation(now)
        self.assertFalse(self.reached)
        g.stationary = lambda now: True
        g.goal_error = lambda: (.22, .01)
        g.finish_orientation(4.)
        g.finish_orientation(4.2)
        self.assertFalse(self.reached)
        self.assertEqual(len(self.recovery), 1)

    def test_cloud_age_does_not_block_execution_after_first_valid_input(self):
        g = self.gate
        g.cloud_received = None
        self.assertFalse(g.has_obstacle_input())
        g.cloud_received = 1.0
        g.cloud_stamp = rospy.Time(1)
        with patch('scan_command_gate.time.monotonic', return_value=1000.):
            self.assertTrue(g.has_obstacle_input())

    def test_tracking_endpoint_capture_requires_matching_goal_version(self):
        g = self.gate
        g.lock, g.shutting_down = threading.RLock(), False
        g.collision_stop_pending = g.require_contact_feedback = False
        g.fresh_pose = g.has_obstacle_input = lambda: True
        g.ready = lambda: False
        g.curve_ready, g.command_received = False, None
        g.goal_error = lambda: (.3, .01)
        g.tracking_goal, g.pose = (1., 0., .4), (.99, 0., .4, 0.)
        g.tracking_stamp, g.phase = rospy.Time(199), 'tracking'
        g.position_pub = SimpleNamespace(publish=lambda m: None)
        with patch('scan_command_gate.time.monotonic', return_value=1.):
            g.watchdog(None)
            self.assertEqual(g.phase, 'tracking')
            g.tracking_stamp = g.path_stamp
            g.watchdog(None)
        self.assertEqual(g.phase, 'settling')
        self.assertFalse(self.reached)


if __name__ == '__main__':
    unittest.main()
