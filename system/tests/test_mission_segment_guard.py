#!/usr/bin/env python3
"""The elevator mission planner must never accept a cross-floor A* segment."""
import sys
import threading
import unittest
from pathlib import Path

import numpy as np
from geometry_msgs.msg import Pose, PoseStamped

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "nav/src/map_for_plan/scripts"))
from global_planner import GlobalPlanner


class MissionSegmentGuardTest(unittest.TestCase):
    def setUp(self):
        p = GlobalPlanner.__new__(GlobalPlanner)
        self.p = p
        p.lock = threading.RLock()
        p.loop_nav = p.loop_available = False
        p._clear_loop()
        p.planning_only = False
        p.mission_segment_only = True
        p.body_height = 0.4
        p.floors = {"floor1": {"height_z": 0.2}, "floor2": {"height_z": 4.2}}
        p.current_floor = "floor1"
        p.latest_odom = Pose()
        p.latest_odom.position.z = 0.6  # ground z=0.2
        p._set_start_from_body_pose = lambda pose: None
        p._publish_status = lambda *args, **kwargs: None
        p._publish_queries = lambda stamp: None
        p.failures = []
        p._fail_goal = p.failures.append
        p.goal_position_recoveries = 0
        p.reference_refreshes = 0
        p.reference_refresh_origin = None
        p.reference_refreshed_at = None
        p.finishing_goal = False
        p.active_goal = None
        p.active_path_stamp = None
        p.goal_pending_for_pose = False
        p.scan_ack = False
        p.path_xyz = p.path_yaws = None
        p.classes = []

    def test_cross_floor_segment_is_rejected_before_graph_query(self):
        p = self.p
        p.graph = type("Graph", (), {
            "nearest": lambda *args: p.classes.append("nearest")
        })()
        p.goal_pose_pub = type("Pub", (), {"publish": lambda *args: None})()
        msg = PoseStamped()
        msg.pose.orientation.w = 1.0
        p._set_goal(
            np.array([1.0, 1.0, 4.2]), 0.0, "floor2", msg
        )
        self.assertEqual(len(p.failures), 1)
        self.assertIn("cross-floor segment rejected", p.failures[0])
        self.assertEqual(p.classes, [])


if __name__ == "__main__":
    unittest.main()
