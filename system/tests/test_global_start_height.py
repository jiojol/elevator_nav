#!/usr/bin/env python3
"""Check measured-body to ground-graph snapping without a ROS master."""
import copy
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch, Mock

import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'nav/src/map_for_plan/scripts'))
from global_planner import GlobalPlanner
from plan_graph import FreeGraph, trim_vertical_goal_tail


class GlobalStartHeightTest(unittest.TestCase):
    def setUp(self):
        self.clock = patch('global_planner.rospy.Time.now', return_value=rospy.Time(100))
        self.clock.start()
        self.addCleanup(self.clock.stop)
        self.log = patch('global_planner.rospy.loginfo')
        self.log.start()
        self.addCleanup(self.log.stop)
        self.planner = GlobalPlanner.__new__(GlobalPlanner)
        p = self.planner
        p.loop_nav = p.loop_available = False
        p._clear_loop()
        p.body_height, p.path_z_offset = .4, .1
        p.max_start_height_alignment = .2
        p.latest_odom_at = p.latest_odom_stamp = None
        p.graph = FreeGraph(np.array([[0., 0., 0.], [0., 0., .4], [1., 0., 0.]]), .2)
        p.snap_radius = 2.
        p.frame_id = 'map'
        p.lock = threading.RLock()
        p.current_floor = 'floor1'
        p._floor_for_z = lambda z: 'floor1'
        p._publish_queries = lambda stamp: None
        p._publish_status = lambda *args, **kwargs: None
        p.goal_pose_pub = SimpleNamespace(publish=lambda msg: None)
        p._try_plan = lambda: None
        p.goal_pending_for_pose = False
        p.planning_only = False
        p.latest_odom = Pose()
        p.latest_odom.position.z = .4
        p.latest_odom.orientation.z = np.sin(.3 / 2)
        p.latest_odom.orientation.w = np.cos(.3 / 2)

    @patch('global_planner.rospy.get_param', return_value='web')
    def test_cloud_map_label_survives_a_different_geometric_floor(self, _):
        import json
        p=self.planner
        context=dict(instance_id='instance',asset_id='asset',map_request_id='load',frame_id='map',ready=True,task_owner='web')
        p._cloud_context=lambda:context;p.cloud_seen=set();p.active_goal=None
        p._set_goal=Mock();p._floor_for_z=Mock(return_value='floor2')
        request=dict(request_id='request',owner='web',coordinate_frame='map',map_label='floor1',
                     pose=dict(x=.45,y=5.55,z=2.73338,qx=0,qy=0,qz=0,qw=1),
                     **{k:context[k] for k in ('instance_id','asset_id','map_request_id')})
        p._on_cloud_goal(SimpleNamespace(data=json.dumps(request)))
        args=p._set_goal.call_args.args
        self.assertEqual(args[2],'floor1');self.assertAlmostEqual(args[0][2],2.73338)
        self.assertEqual(p.cloud_request['request_id'],'request')
        p._floor_for_z.assert_not_called()

    def test_native_goal_still_infers_its_geometric_floor(self):
        p=self.planner;p.cloud_asset='';p._floor_for_z=lambda z:'floor2';p._set_goal=Mock()
        pose=PoseStamped();pose.pose.position.z=2.73338;pose.pose.orientation.w=1.
        p._on_goal(pose)
        self.assertEqual(p._set_goal.call_args.args[2],'floor2')

    def assert_ground_start(self):
        np.testing.assert_allclose(self.planner.start_xyz, [0, 0, 0])
        self.assertAlmostEqual(self.planner.start_yaw, .3)
        self.assertEqual(self.planner.latest_odom.position.z, .4)

    def test_new_goal_snaps_body_to_lower_ground_layer(self):
        pose = PoseStamped()
        pose.pose.position.x = 1.
        pose.pose.orientation.w = 1.
        self.planner._set_goal(np.array([1., 0., 0.]), 0., 'floor1', pose)
        self.assert_ground_start()
        np.testing.assert_allclose(self.planner.active_goal, [1., 0., 0.])

    def test_pending_pose_uses_same_projection(self):
        msg = Odometry()
        msg.pose.pose = copy.deepcopy(self.planner.latest_odom)
        self.planner.goal_pending_for_pose = True
        self.planner._on_odom(msg)
        self.assert_ground_start()
        self.assertEqual(msg.pose.pose.position.z, .4)

    def test_final_turn_recovery_projects_body_pose_once(self):
        p = self.planner
        p.active_goal = np.array([1., 0., 0.])
        p.active_path_stamp = rospy.Time(10)
        p.max_goal_position_recoveries, p.goal_position_recoveries = 2, 0
        p.goal_yaw = .7
        p._on_goal_position_lost(Header(stamp=rospy.Time(10), frame_id='map'))
        self.assert_ground_start()
        self.assertEqual(p.goal_yaw, .7)
        self.assertEqual(p.goal_position_recoveries, 1)

    def test_manual_ground_query_is_not_lowered_again(self):
        self.planner._set_start(np.array([0., 0., .4]), .3)
        np.testing.assert_allclose(self.planner.start_xyz, [0, 0, .4])

    def test_initial_hint_waits_for_registered_pose_in_full_navigation(self):
        p=self.planner
        msg=PoseWithCovarianceStamped()
        msg.pose.pose.position.x=1.
        p._on_start(msg)
        self.assertIsNone(p.start_xyz)
        self.assertIsNone(p.latest_odom)
        old=Odometry();old.header.stamp=rospy.Time(99)
        p._on_odom(old)
        self.assertIsNone(p.latest_odom)

    def test_planning_only_keeps_manual_initial_query(self):
        p=self.planner;p.planning_only=True
        msg=PoseWithCovarianceStamped()
        msg.pose.pose.position.x=1.
        p._on_start(msg)
        np.testing.assert_allclose(p.start_xyz,[1.,0.,0.])

    def test_nondefault_body_height_excludes_visual_offset(self):
        self.planner.body_height = .52
        self.planner.path_z_offset = .13
        self.planner.latest_odom.position.z = .52
        self.planner._set_start_from_body_pose(self.planner.latest_odom)
        np.testing.assert_allclose(self.planner.start_xyz, [0, 0, 0])
        self.assertEqual(self.planner.latest_odom.position.z, .52)

    def test_vertical_tail_uses_flat_approach_without_changing_xy(self):
        route = np.array([[-1.1, -1.1, -.5], [-1.3, -1.3, -.5],
                          [-1.5, -1.5, -.5], [-1.5, -1.5, -.3]])
        original = route.copy()
        actual = trim_vertical_goal_tail(route, .2)
        np.testing.assert_array_equal(actual, route[:-1])
        np.testing.assert_array_equal(actual[-1, :2], route[-1, :2])
        np.testing.assert_array_equal(route, original)
        downward = route.copy()
        downward[-1, 2] = -.7
        np.testing.assert_array_equal(trim_vertical_goal_tail(downward, .2), route[:-1])

    def test_goal_tail_preserves_stairs_and_other_floors(self):
        cases = [
            [[0, 0, 0], [.2, 0, .2], [.2, 0, .4]],  # Stair approach.
            [[0, 0, 0], [.2, 0, 0], [.2, 0, 1]],  # Another floor.
            [[0, 0, 0], [.2, 0, 0], [.2, 0, .2], [.2, 0, .4]],
            [[0, 0, 0], [0, 0, .2], [0, 0, .4]],  # No planar approach.
            [[0, 0, 0], [.2, 0, 0], [.4, 0, .2]],  # Real slope.
        ]
        for points in cases:
            route = np.array(points, dtype=float)
            np.testing.assert_array_equal(trim_vertical_goal_tail(route, .2), route)

    def test_global_planning_publishes_corrected_goal_height_and_original_yaw(self):
        p = self.planner
        route = np.array([[-1.1, -1.1, -.5], [-1.3, -1.3, -.5],
                          [-1.5, -1.5, -.5], [-1.5, -1.5, -.3]])
        p.graph = SimpleNamespace(resolution=.2, nearest=lambda *args: 0,
                                  astar=lambda *args: route.copy(), reference_heights=lambda points: points.copy())
        p.start_xyz, p.goal_xyz = route[0].copy(), route[-1].copy()
        p.goal_yaw, p.start_yaw = -.00531, -2.489
        p.planning_only = False
        p.active_floor = 'floor1'
        paths = []
        p.path_pub = SimpleNamespace(publish=paths.append)
        with patch('global_planner.rospy.logwarn'), patch('global_planner.smooth_free_path', side_effect=lambda path,graph:path):
            GlobalPlanner._try_plan(p)
        np.testing.assert_allclose(p.active_goal, [-1.5, -1.5, -.5])
        np.testing.assert_allclose(p.goal_xyz, p.active_goal)
        self.assertEqual(len(paths[-1].poses), 3)
        self.assertAlmostEqual(paths[-1].poses[-1].pose.position.z, 0.)
        self.assertAlmostEqual(p.path_yaws[-1], -.00531)
        np.testing.assert_array_equal(route[-1], [-1.5, -1.5, -.3])

    def test_continuous_reference_height_does_not_move_graph_query_goal(self):
        p = self.planner
        points = np.array([[.1, .1, .1], [.3, .1, .1], [.5, .1, .1]])
        p.graph = FreeGraph(points, .2, centerline=points + [0, 0, .04])
        p.start_xyz, p.goal_xyz = points[0].copy(), points[-1].copy()
        p.active_goal = p.goal_xyz.copy()
        p.goal_yaw, p.start_yaw = -.7, .3
        p.planning_only, p.active_floor = False, 'floor1'
        paths = []
        p.path_pub = SimpleNamespace(publish=paths.append)
        GlobalPlanner._try_plan(p)
        np.testing.assert_array_equal(p.goal_xyz, points[-1])
        np.testing.assert_array_equal(p.active_goal, points[-1])
        np.testing.assert_array_equal(p.graph.points, points)
        np.testing.assert_allclose(p.path_xyz[[0,-1], :2], points[[0,-1], :2])
        np.testing.assert_allclose(p.path_xyz[:,1], .1)
        self.assertAlmostEqual(paths[-1].poses[-1].pose.position.z, .64)
        self.assertAlmostEqual(p.path_yaws[-1], -.7)


if __name__ == '__main__':
    unittest.main()
