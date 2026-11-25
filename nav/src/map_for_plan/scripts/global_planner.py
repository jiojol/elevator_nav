#!/usr/bin/env python3
"""在单层 FREE 体素上做快速全局规划。

起点：/initialpose（3D Pose Estimate 或 2D Pose Estimate）
终点：/move_base_simple/goal（3D Nav Goal 或 2D Nav Goal）
结果：/plan_map/global_path（机身参考高度）
"""

from __future__ import annotations

import argparse
import json
from uuid import uuid4
import math
import sys
import threading
import time
from pathlib import Path

_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from std_msgs.msg import Empty, Header, String
from visualization_msgs.msg import Marker

from cloud_contract import validate_request
from plan_graph import FreeGraph, trim_vertical_goal_tail
from path_smoothing import smooth_free_path
from plan_io import load_plan_meta, read_pcd_xyz


def _xyz_from_pose(pose) -> np.ndarray:
    return np.array([pose.position.x, pose.position.y, pose.position.z], dtype=np.float64)


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _yaws_along(points: np.ndarray, start_yaw: float, goal_yaw: float) -> np.ndarray:
    yaws = np.zeros(len(points), dtype=np.float64)
    for i in range(len(points)):
        if i + 1 < len(points):
            dx = points[i + 1, 0] - points[i, 0]
            dy = points[i + 1, 1] - points[i, 1]
            if dx * dx + dy * dy > 1.0e-8:
                yaws[i] = math.atan2(dy, dx)
                continue
        yaws[i] = yaws[i - 1] if i else start_yaw
    if len(points):
        yaws[0] = start_yaw
        yaws[-1] = goal_yaw
    return yaws


def to_path(points: np.ndarray, frame_id: str, stamp, yaws=None) -> PathMsg:
    path = PathMsg()
    path.header = Header(stamp=stamp, frame_id=frame_id)
    for i, (x, y, z) in enumerate(points):
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        if yaws is None:
            pose.pose.orientation.w = 1.0
        else:
            yaw = float(yaws[i])
            pose.pose.orientation.z = math.sin(yaw * 0.5)
            pose.pose.orientation.w = math.cos(yaw * 0.5)
        path.poses.append(pose)
    return path


def ball(marker_id, xyz, rgba, frame_id, stamp, scale=0.7) -> Marker:
    marker = Marker()
    marker.header = Header(stamp=stamp, frame_id=frame_id)
    marker.ns = "plan_query"
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, xyz)
    marker.pose.orientation.w = 1.0
    marker.scale.x = marker.scale.y = marker.scale.z = scale
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
    return marker


def continuous_goal_path(points, goal, resolution):
    """Keep a requested position inside the selected FREE voxel, including Z."""
    points = np.asarray(points, dtype=float)
    if not len(points):
        return points.copy()
    goal = np.asarray(goal, dtype=float)
    if np.linalg.norm(points[-1, :2] - goal[:2]) > resolution / np.sqrt(2) + 1e-5:
        raise ValueError("Requested goal lies outside the reachable FREE cell")
    # FREE heights are voxelized, while a scene click hits the rendered
    # surface. Retain that exact goal within one 10 cm-or-smaller height cell.
    height_tolerance = min(0.1, max(0.05, resolution))
    if abs(points[-1, 2] - goal[2]) > height_tolerance:
        raise ValueError("Requested goal is on a different support height")
    result = points.copy()
    if len(result) == 1 and np.linalg.norm(result[-1] - goal) > 1e-6:
        result = np.vstack([result, goal])
    else:
        result[-1] = goal
    return result


class GlobalPlanner:
    def __init__(self, graph: FreeGraph, frame_id: str, snap_radius: float) -> None:
        self.graph = graph
        self.frame_id = frame_id
        self.snap_radius = snap_radius
        self.lock = threading.RLock()
        self.planning_only = bool(rospy.get_param("~planning_only", False))
        self.manual_start = self.planning_only and bool(rospy.get_param("~manual_start", False))
        self.mission_segment_only = bool(rospy.get_param("~mission_segment_only", False))
        self.elevator = rospy.get_param("~elevator", {}) if self.mission_segment_only else {}
        self.loop_nav = bool(rospy.get_param("~loop_nav", False))
        self.loop_available = self.loop_nav
        self.loop_pause = float(rospy.get_param("~loop_pause", 0.5))
        if not math.isfinite(self.loop_pause) or self.loop_pause < 0.1:
            raise ValueError("loop_pause must be finite and at least 0.1 seconds")
        if self.loop_nav and self.planning_only:
            raise ValueError("loop_nav requires full navigation with controller arrival acknowledgements")
        self._clear_loop()
        self.floors = rospy.get_param("~floors", {})
        self.current_floor = str(rospy.get_param("~initial_floor", "floor1"))
        self.body_height = float(rospy.get_param("~body_height", 0.40))
        self.path_z_offset = float(rospy.get_param("~path_z_offset", 0.10))
        if not math.isfinite(self.path_z_offset) or self.path_z_offset < 0:
            raise ValueError("path_z_offset must be finite and nonnegative")
        self.goal_timeout = float(rospy.get_param("~goal_timeout", 600.0))
        self.max_goal_position_recoveries = int(rospy.get_param("~max_goal_position_recoveries", 2))
        if self.max_goal_position_recoveries < 0:
            raise ValueError("max_goal_position_recoveries must be nonnegative")
        self.goal_position_recoveries = 0
        self.max_reference_refreshes = int(rospy.get_param("~max_reference_refreshes", 2))
        self.reference_refresh_interval = float(rospy.get_param("~reference_refresh_interval", 2.0))
        if (self.max_reference_refreshes < 0 or not math.isfinite(self.reference_refresh_interval)
                or self.reference_refresh_interval < 0.5):
            raise ValueError("invalid bounded reference refresh settings")
        self.reference_refreshes = 0
        self.reference_refresh_origin = None
        self.reference_refreshed_at = None
        self.finishing_goal = False
        self.max_start_height_alignment = float(rospy.get_param("~max_start_height_alignment", 0.20))
        if not math.isfinite(self.max_start_height_alignment) or not 0.0 <= self.max_start_height_alignment <= 0.20:
            raise ValueError("max_start_height_alignment must be within [0, 0.20] metres")
        self.reference_start_z_bias = 0.0
        self.start_xyz = None
        self.goal_xyz = None
        self.requested_goal_xyz = None
        self.start_yaw = 0.0
        self.goal_yaw = 0.0
        self.path_xyz = None
        self.path_yaws = None
        self.latest_odom = None
        self.latest_odom_at = None
        self.odom_receipt = None
        self.latest_odom_stamp = None
        self.active_goal = None
        self.active_floor = ""
        self.goal_started_at = None
        self.active_path_stamp = None
        self.goal_pending_for_pose = False
        self.waiting_for_map_recovery = False
        self.scan_ack = False

        self.cloud_request = None
        self.cloud_seen = set()
        self.cloud_instance = uuid4().hex
        self.cloud_map_request = uuid4().hex
        self.cloud_asset = rospy.get_param('/integration/navigation_asset_id', '')
        self.cloud_localization = ''
        self.cloud_localization_at = 0.
        rospy.Subscriber('/dog_nav/localization_status', String, self._cloud_localization, queue_size=1)
        rospy.Subscriber('/map_manager/goal_request', String, self._on_cloud_goal, queue_size=1)
        self.path_pub = rospy.Publisher("/plan_map/global_path", PathMsg, queue_size=1, latch=True)
        self.start_pub = rospy.Publisher("/plan_map/query_start", Marker, queue_size=1, latch=True)
        self.goal_pub = rospy.Publisher("/plan_map/query_goal", Marker, queue_size=1, latch=True)
        self.goal_pose_pub = rospy.Publisher("/dog_nav/goal_pose", PoseStamped, queue_size=1, latch=True)
        self.current_map_pub = rospy.Publisher("/map_manager/current_map", String, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher("/map_manager/status", String, queue_size=10, latch=True)
        self.result_pub = rospy.Publisher("/map_manager/result", String, queue_size=10)
        if self.planning_only and self.mission_segment_only:
            self.ingress_preview_pub = rospy.Publisher(
                "/plan_map/ingress_preview", PathMsg, queue_size=1, latch=True)
            self.preview_path_pub = rospy.Publisher(
                "/plan_map/destination_preview", PathMsg, queue_size=1, latch=True)
            self.final_goal_marker_pub = rospy.Publisher(
                "/plan_map/final_goal_marker", Marker, queue_size=1, latch=True)
            rospy.Subscriber("/elevator_nav/preview_request", PathMsg,
                             self._on_preview_request, queue_size=1)
        self.cancel_scan_pub = rospy.Publisher("/plan_map/cancel_scan", Empty, queue_size=1)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self._on_start, queue_size=1)
        if not self.mission_segment_only:
            rospy.Subscriber("/move_base_simple/goal", PoseStamped, self._on_goal, queue_size=1)
        # A cross-floor mission manager owns final goals and emits only
        # same-floor, executable segments here. Keeping this topic separate
        # prevents a final floor-2 goal from being consumed by A* on floor 1.
        rospy.Subscriber("/plan_map/segment_goal", PoseStamped, self._on_goal, queue_size=1)
        rospy.Subscriber("/plan_map/start_pose", PoseStamped, self._on_start_pose, queue_size=1)
        rospy.Subscriber("/plan_map/goal_pose", PoseStamped, self._on_goal, queue_size=1)
        odom_topic = ("/dog_nav/odometry" if self.planning_only else
                      rospy.get_param("/body_pose_topic", "/planning/body_odometry"))
        if not self.manual_start:
            rospy.Subscriber(odom_topic, Odometry, self._on_odom, queue_size=20)
        if not self.planning_only:
            if rospy.get_param("~local_planner", "modified") == "upstream":
                from scan_upstream_planner.msg import Bspline
            else:
                from scan_planner.msg import Bspline
            rospy.Subscriber("/planning/bspline", Bspline, self._on_scan_trajectory, queue_size=1)
        rospy.Subscriber("/map_manager/cancel", Empty, self._on_cancel, queue_size=1)
        rospy.Subscriber("/planning/goal_reached", Header, self._on_goal_reached, queue_size=10)
        rospy.Subscriber("/planning/goal_failed", Header, self._on_goal_failed, queue_size=10)
        rospy.Subscriber("/planning/goal_position_lost", Header, self._on_goal_position_lost, queue_size=10)
        rospy.Subscriber("/planning/reference_refresh", Header, self._on_reference_refresh, queue_size=1)
        rospy.Subscriber("/planning/goal_position_reached", Header, self._on_goal_position_reached, queue_size=10)
        for floor_label, config in sorted(self.floors.items()):
            topic = str(config.get("goal_topic", "")).strip()
            if topic:
                rospy.Subscriber(topic, PoseStamped, self._on_floor_goal,
                                 callback_args=floor_label, queue_size=1)
        self.goal_timer = rospy.Timer(rospy.Duration(0.2), self._on_goal_timer)
        self.current_map_pub.publish(String(data=self.current_floor))
        self._publish_status("IDLE", "FREE-space planning ready")

    def _on_preview_request(self, request: PathMsg) -> None:
        if request.header.frame_id != self.frame_id or len(request.poses) not in (0, 1, 4):
            return
        stamp = request.header.stamp
        empty_path = PathMsg(header=request.header)
        if len(request.poses) < 4:
            self.ingress_preview_pub.publish(empty_path)
        if not request.poses:
            marker = ball(2, np.zeros(3), (0.0, 1.0, 0.3, 1.0), self.frame_id, stamp)
            marker.action = Marker.DELETE
            self.final_goal_marker_pub.publish(marker)
            self.preview_path_pub.publish(empty_path)
            return

        goal = _xyz_from_pose(request.poses[-1].pose)
        if not np.isfinite(goal).all():
            return
        if len(request.poses) == 1:
            self.publish_preview_goal(goal, stamp)
            self.preview_path_pub.publish(empty_path)
            return

        lobby, ingress, destination_cabin, target = (item.pose for item in request.poses)
        try:
            path = self.plan_ingress_preview(lobby, ingress)
            self.publish_preview_path(self.ingress_preview_pub, path, lobby, ingress, stamp)
        except ValueError as error:
            self.ingress_preview_pub.publish(empty_path)
            rospy.logwarn("入舱预览失败: %s", error)

        try:
            path, endpoint = self.plan_destination_preview(
                _xyz_from_pose(destination_cabin), goal)
        except ValueError as error:
            self.publish_preview_goal(goal, stamp, reachable=False)
            self.preview_path_pub.publish(empty_path)
            rospy.logwarn("目标层预览失败: %s", error)
            return
        self.publish_preview_goal(endpoint, stamp)
        self.publish_preview_path(self.preview_path_pub, path, destination_cabin, target, stamp)
        rospy.loginfo("目标层预览: 轿厢到最终目标，%d 点", len(path))

    def publish_preview_path(self, publisher, path, start, goal, stamp) -> None:
        path[:, 2] += self.body_height + self.path_z_offset
        if len(path) == 1:
            path = np.repeat(path, 2, axis=0)
        yaws = _yaws_along(path, _yaw_from_quat(start.orientation),
                           _yaw_from_quat(goal.orientation))
        publisher.publish(to_path(path, self.frame_id, stamp, yaws))

    def plan_ingress_preview(self, lobby, cabin):
        positions = [_xyz_from_pose(pose) for pose in (lobby, cabin)]
        graph = self.graph
        indices = [graph.nearest(xyz, self.snap_radius) for xyz in positions]
        if any(index is None for index in indices):
            raise ValueError("候梯点或轿厢点超出 FREE 地图")
        if self._floor_for_z(positions[0][2]) != self._floor_for_z(positions[1][2]):
            raise ValueError("候梯点和轿厢点必须在同一楼层")
        for xyz, index in zip(positions, indices):
            continuous_goal_path(graph.points[index:index + 1], xyz, graph.resolution)
        path = self.destination_preview_path(*indices, positions[1])
        if path is None:
            raise ValueError("候梯点与轿厢点之间无可通行路径")
        return continuous_goal_path(path[::-1], positions[0], graph.resolution)[::-1].copy()

    def publish_preview_goal(self, goal, stamp, reachable=True) -> None:
        color = (0.0, 1.0, 0.3, 1.0) if reachable else (1.0, 0.2, 0.1, 1.0)
        self.final_goal_marker_pub.publish(ball(
            2, goal + [0.0, 0.0, 0.30], color, self.frame_id, stamp))

    def plan_destination_preview(self, cabin, goal):
        graph = self.graph
        target_floor = self._floor_for_z(goal[2])
        start_index = graph.nearest(cabin, self.snap_radius)
        if start_index is None or self._floor_for_z(graph.points[start_index, 2]) != target_floor:
            raise ValueError("目标层轿厢点无法吸附到同层 FREE 地图")

        exact = graph.nearest(goal, self.snap_radius)
        if exact is not None:
            exact, exact_goal = self._snap_mission_goal_height(exact, goal, target_floor)
            try:
                continuous_goal_path(graph.points[exact:exact + 1], exact_goal, graph.resolution)
                path = self.destination_preview_path(start_index, exact, exact_goal)
                if path is not None:
                    return path, exact_goal
            except ValueError:
                pass

        # RViz may pick a wall return above the desired ground. Project only
        # to nearby, reachable ground on the requested floor in this preview.
        floor_height = float(self.floors[target_floor]["height_z"])
        xy = np.linalg.norm(graph.points[:, :2] - goal[:2], axis=1)
        dz = np.abs(graph.points[:, 2] - goal[2])
        nearby = np.flatnonzero(
            (xy <= 0.75) & (dz <= 2.50) &
            (np.abs(graph.points[:, 2] - floor_height) <= 0.75))
        for index in nearby[np.lexsort((dz[nearby], xy[nearby]))]:
            endpoint = graph.points[int(index)]
            if self._floor_for_z(endpoint[2]) != target_floor:
                continue
            try:
                path = self.destination_preview_path(start_index, int(index), endpoint)
            except ValueError:
                continue
            if path is not None:
                rospy.loginfo(
                    "目标层预览吸附至 FREE (%.2f, %.2f, %.2f)，水平偏移 %.2f m，高差 %.2f m",
                    *endpoint, xy[index], dz[index])
                return path, endpoint
        raise ValueError("点击点附近没有从轿厢可达的同层 FREE 网格，请在 FREE 地面重新选点")

    def destination_preview_path(self, start_index, goal_index, endpoint):
        graph = self.graph
        floor = self._floor_for_z(endpoint[2])
        path, _ = self._graph_route(start_index, goal_index, floor)
        if path is None:
            return None
        return continuous_goal_path(path, endpoint, graph.resolution)

    def _exit_waypoints(self, start, goal, floor):
        """Keep the exit reference on the marked doorway centerline."""
        elevator = getattr(self, "elevator", {})
        stop = elevator.get("floors", {}).get(floor, {})
        if not elevator.get("configured") or "door_center_xy" not in stop:
            return ()
        cabin = np.asarray(stop["inside_pose"][:3], dtype=float)
        center = np.asarray(stop["door_center_xy"], dtype=float)
        normal = center - cabin[:2]
        length = np.linalg.norm(normal)
        if length < .05:
            raise ValueError("invalid elevator door center")
        normal /= length
        start_offset = start[:2] - center
        cross_track = np.dot(start_offset, [-normal[1], normal[0]])
        if (np.dot(start_offset, normal) > .05 or np.dot(goal[:2] - center, normal) <= .35
                or np.linalg.norm(start[:2] - cabin[:2]) > 1.0
                or abs(cross_track) > float(elevator["door_width"]) * .5):
            return ()
        return (center, center + .35 * normal)

    def _graph_route(self, start, goal, floor):
        graph = self.graph
        waypoints = self._exit_waypoints(graph.points[start], graph.points[goal], floor)
        nodes = [start]
        for xy in waypoints:
            probe = np.array([xy[0], xy[1], graph.points[start, 2]])
            index = graph.nearest(probe, graph.resolution / np.sqrt(2) + 1e-6)
            if (index is None or self._floor_for_z(graph.points[index, 2]) != floor
                    or abs(graph.points[index, 2] - probe[2]) > .20 + 1e-6):
                raise ValueError("configured elevator exit centerline has no same-floor FREE support")
            nodes.append(index)
        nodes.append(goal)
        parts = []
        for source, target in zip(nodes, nodes[1:]):
            part = graph.astar(source, target)
            if part is None:
                return None, None
            if target == goal:
                part = trim_vertical_goal_tail(part, graph.resolution)
            parts.append(smooth_free_path(graph.reference_heights(part), graph))
        path = np.concatenate([parts[0]] + [part[1:] for part in parts[1:]])
        if waypoints:
            rospy.loginfo("Elevator exit reference follows doorway center at x=%.2f y=%.2f",
                          *graph.points[nodes[1], :2])
        return path, part[-1]

    def _on_odom(self, msg: Odometry) -> None:
        after = getattr(self, 'initial_pose_after', None)
        if after is not None and msg.header.stamp <= after:
            return
        # Receipt must not wait behind graph search. Otherwise search itself
        # makes live localization look stale and the bridge cancels its goal.
        # Store the source stamp unchanged; deferred floor/start work runs on
        # the next observation after the planner releases its state lock.
        received=time.monotonic()
        self.odom_receipt=(msg.header.stamp,received)
        if not self.lock.acquire(blocking=False):
            return
        try:
            after = getattr(self, 'initial_pose_after', None)
            if after is not None and msg.header.stamp <= after:
                return
            self.latest_odom = msg.pose.pose
            self.latest_odom_at = received
            self.latest_odom_stamp = msg.header.stamp
            floor = self._floor_for_z(msg.pose.pose.position.z - self.body_height)
            if floor and floor != self.current_floor:
                self.current_floor = floor
                self.current_map_pub.publish(String(data=floor))
            if self.goal_pending_for_pose:
                if self.waiting_for_map_recovery:
                    if self.cloud_localization != 'TRACKING' or not self._has_fresh_odom():
                        return
                    if floor != self.active_floor:
                        self._fail_goal('recovered pose is on a different floor from the active segment')
                        return
                self._set_start_from_body_pose(self.latest_odom)
                if not self.goal_pending_for_pose:
                    self.waiting_for_map_recovery = False
        finally:
            self.lock.release()

    def _on_goal_reached(self, msg: Header) -> None:
        with self.lock:
            # Only the controller knows when both position and final yaw are
            # satisfied. A delayed event from an old goal must not finish a new one.
            if (self.active_goal is None or self.active_path_stamp is None
                    or msg.stamp != self.active_path_stamp or msg.frame_id != self.frame_id):
                return
            target_floor = self.active_floor
            self.active_goal = None
            self.active_path_stamp = None
            self.goal_started_at = None
            self.current_floor = target_floor or self.current_floor
            self.current_map_pub.publish(String(data=self.current_floor))
            if self.loop_points is not None:
                self.loop_target_index = 1 - self.loop_target_index
                self.loop_due_at = time.monotonic() + self.loop_pause
                self._publish_status("LOOP_WAIT", "arrived; waiting for next loop leg", target_map=target_floor)
            else:
                self._publish_status("IDLE", "arrived", target_map=target_floor)
            self._publish_result("succeeded", "arrived", target_floor)

    def _on_goal_failed(self, msg: Header) -> None:
        with self.lock:
            if (self.active_goal is None or self.active_path_stamp is None
                    or msg.stamp != self.active_path_stamp or msg.frame_id != self.frame_id):
                return
            self._fail_goal("navigation controller/planner reported failure")

    def _on_goal_position_lost(self, msg: Header) -> None:
        with self.lock:
            if (self.active_goal is None or self.active_path_stamp is None
                    or msg.stamp != self.active_path_stamp or msg.frame_id != self.frame_id):
                return
            if self.goal_position_recoveries >= self.max_goal_position_recoveries:
                self._fail_goal("final turn repeatedly displaced goal position")
                return
            if self.latest_odom is None:
                self._fail_goal("no measured pose for final turn position recovery")
                return
            self.goal_position_recoveries += 1
            self.scan_ack = False
            # Replan on the real graph from measured pose, preserving the user's
            # goal and yaw. A new path stamp rejects late pre-recovery events.
            self.active_path_stamp = None
            self._set_start_from_body_pose(self.latest_odom)
            if self.start_xyz is None:
                self._fail_goal("final turn position is outside the traversable map")
                return
            rospy.logwarn("Final turn displaced position; checked replanning attempt %d/%d",
                          self.goal_position_recoveries, self.max_goal_position_recoveries)
            self._try_plan()

    def _on_goal_position_reached(self, msg: Header) -> None:
        with self.lock:
            if (self.active_goal is not None and self.active_path_stamp is not None
                    and msg.stamp == self.active_path_stamp and msg.frame_id == self.frame_id):
                self.finishing_goal = True

    def _has_fresh_odom(self) -> bool:
        if self.latest_odom is None or self.latest_odom_at is None or self.latest_odom_stamp is None:
            return False
        age = (rospy.Time.now() - self.latest_odom_stamp).to_sec()
        return time.monotonic() - self.latest_odom_at <= 0.5 and -0.1 <= age <= 0.5

    def _on_reference_refresh(self, msg: Header) -> None:
        with self.lock:
            if (self.planning_only or self.finishing_goal or self.active_goal is None or self.active_path_stamp is None
                    or msg.stamp != self.active_path_stamp or msg.frame_id != self.frame_id
                    or not self._has_fresh_odom()):
                return
            now = time.monotonic()
            if self.goal_started_at is not None and now - self.goal_started_at > self.goal_timeout:
                self._fail_goal("navigation timeout")
                return
            if self.reference_refreshed_at is not None and now - self.reference_refreshed_at < self.reference_refresh_interval:
                return
            position = _xyz_from_pose(self.latest_odom)
            if (self.reference_refresh_origin is not None
                    and np.linalg.norm(position - self.reference_refresh_origin) >= 0.30):
                self.reference_refreshes = 0
                self.reference_refresh_origin = None
            if self.reference_refreshes >= self.max_reference_refreshes:
                rospy.logwarn_throttle(10., "Reference refresh limit at this position reached; keep collision checks and wait for a safe local path.")
                return
            if self.reference_refresh_origin is None:
                self.reference_refresh_origin = position.copy()
            self.reference_refreshes += 1
            self.reference_refreshed_at = now
            self.scan_ack = False
            self.active_path_stamp = None
            self._set_start_from_body_pose(self.latest_odom)
            if self.start_xyz is None:
                self._fail_goal("no traversable graph start for stalled reference recovery")
                return
            # Same user goal, yaw, loop and deadline; only the checked graph
            # route and its identity are refreshed, as with a measured restart.
            rospy.logwarn("Local replanning stalled; refresh graph route from measured pose (%d/%d at this position).",
                          self.reference_refreshes, self.max_reference_refreshes)
            self._try_plan()

    def _on_start(self, msg: PoseWithCovarianceStamped) -> None:
        with self.lock:
            if not self.planning_only:
                # A clicked surface is only a registration hint, not verified
                # robot odometry. Do not advertise a usable graph start yet.
                if getattr(self, 'active_goal', None) is not None or self.loop_points is not None:
                    self._on_cancel(Empty())
                self.initial_pose_after = rospy.Time.now()
                self.latest_odom = self.latest_odom_at = self.latest_odom_stamp = None
                self.start_xyz = None
                self._clear_loop()
                rospy.loginfo("已提交定位初值；等待点云配准验证后的机身位置，再吸附导航起点。")
                return
            if self.loop_points is not None:
                self._on_cancel(Empty())
            self._set_start(_xyz_from_pose(msg.pose.pose), _yaw_from_quat(msg.pose.pose.orientation))

    def _on_start_pose(self, msg: PoseStamped) -> None:
        with self.lock:
            if self.loop_points is not None:
                self._on_cancel(Empty())
            self._set_start(_xyz_from_pose(msg.pose), _yaw_from_quat(msg.pose.orientation))

    def _cloud_localization(self, msg):
        with self.lock:
            self.cloud_localization = msg.data
            self.cloud_localization_at = time.monotonic()
            if (msg.data == 'LOST_MAP_MATCH' and self.mission_segment_only
                    and not self.planning_only and self.active_goal is not None
                    and self.active_path_stamp is not None):
                # Retire the old route before relocalization can move its start.
                self.path_xyz = None
                self.scan_ack = False
                self.goal_pending_for_pose = self.waiting_for_map_recovery = True
                self._publish_path(rospy.Time.now())
                self._publish_status('WAITING_FOR_POSE', 'waiting for map recovery', target_map=self.active_floor)

    def _cloud_context(self):
        receipt=getattr(self,'odom_receipt',None)
        if receipt is None:
            fresh=self._has_fresh_odom()
            stamp=self.latest_odom_stamp
        else:
            stamp,received=receipt
            after=getattr(self,'initial_pose_after',None)
            fresh=(time.monotonic()-received<=.5 and
                   -.1 <= (rospy.Time.now()-stamp).to_sec() <= .5 and
                   (after is None or stamp>after))
        ready = (not self.planning_only and fresh
                 and ((self.cloud_localization == 'TRACKING' and time.monotonic()-self.cloud_localization_at<2.)
                      or rospy.get_param('/integration/simulation', False)))
        return {'instance_id': self.cloud_instance, 'asset_id': self.cloud_asset,
                'map_request_id': self.cloud_map_request, 'frame_id': self.frame_id,
                'ready': bool(ready and self.cloud_asset), 'localization_ready': bool(ready),
                'localization_stamp': stamp.to_sec() if stamp else 0,
                'task_owner': rospy.get_param('/3dnav/task_owner', 'native'),
                'recovery_required': False}

    def _on_cloud_goal(self, msg):
        with self.lock:
            try:
                if len(msg.data)>16384:raise ValueError('Request too large')
                context=self._cloud_context()
                value=validate_request(json.loads(msg.data),context)
                if not context['ready'] or context['task_owner']!='web':raise ValueError('Navigation not ready')
                if value['request_id'] in self.cloud_seen:return
                if self.active_goal is not None:raise ValueError('Navigation busy')
                self.cloud_seen.add(value['request_id'])
                self.cloud_request=value
                pose=PoseStamped();pose.header=Header(stamp=rospy.Time.now(),frame_id=self.frame_id)
                for key in ('x','y','z'):setattr(pose.pose.position,key,value['pose'][key])
                for key in ('x','y','z','w'):setattr(pose.pose.orientation,key,value['pose']['q'+key])
                self.cloud_submitting=True
                try:
                    # A cloud map label identifies the requested navigation
                    # map, not a floor inferred from a point's Z. A single map
                    # can include stair flights at several elevations.
                    label=value.get('map_label') or self._floor_for_z(pose.pose.position.z)
                    self._set_goal(_xyz_from_pose(pose.pose),_yaw_from_quat(pose.pose.orientation),label,pose)
                finally:
                    self.cloud_submitting = False
            except (ValueError,TypeError,KeyError) as exc:
                rospy.logwarn('Cloud goal rejected: %s',exc)

    def _on_goal(self, msg: PoseStamped) -> None:
        with self.lock:
            if getattr(self, 'cloud_asset', '') and rospy.get_param('/3dnav/task_owner', 'native')=='web' and not getattr(self,'cloud_submitting',False):
                return
            self._set_goal(_xyz_from_pose(msg.pose), _yaw_from_quat(msg.pose.orientation),
                           self._floor_for_z(msg.pose.position.z), msg)

    def _on_floor_goal(self, msg: PoseStamped, floor_label: str) -> None:
        with self.lock:
            if getattr(self,'cloud_asset','') and rospy.get_param('/3dnav/task_owner','native')=='web':
                return
            xyz = _xyz_from_pose(msg.pose)
            if abs(xyz[2]) < 1.0e-3:
                xyz[2] = float(self.floors[floor_label].get("height_z", 0.0))
            self._set_goal(xyz, _yaw_from_quat(msg.pose.orientation), floor_label, msg)

    def _floor_for_z(self, z: float) -> str:
        if not self.floors:
            return self.current_floor
        return min(self.floors, key=lambda label: abs(
            float(self.floors[label].get("height_z", 0.0)) - float(z)))

    def _snap_mission_goal_height(self, index, goal, floor_label):
        """Project a clicked goal onto a same-floor FREE cell without shifting XY."""
        graph = self.graph
        resolution = graph.resolution
        if not self.mission_segment_only:
            return index, goal
        height_tolerance = min(0.1, max(0.05, resolution))
        nearest = graph.points[index]
        if (abs(goal[2] - nearest[2]) <= height_tolerance
                and np.linalg.norm(goal[:2] - nearest[:2]) <= resolution / np.sqrt(2) + 1e-5):
            return index, goal

        column = tuple(np.floor(goal[:2] / resolution).astype(int))
        candidates = (
            i for i in (index, *graph.columns.get(column, ()))
            if abs(goal[2] - graph.points[i, 2]) <= 0.50 + 1e-6
            and np.all(np.abs(goal[:2] - graph.points[i, :2]) <= resolution / 2.0 + 1e-6)
            and self._floor_for_z(graph.points[i, 2]) == floor_label
        )
        snapped = min(candidates, key=lambda i: abs(goal[2] - graph.points[i, 2]), default=None)
        if snapped is None:
            return index, goal

        endpoint = goal.copy()
        endpoint[2] = graph.points[snapped, 2]
        rospy.loginfo("任务目标吸附到同一 FREE 格的地面高度: z=%.3f -> %.3f", goal[2], endpoint[2])
        return snapped, endpoint

    def _set_start_from_body_pose(self, pose) -> None:
        # Graph vertices are on the ground; odometry is the measured body pose.
        # This conversion is only for graph snapping, never for local tracking.
        ground = _xyz_from_pose(pose)
        ground[2] -= self.body_height
        self._set_start(ground, _yaw_from_quat(pose.orientation))

    def _set_start(self, xyz: np.ndarray, yaw: float) -> None:
        idx = self.graph.nearest(xyz, self.snap_radius)
        if idx is None:
            self.start_xyz = None
            if self.manual_start:
                marker = ball(0, np.zeros(3), (0.15, 0.85, 1.0, 1.0),
                              self.frame_id, rospy.Time.now())
                marker.action = Marker.DELETE
                self.start_pub.publish(marker)
            rospy.logwarn("起点离可通行地图太远: (%.2f, %.2f, %.2f)", *xyz)
            return
        self.start_xyz = self.graph.points[idx]
        self.start_yaw = yaw
        rospy.loginfo("起点吸附 (%.2f, %.2f, %.2f)", *self.start_xyz)
        self._publish_queries(rospy.Time.now())
        if self.goal_pending_for_pose:
            self.goal_pending_for_pose = False
            self._try_plan()

    def _clear_loop(self) -> None:
        self.loop_points = None
        self.loop_due_at = None
        self.loop_target_index = 1
        self.loop_arm_requested = False

    def _arm_loop(self) -> None:
        self.loop_arm_requested = False
        yaw_delta = math.atan2(math.sin(self.goal_yaw - self.start_yaw), math.cos(self.goal_yaw - self.start_yaw))
        if np.linalg.norm(self.goal_xyz - self.start_xyz) < .20 and abs(yaw_delta) < .12:
            rospy.loginfo("Loop not armed: start and goal already coincide within arrival tolerances.")
            return
        self.loop_points = ((self.start_xyz.copy(), self.start_yaw, self._floor_for_z(self.start_xyz[2])),
                            (self.goal_xyz.copy(), self.goal_yaw, self.active_floor))
        rospy.loginfo("Navigation loop armed: start <-> published goal; cancel or a new goal ends this loop.")

    def _set_goal(self, xyz: np.ndarray, yaw: float, floor_label: str, msg: PoseStamped, *, loop_leg=False) -> None:
        if not loop_leg:
            was_looping = self.loop_points is not None or self.loop_arm_requested
            self._clear_loop()
            self.loop_arm_requested = self.loop_available and not self.planning_only
            self.loop_available = False
            if was_looping:
                rospy.loginfo("Navigation loop ended by a new goal; the new goal runs once.")
        self.goal_position_recoveries = 0
        self.reference_refreshes = 0
        self.reference_refresh_origin = None
        self.reference_refreshed_at = None
        self.finishing_goal = False
        self.active_goal = None
        self.active_path_stamp = None
        self.goal_pending_for_pose = False
        self.waiting_for_map_recovery = False
        self.scan_ack = False
        self.path_xyz = None
        self.path_yaws = None
        self.goal_started_at = time.monotonic()
        self.active_floor = floor_label
        self._publish_status("PLANNING", "planning traversable-region path", target_map=floor_label)
        if self.latest_odom is not None:
            self._set_start_from_body_pose(self.latest_odom)
            if self.mission_segment_only:
                measured_floor = self._floor_for_z(
                    self.latest_odom.position.z - self.body_height
                )
                if measured_floor != floor_label:
                    self._fail_goal(
                        "cross-floor segment rejected: current=%s target=%s" %
                        (measured_floor, floor_label)
                    )
                    return
        if self.manual_start and self.start_xyz is not None:
            start_floor = self._floor_for_z(self.start_xyz[2])
            if self.mission_segment_only and start_floor != floor_label:
                self._fail_goal("cross-floor segment rejected: current=%s target=%s" %
                                (start_floor, floor_label))
                return
        idx = self.graph.nearest(xyz, self.snap_radius)
        if idx is None:
            self.goal_xyz = None
            self._fail_goal("终点离可通行地图太远: (%.2f, %.2f, %.2f)" % tuple(xyz))
            return
        idx, xyz = self._snap_mission_goal_height(idx, xyz, floor_label)
        resolution = self.graph.resolution
        try:
            continuous_goal_path(self.graph.points[idx:idx + 1], xyz, resolution)
        except ValueError as exc:
            self._fail_goal(str(exc))
            return
        self.requested_goal_xyz = xyz.copy()
        self.goal_xyz = self.graph.points[idx]
        self.active_goal = self.goal_xyz.copy()
        self.goal_yaw = yaw
        rospy.loginfo("终点吸附 (%.2f, %.2f, %.2f)", *self.goal_xyz)
        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = self.frame_id
        goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z = map(float, xyz)
        goal_pose.pose.orientation = msg.pose.orientation
        self.goal_pose_pub.publish(goal_pose)
        self._publish_queries(rospy.Time.now())
        if self.start_xyz is None:
            self.goal_pending_for_pose = True
            self._publish_status("WAITING_FOR_POSE", "set 3D Pose Estimate or wait for odometry",
                                 target_map=floor_label)
            return
        self._try_plan()

    def _try_plan(self) -> None:
        if self.start_xyz is None or self.goal_xyz is None:
            return
        start = self.graph.nearest(self.start_xyz, self.snap_radius)
        goal = self.graph.nearest(self.goal_xyz, self.snap_radius)
        if start is None or goal is None:
            self._fail_goal("起终点无法吸附到 FREE 体素")
            return
        t0 = time.perf_counter()
        try:
            path, surface_goal = self._graph_route(start, goal, self.active_floor)
        except ValueError as exc:
            self._fail_goal(str(exc))
            return
        dt_ms = (time.perf_counter() - t0) * 1000.0
        if path is None:
            self._fail_goal("规划失败: 无可通行路径 (%.1f ms)" % dt_ms)
            return
        if not np.array_equal(surface_goal, self.graph.points[goal]):
            rospy.logwarn("Goal voxel has a vertical-only tail on a flat approach: z=%.3f -> %.3f; "
                          "use connected surface at unchanged XY and yaw.",
                          self.graph.points[goal, 2], surface_goal[2])
            self.goal_xyz = surface_goal.copy()
            self.active_goal = self.goal_xyz.copy()
        self.reference_start_z_bias = 0.0
        if not self.planning_only and self._has_fresh_odom():
            measured = _xyz_from_pose(self.latest_odom)
            error = measured[2] - (path[0, 2] + self.body_height)
            if (np.linalg.norm(measured[:2] - path[0, :2]) <= 0.30
                    and abs(error) <= self.max_start_height_alignment + 0.5 * self.graph.resolution + 1e-6):
                self.reference_start_z_bias = float(np.clip(error, -self.max_start_height_alignment,
                                                          self.max_start_height_alignment))
                path = path.copy()
                # Register once at the measured start, never at the endpoint:
                # preserve every relative stair/floor height and graph query.
                path[:, 2] += self.reference_start_z_bias
                if abs(self.reference_start_z_bias) > 1e-3:
                    rospy.loginfo("Reference start height alignment: %.3f m; relative route heights unchanged.",
                                  self.reference_start_z_bias)
        dt_ms = (time.perf_counter() - t0) * 1000.0
        length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()) if len(path) > 1 else 0.0
        self.path_xyz = path
        self.path_yaws = _yaws_along(path, self.start_yaw, self.goal_yaw)
        if self.loop_arm_requested:
            self._arm_loop()
        rospy.loginfo("规划成功: %d 点, %.1f m, %.1f ms", len(path), length, dt_ms)
        self._publish_path(rospy.Time.now())
        if self.planning_only:
            target_floor = self.active_floor
            self.active_goal = None
            self.goal_started_at = None
            if target_floor and target_floor != self.current_floor:
                self.current_floor = target_floor
                self.current_map_pub.publish(String(data=target_floor))
            self._publish_status("IDLE", "planned", target_map=target_floor,
                                 path_points=len(path))
            self._publish_result("succeeded", "planned", target_floor)
        else:
            self._publish_status("WAITING_SCAN", "path sent to SCAN", target_map=self.active_floor,
                                 path_points=len(path))

    def _on_scan_trajectory(self, _msg) -> None:
        with self.lock:
            if (self.active_goal is None or self.path_xyz is None or self.active_path_stamp is None
                    or _msg.start_time < self.active_path_stamp):
                return
            if getattr(_msg, "obstacle_hold", False):
                self.scan_ack = False
                self._publish_status("WAITING_OBSTACLE", "waiting for a collision-free local route",
                                     target_map=self.active_floor, path_points=len(self.path_xyz))
            elif _msg.pos_pts and _msg.knots and not self.scan_ack:
                self.scan_ack = True
                self._publish_status("NAVIGATING", "SCAN trajectory ready",
                                     target_map=self.active_floor,
                                     path_points=len(self.path_xyz))

    def _on_goal_timer(self, _event) -> None:
        with self.lock:
            if getattr(self, 'cloud_asset', ''):
                self._publish_status('RUNNING' if self.active_goal is not None else 'IDLE', 'native navigation heartbeat')
            if self.loop_due_at is not None and time.monotonic() >= self.loop_due_at:
                if not self._has_fresh_odom():
                    rospy.logwarn_throttle(5., "Navigation loop waits for fresh localization before the next leg.")
                    return
                xyz, yaw, floor = self.loop_points[self.loop_target_index]
                self.loop_due_at = None
                pose = PoseStamped()
                pose.header = Header(stamp=rospy.Time.now(), frame_id=self.frame_id)
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, xyz)
                pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                self._set_goal(xyz.copy(), yaw, floor, pose, loop_leg=True)
            if (self.active_goal is not None and self.goal_started_at is not None
                    and time.monotonic() - self.goal_started_at > self.goal_timeout):
                self._fail_goal("navigation timeout")

    def _on_cancel(self, _msg: Empty) -> None:
        with self.lock:
            target_floor = self.active_floor
            was_active = self.active_goal is not None or self.goal_pending_for_pose or self.loop_points is not None
            self._clear_loop()
            self.active_goal = None
            self.goal_xyz = None
            self.goal_started_at = None
            self.goal_pending_for_pose = False
            self.waiting_for_map_recovery = False
            self.scan_ack = False
            self.path_xyz = None
            self.path_yaws = None
            self._publish_path(rospy.Time.now())
            self._publish_status("CANCELED", "navigation canceled", target_map=target_floor)
            if was_active:
                self._publish_result("preempted", "navigation canceled", target_floor)

    def _fail_goal(self, message: str) -> None:
        self._clear_loop()
        target_floor = self.active_floor
        self.active_goal = None
        self.goal_started_at = None
        self.goal_pending_for_pose = False
        self.waiting_for_map_recovery = False
        self.scan_ack = False
        self.path_xyz = None
        self.path_yaws = None
        self._publish_path(rospy.Time.now())
        self.cancel_scan_pub.publish(Empty())
        self._publish_status("FAILED", message, target_map=target_floor)
        self._publish_result("failed", message, target_floor)
        rospy.logwarn("FREE planner: %s", message)

    def _publish_status(self, state: str, message: str, **extra) -> None:
        payload = {
            "state": state,
            "message": message,
            "current_map": self.current_floor,
            "stamp": rospy.Time.now().to_sec(),
        }
        if getattr(self, 'cloud_asset', ''):
            payload.update(self._cloud_context())
        payload.update(extra)
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=True)))
        rospy.loginfo("FREE planner state=%s: %s", state, message)

    def _publish_result(self, status: str, message: str, target_floor: str = "") -> None:
        payload = {
            "status": status,
            "message": message,
            "current_map": self.current_floor,
            "target_map": target_floor,
            "stamp": rospy.Time.now().to_sec(),
        }
        if getattr(self, 'cloud_request', None):
            payload.update(self._cloud_context())
            payload['request_id'] = self.cloud_request['request_id']
            if status == 'preempted':payload['status']='cancelled'
        self.result_pub.publish(String(data=json.dumps(payload, ensure_ascii=True)))

    def _publish_queries(self, stamp) -> None:
        if self.start_xyz is not None:
            self.start_pub.publish(ball(0, self.start_xyz, (0.15, 0.85, 1.0, 1.0), self.frame_id, stamp))
        if self.goal_xyz is not None:
            self.goal_pub.publish(ball(1, self.goal_xyz, (1.0, 0.45, 0.85, 1.0), self.frame_id, stamp))

    def _publish_path(self, stamp) -> None:
        self.active_path_stamp = stamp if self.path_xyz is not None else None
        if self.path_xyz is None:
            empty = PathMsg()
            empty.header = Header(stamp=stamp, frame_id=self.frame_id)
            self.path_pub.publish(empty)
            return
        # Keep graph queries and floor selection on the original ground map.
        reference = self.path_xyz.copy()
        if len(reference) == 1 and self.latest_odom is not None:
            start = _xyz_from_pose(self.latest_odom)
            start[2] -= self.body_height
            if np.linalg.norm(start - reference[0]) > 1e-6:
                reference = np.vstack([start, reference])
        reference = continuous_goal_path(
            reference, self.requested_goal_xyz, self.graph.resolution
        )
        reference[:, 2] += self.body_height + self.path_z_offset
        self.finishing_goal = False
        self.path_pub.publish(
            to_path(
                reference,
                self.frame_id,
                stamp,
                _yaws_along(reference, self.start_yaw, self.goal_yaw),
            )
        )
        self._publish_queries(stamp)

    def spin(self) -> None:
        rospy.spin()


def _load_graph(meta: dict, max_step: float, center_weight: float) -> FreeGraph:
    points = read_pcd_xyz(Path(meta["free_pcd"]))
    # Navigation connectivity, clearance cost and height come exclusively from
    # traversable voxels. Ignore legacy recorded-trajectory metadata entirely.
    return FreeGraph(
        points,
        float(meta.get("resolution", 0.2)),
        max_step=max_step,
        center_weight=center_weight,
    )


def _path_center_score(graph: FreeGraph, path: np.ndarray) -> float:
    keys = [
        graph.index.get(
            (
                int(math.floor(x / graph.resolution)),
                int(math.floor(y / graph.resolution)),
                int(math.floor(z / graph.resolution)),
            )
        )
        for x, y, z in path
    ]
    vals = [graph.center_pen[i] for i in keys if i is not None]
    return float(np.mean(vals)) if vals else 1.0


def self_test(meta_path: Path, snap_radius: float, max_step: float, center_weight: float) -> int:
    meta = load_plan_meta(meta_path)
    graph = _load_graph(meta, max_step, center_weight)
    start = graph.nearest(graph.points[0], snap_radius)
    goal = graph.nearest(graph.points[len(graph.points) // 2], snap_radius)
    if start is None or goal is None:
        print("self-test: 无法吸附起终点")
        return 1
    t0 = time.perf_counter()
    path = graph.astar(start, goal)
    dt_ms = (time.perf_counter() - t0) * 1000.0
    if path is None:
        print("self-test: 规划失败 {:.1f} ms, nodes={}".format(dt_ms, len(graph.points)))
        return 1
    length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
    print(
        "self-test: {} -> {} points, {:.1f} m, {:.1f} ms, center_pen={:.3f}, free={}".format(
            start, goal, length, dt_ms, _path_center_score(graph, path), len(graph.points)
        )
    )
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="FREE 体素全局规划")
    parser.add_argument("--meta", type=Path, required=True)
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--snap-radius", type=float, default=2.0)
    parser.add_argument("--max-step", type=float, default=0.80, help="相邻体素允许的最大高度差，米")
    parser.add_argument("--center-weight", type=float, default=6.0, help="越大越贴走廊中央")
    parser.add_argument("--self-test", action="store_true")
    argv = sys.argv[1:] if "--self-test" in sys.argv else rospy.myargv(argv=sys.argv)[1:]
    args = parser.parse_args(argv)

    if args.self_test:
        return self_test(args.meta, args.snap_radius, args.max_step, args.center_weight)

    meta = load_plan_meta(args.meta)
    graph = _load_graph(meta, args.max_step, args.center_weight)
    rospy.init_node("global_planner")
    frame_id = rospy.get_param("~frame_id", args.frame_id)
    snap_radius = float(rospy.get_param("~snap_radius", args.snap_radius))
    rospy.loginfo(
        "FREE=%d res=%.2f snap=%.2f max_step=%.2f center_weight=%.2f",
        len(graph.points),
        graph.resolution,
        snap_radius,
        graph.max_step,
        graph.center_weight,
    )
    GlobalPlanner(graph, frame_id, snap_radius).spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
