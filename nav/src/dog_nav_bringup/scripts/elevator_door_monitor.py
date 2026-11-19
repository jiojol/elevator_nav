#!/usr/bin/env python3
"""Detect whether the configured elevator door aperture is clear.

The monitor compares live obstacle returns with a known 3-D door plane between
the floor's `lobby_pose` and `inside_pose`. A dense plane means closed; a clear
plane plus returns from the cabin interior means open. Hysteresis prevents a
single noisy scan from starting ingress.
"""

from __future__ import annotations

import json
import math
import time
from typing import Dict, Optional, Tuple

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String


def floor_for_z(floors: Dict[str, dict], ground_z: float) -> str:
    return min(floors, key=lambda label: abs(
        float(floors[label].get("height_z", 0.0)) - float(ground_z)))


def cloud_xyz(msg: PointCloud2) -> np.ndarray:
    if not msg.data or msg.width * msg.height == 0:
        return np.empty((0, 3), dtype=np.float64)
    fields = {field.name: int(field.offset) for field in msg.fields}
    if not all(name in fields for name in ("x", "y", "z")):
        raise ValueError("PointCloud2 must contain x/y/z offsets")
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if raw.size < msg.point_step * msg.width * msg.height:
        raise ValueError("PointCloud2 payload is shorter than row_step metadata")
    stride = int(msg.point_step)
    count = int(msg.width * msg.height)
    values = []
    for name in ("x", "y", "z"):
        offset = fields[name]
        if offset + 4 > stride:
            raise ValueError("PointCloud2 field exceeds point_step")
        values.append(np.ndarray(
            shape=(count,), dtype="<f4", buffer=raw,
            offset=offset, strides=(stride,)).copy())
    points = np.column_stack(values).astype(np.float64)
    return points[np.isfinite(points).all(axis=1)]


def door_geometry(lobby_xyz, inside_xyz, door_center_xy=None):
    lobby = np.asarray(lobby_xyz[:2], dtype=np.float64)
    inside = np.asarray(inside_xyz[:2], dtype=np.float64)
    center = (0.5 * (lobby + inside) if door_center_xy is None else
              np.asarray(door_center_xy, dtype=np.float64))
    if center.shape != (2,) or not np.isfinite(center).all():
        raise ValueError("door_center_xy must contain two finite coordinates")
    direction = inside - center
    depth = float(np.linalg.norm(direction))
    if not math.isfinite(depth) or depth < 0.05:
        raise ValueError("door center and inside_pose are too close")
    normal = direction / depth
    if np.dot(lobby - center, normal) >= 0:
        raise ValueError("lobby_pose must be outside the door plane")
    return center, normal, 2.0 * depth


def evaluate_door_points(points: np.ndarray, lobby_xyz, inside_xyz,
                         floor_z: float, door_width: float,
                         door_height: float, plane_thickness: float,
                         cell_size: float, from_cabin: bool = False,
                         door_center_xy=None) -> Dict[str, float]:
    center, normal, length = door_geometry(lobby_xyz, inside_xyz, door_center_xy)
    across_axis = np.array([-normal[1], normal[0]], dtype=np.float64)
    relative = points[:, :2] - center
    along = relative @ normal
    across = relative @ across_axis
    vertical = points[:, 2] - floor_z

    plane_mask = ((np.abs(along) <= 0.5 * plane_thickness) &
                  (np.abs(across) <= 0.5 * door_width) &
                  (vertical >= 0.0) & (vertical <= door_height))
    plane = points[plane_mask]
    if len(plane):
        plane_u = np.floor((plane[:, :2] - center[:2]) @ across_axis / cell_size).astype(np.int64)
        plane_v = np.floor((plane[:, 2] - floor_z) / cell_size).astype(np.int64)
        occupied = len(set(zip(plane_u.tolist(), plane_v.tolist())))
    else:
        occupied = 0
    columns = max(1, int(math.ceil(door_width / cell_size)))
    rows = max(1, int(math.ceil(door_height / cell_size)))
    occupancy_ratio = float(occupied) / float(columns * rows)

    beyond_door = -along if from_cabin else along
    beyond_mask = ((beyond_door > 0.5 * plane_thickness) &
                   (beyond_door < max(1.5, length + 0.5)) &
                   (np.abs(across) <= 0.5 * door_width) &
                   (vertical >= 0.0) & (vertical <= door_height + 0.4))
    inside_points = int(np.count_nonzero(beyond_mask))
    return {
        "plane_points": int(len(plane)),
        "occupied_cells": int(occupied),
        "total_cells": int(columns * rows),
        "occupancy_ratio": occupancy_ratio,
        "inside_points": inside_points,
        "direction_length": length,
    }


class ElevatorDoorMonitor:
    def __init__(self) -> None:
        self.floors = rospy.get_param("~floors", {})
        self.elevator = rospy.get_param("~elevator", {})
        self.body_height = float(rospy.get_param("~body_height", 0.40))
        self.near_radius = float(rospy.get_param("~near_radius", 1.5))
        self.cell_size = float(rospy.get_param("~cell_size", 0.10))
        self.plane_thickness = float(rospy.get_param("~plane_thickness", 0.16))
        self.closed_ratio = float(rospy.get_param("~closed_occupancy_ratio", 0.18))
        self.open_ratio = float(rospy.get_param("~open_occupancy_ratio", 0.06))
        self.min_inside_points = int(rospy.get_param("~min_inside_points", 15))
        self.open_hold_s = float(rospy.get_param("~open_hold_s", 0.60))
        self.closed_hold_s = float(rospy.get_param("~closed_hold_s", 0.40))
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 0.50))
        self.require_tracking = bool(rospy.get_param("~require_tracking", True))
        self.configured = bool(self.elevator.get("configured", False))
        if not self.floors:
            raise ValueError("door monitor requires floor configuration")
        if not (0.0 < self.open_ratio < self.closed_ratio <= 1.0):
            raise ValueError("door occupancy thresholds are invalid")
        if not self.configured:
            rospy.logwarn("Elevator door monitor idle: elevator poses are not configured")

        self.latest_odom: Optional[Odometry] = None
        self.latest_odom_receipt = 0.0
        self.localization_status = ""
        self.open_state = False
        self.door_context = None
        self.open_candidate_since: Optional[float] = None
        self.closed_candidate_since: Optional[float] = None
        self.publisher = rospy.Publisher(
            str(rospy.get_param("~door_open_topic", "/elevator/door_open")),
            Bool, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(
            str(rospy.get_param("~door_status_topic", "/elevator/door_status")),
            String, queue_size=1, latch=True)
        rospy.Subscriber(
            str(rospy.get_param("~cloud_topic", "/navigation/obstacle_points")),
            PointCloud2, self.on_cloud, queue_size=2)
        rospy.Subscriber(
            str(rospy.get_param("~odom_topic", "/dog_nav/odometry")),
            Odometry, self.on_odom, queue_size=5)
        rospy.Subscriber("/dog_nav/localization_status", String,
                         self.on_localization, queue_size=1)
        self.publisher.publish(Bool(data=False))
        rospy.loginfo("Elevator door monitor ready")

    @staticmethod
    def now() -> float:
        return time.monotonic()

    def on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.latest_odom_receipt = self.now()

    def on_localization(self, msg: String) -> None:
        self.localization_status = msg.data

    def publish_status(self, state: str, **extra) -> None:
        payload = {
            "state": state,
            "door_open": self.open_state,
            "stamp": rospy.Time.now().to_sec(),
        }
        payload.update(extra)
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=True)))

    def fresh_odom(self) -> bool:
        return (self.latest_odom is not None and
                self.now() - self.latest_odom_receipt <= self.odom_timeout)

    def reset_door(self) -> None:
        self.open_candidate_since = self.closed_candidate_since = None
        if self.open_state:
            self.open_state = False
            self.publisher.publish(Bool(data=False))

    def on_cloud(self, msg: PointCloud2) -> None:
        now = self.now()
        if not self.configured:
            self.publish_status("ELEVATOR_NOT_CONFIGURED")
            return
        if not self.fresh_odom():
            self.reset_door()
            self.publish_status("WAIT_ODOM")
            return
        if self.require_tracking and self.localization_status != "TRACKING":
            self.reset_door()
            self.publish_status("WAIT_TRACKING", localization=self.localization_status)
            return
        pose = self.latest_odom.pose.pose
        ground_z = float(pose.position.z) - self.body_height
        floor = floor_for_z(self.floors, ground_z)
        if floor not in self.elevator.get("floors", {}):
            self.reset_door()
            self.publish_status("NO_ELEVATOR_FOR_FLOOR", floor=floor)
            return
        config = self.elevator["floors"][floor]
        lobby = config.get("lobby_pose")
        inside = config.get("inside_pose")
        if not isinstance(lobby, (list, tuple)) or len(lobby) != 4 or \
           not isinstance(inside, (list, tuple)) or len(inside) != 4:
            self.publish_status("INVALID_ELEVATOR_POSE", floor=floor)
            return
        lobby_xy = np.asarray(lobby[:2], dtype=np.float64)
        inside_xy = np.asarray(inside[:2], dtype=np.float64)
        robot_xy = np.asarray([pose.position.x, pose.position.y], dtype=np.float64)
        near = bool(min(np.linalg.norm(robot_xy - lobby_xy),
                        np.linalg.norm(robot_xy - inside_xy)) <= self.near_radius)
        if not near:
            self.reset_door()
            self.publish_status("NOT_AT_ELEVATOR", floor=floor)
            return
        center_xy = config.get("door_center_xy")
        try:
            center, normal, _ = door_geometry(lobby, inside, center_xy)
        except ValueError as error:
            self.reset_door()
            self.publish_status("INVALID_ELEVATOR_POSE", floor=floor, error=str(error))
            return
        from_cabin = np.dot(robot_xy - center, normal) > 0
        side = "cabin" if from_cabin else "lobby"
        context = (floor, side)
        if self.door_context != context:
            self.reset_door()
            self.door_context = context
        try:
            points = cloud_xyz(msg)
            metrics = evaluate_door_points(
                points, lobby[:3], inside[:3], float(self.floors[floor]["height_z"]),
                float(config.get("door_width", self.elevator.get("door_width", 1.0))),
                float(config.get("door_height", self.elevator.get("door_height", 2.2))),
                self.plane_thickness, self.cell_size, from_cabin=from_cabin,
                door_center_xy=center_xy)
        except (ValueError, KeyError) as error:
            self.reset_door()
            self.publish_status("INVALID_CLOUD", floor=floor, error=str(error))
            return

        closed_candidate = metrics["occupancy_ratio"] >= self.closed_ratio
        open_candidate = (metrics["occupancy_ratio"] <= self.open_ratio and
                          metrics["inside_points"] >= self.min_inside_points)
        if open_candidate:
            self.closed_candidate_since = None
            if self.open_candidate_since is None:
                self.open_candidate_since = now
            if (not self.open_state and
                    now - self.open_candidate_since >= self.open_hold_s):
                self.open_state = True
                self.publisher.publish(Bool(data=True))
                rospy.loginfo("Elevator door open detected on %s", floor)
        elif closed_candidate:
            self.open_candidate_since = None
            if self.closed_candidate_since is None:
                self.closed_candidate_since = now
            if (self.open_state and
                    now - self.closed_candidate_since >= self.closed_hold_s):
                self.open_state = False
                self.publisher.publish(Bool(data=False))
                rospy.loginfo("Elevator door closed detected on %s", floor)
        else:
            self.open_candidate_since = self.closed_candidate_since = None
        self.publish_status(
            "DOOR_OPEN" if self.open_state else "DOOR_CLOSED_OR_UNKNOWN",
            floor=floor, side=side, near=near,
            open_confirmed=bool(self.open_state and open_candidate), **metrics)


def main() -> int:
    rospy.init_node("elevator_door_monitor")
    try:
        ElevatorDoorMonitor()
    except Exception as error:
        rospy.logfatal("Elevator door monitor: %s", error)
        return 1
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
