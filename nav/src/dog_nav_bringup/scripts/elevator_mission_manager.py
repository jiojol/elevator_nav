#!/usr/bin/env python3
"""Coordinate two-stage navigation through one configured elevator.

The FREE global planner remains a same-floor planner. This node receives the
user's final goal, sends only executable segments to `/plan_map/segment_goal`,
and uses Elevator-LIO's `in_elevator` signal to pause during the ride and resume
on the destination floor.
"""

from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Empty, Header, String
from visualization_msgs.msg import Marker, MarkerArray


WAIT_GOAL = "WAIT_GOAL"
WAIT_POSE = "WAIT_POSE"
TRACK_SINGLE = "TRACK_SINGLE"
TRACK_LEG1 = "TRACK_LEG1"
WAIT_DOOR_OUTSIDE = "WAIT_DOOR_OUTSIDE"
TRACK_INGRESS = "TRACK_INGRESS"
ALIGN_CABIN = "ALIGN_CABIN"
ALIGN_EXIT = "ALIGN_EXIT"
WAIT_ELEVATOR_MODE = "WAIT_ELEVATOR_MODE"
ELEVATOR_TRANSIT = "ELEVATOR_TRANSIT"
WAIT_DOOR_OPEN = "WAIT_DOOR_OPEN"
TRACK_LEG2 = "TRACK_LEG2"
ARRIVED = "ARRIVED"
FAILED = "FAILED"
CANCELED = "CANCELED"


@dataclass(frozen=True)
class MissionLeg:
    kind: str
    floor: str
    xyz: Tuple[float, float, float]
    yaw: float


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def floor_for_z(floors: Dict[str, dict], ground_z: float) -> str:
    if not floors:
        raise ValueError("floor configuration is empty")
    return min(
        floors,
        key=lambda label: abs(float(floors[label].get("height_z", 0.0)) - float(ground_z)),
    )


def _inside_pose(elevator: dict, floor: str) -> Tuple[Tuple[float, float, float], float]:
    floors = elevator.get("floors", {})
    if floor not in floors:
        raise ValueError("elevator {} has no inside_pose for {}".format(
            elevator.get("id", "E?"), floor))
    pose = floors[floor].get("inside_pose")
    if not isinstance(pose, (list, tuple)) or len(pose) != 4:
        raise ValueError("elevator {} {} inside_pose must be [x,y,z,yaw]".format(
            elevator.get("id", "E?"), floor))
    values = [float(value) for value in pose]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("elevator inside_pose contains non-finite values")
    return tuple(values[:3]), values[3]


def _lobby_pose(elevator: dict, floor: str) -> Tuple[Tuple[float, float, float], float]:
    floors = elevator.get("floors", {})
    if floor not in floors:
        raise ValueError("elevator {} has no lobby_pose for {}".format(
            elevator.get("id", "E?"), floor))
    pose = floors[floor].get("lobby_pose")
    if not isinstance(pose, (list, tuple)) or len(pose) != 4:
        raise ValueError("elevator {} {} lobby_pose must be [x,y,z,yaw]".format(
            elevator.get("id", "E?"), floor))
    values = [float(value) for value in pose]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("elevator lobby_pose contains non-finite values")
    return tuple(values[:3]), values[3]


def build_mission_legs(floors: Dict[str, dict], elevator: dict,
                       start_floor: str, target_floor: str,
                       final_xyz: Sequence[float], final_yaw: float) -> List[MissionLeg]:
    """Return one same-floor leg or the two executable cross-floor legs."""
    if start_floor not in floors or target_floor not in floors:
        raise ValueError("mission floor is not configured: {} -> {}".format(
            start_floor, target_floor))
    final = (float(final_xyz[0]), float(final_xyz[1]), float(final_xyz[2]))
    if not all(math.isfinite(value) for value in final + (float(final_yaw),)):
        raise ValueError("final goal contains non-finite values")
    if start_floor == target_floor:
        return [MissionLeg("FINAL", target_floor, final, float(final_yaw))]
    if not bool(elevator.get("configured", False)):
        raise ValueError("elevator poses are not configured")
    lobby, lobby_yaw = _lobby_pose(elevator, start_floor)
    inside, inside_yaw = _inside_pose(elevator, start_floor)
    if elevator.get("align_exit_yaw", False):
        inside_yaw = lobby_yaw
    # The destination anchor is not sent as a goal, but its presence validates
    # that this elevator serves the requested target floor. The second A* leg
    # starts from the measured pose after the door opens.
    _inside_pose(elevator, target_floor)
    return [
        MissionLeg("LOBBY", start_floor, lobby, lobby_yaw),
        MissionLeg("INGRESS", start_floor, inside, inside_yaw),
        MissionLeg("FINAL", target_floor, final, float(final_yaw)),
    ]


class ElevatorMissionManager:
    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.body_height = float(rospy.get_param("~body_height", 0.40))
        self.destination_height_tolerance = float(rospy.get_param("~goal_z_tolerance", 0.40))
        if not math.isfinite(self.destination_height_tolerance) or self.destination_height_tolerance <= 0:
            raise ValueError("goal_z_tolerance must be finite and positive")
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 0.60))
        self.door_settle_s = float(rospy.get_param("~door_settle_s", 0.50))
        self.exit_door_confirm_s = float(rospy.get_param("~exit_door_confirm_s", 1.0))
        self.door_open_timeout_s = float(rospy.get_param("~door_open_timeout_s", 180.0))
        self.door_plan_timeout_s = float(rospy.get_param("~door_plan_timeout_s", 30.0))
        self.elevator_timeout_s = float(rospy.get_param("~elevator_timeout_s", 300.0))
        self.preview_routes = bool(rospy.get_param("~preview_routes", False))
        self.manual_start = bool(rospy.get_param("~manual_start", False))
        self.manual_start_z: Optional[float] = None
        self.require_tracking = bool(rospy.get_param("~require_tracking", True))
        self.accept_move_base_goal = bool(rospy.get_param("~accept_move_base_goal", True))
        self.frame_id = str(rospy.get_param("~frame_id", "map"))
        self.floors = rospy.get_param("~floors", {})
        self.elevator = rospy.get_param("~elevator", {})
        if self.body_height <= 0.0 or not math.isfinite(self.body_height):
            raise ValueError("body_height must be finite and positive")
        if self.odom_timeout <= 0.0 or not math.isfinite(self.odom_timeout):
            raise ValueError("odom_timeout must be finite and positive")
        if self.door_settle_s < 0.0 or not math.isfinite(self.door_settle_s):
            raise ValueError("door_settle_s must be finite and nonnegative")
        if self.exit_door_confirm_s < 0.0 or not math.isfinite(self.exit_door_confirm_s):
            raise ValueError("exit_door_confirm_s must be finite and nonnegative")
        if self.door_open_timeout_s <= 0.0 or not math.isfinite(self.door_open_timeout_s):
            raise ValueError("door_open_timeout_s must be finite and positive")
        if self.door_plan_timeout_s <= 0.0 or not math.isfinite(self.door_plan_timeout_s):
            raise ValueError("door_plan_timeout_s must be finite and positive")
        if self.elevator_timeout_s <= 0.0 or not math.isfinite(self.elevator_timeout_s):
            raise ValueError("elevator_timeout_s must be finite and positive")

        self.state = WAIT_GOAL
        self.message = "waiting for final goal"
        self.final_goal: Optional[PoseStamped] = None
        self.final_ground_xyz: Optional[Tuple[float, float, float]] = None
        self.final_yaw = 0.0
        self.target_floor = ""
        self.start_floor = ""
        self.legs: List[MissionLeg] = []
        self.leg_index = 0
        self.active_segment_stamp = rospy.Time()
        self.replacement_cancel_at: Optional[rospy.Time] = None
        self.cabin_alignment_stamp = rospy.Time()
        self.cabin_alignment_started_at = None
        self.latest_odom: Optional[Odometry] = None
        self.latest_odom_receipt = 0.0
        self.localization_status = ""
        self.saw_elevator_mode = False
        self.latest_elevator_mode = False
        self.elevator_stopped_at: Optional[float] = None
        self.door_wait_started_at: Optional[float] = None
        self.door_open = False
        self.transit_started_at: Optional[float] = None
        self.source_door_seen_at = float("-inf")
        self.exit_door_since: Optional[float] = None
        self.exit_door_seen_at = float("-inf")

        self.segment_goal_pub = rospy.Publisher(
            "/plan_map/segment_goal", PoseStamped, queue_size=1)
        if self.preview_routes:
            self.preview_request_pub = rospy.Publisher(
                "/elevator_nav/preview_request", Path, queue_size=1, latch=True)
        self.cancel_pub = rospy.Publisher(
            "/map_manager/cancel", Empty, queue_size=1)
        self.status_pub = rospy.Publisher(
            "/elevator_nav/status", String, queue_size=1, latch=True)
        self.result_pub = rospy.Publisher(
            "/elevator_nav/result", String, queue_size=1, latch=True)

        self.elevator_markers_pub = rospy.Publisher(
            "/elevator_nav/markers", MarkerArray, queue_size=1, latch=True)
        self.publish_elevator_markers()

        rospy.Subscriber("/elevator_nav/goal", PoseStamped, self.on_goal, queue_size=1)
        if self.accept_move_base_goal:
            rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.on_goal, queue_size=1)
        rospy.Subscriber("/elevator_nav/cancel", Empty, self.on_cancel, queue_size=1)
        if self.manual_start:
            rospy.Subscriber("/plan_map/query_start", Marker,
                             self.on_manual_start, queue_size=1)
        else:
            rospy.Subscriber("/dog_nav/odometry", Odometry, self.on_odom, queue_size=10)
            rospy.Subscriber("/dog_nav/localization_status", String,
                             self.on_localization, queue_size=1)
        rospy.Subscriber("/elevator_lio/in_elevator", Bool,
                         self.on_elevator_mode, queue_size=1)
        rospy.Subscriber(str(rospy.get_param(
            "~door_open_topic", "/elevator/door_open")), Bool,
            self.on_door_state, queue_size=1)
        rospy.Subscriber(str(rospy.get_param(
            "~door_status_topic", "/elevator/door_status")), String,
            self.on_door_status, queue_size=1)
        rospy.Subscriber("/planning/goal_reached", Header,
                         self.on_goal_reached, queue_size=10)
        rospy.Subscriber("/elevator_nav/cabin_aligned", Header, self.on_cabin_aligned, queue_size=1)
        rospy.Subscriber("/map_manager/result", String,
                         self.on_planner_result, queue_size=10)
        rospy.Subscriber("/map_manager/status", String,
                         self.on_planner_status, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.10), self.on_timer)
        self.publish_status()
        rospy.loginfo("Elevator mission manager ready")

    def publish_elevator_markers(self) -> None:
        markers = MarkerArray()
        if self.elevator.get("configured", False):
            for floor in sorted(self.elevator.get("floors", {})):
                for label, getter, color in (
                    ("Lobby", _lobby_pose, (1.0, 0.65, 0.1)),
                    ("Cabin", _inside_pose, (0.15, 0.75, 1.0)),
                ):
                    try:
                        xyz, _ = getter(self.elevator, floor)
                    except ValueError as exc:
                        rospy.logwarn("Cannot display elevator point: %s", exc)
                        continue
                    marker = Marker()
                    marker.header.frame_id = self.frame_id
                    marker.ns = "elevator_points"
                    marker.id = len(markers.markers)
                    marker.type = Marker.SPHERE
                    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = xyz
                    marker.pose.position.z += 0.15
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = marker.scale.y = marker.scale.z = 0.25
                    marker.color.r, marker.color.g, marker.color.b = color
                    marker.color.a = 1.0
                    markers.markers.append(marker)

                    text = Marker()
                    text.header = marker.header
                    text.ns = marker.ns
                    text.id = len(markers.markers)
                    text.type = Marker.TEXT_VIEW_FACING
                    text.pose.position.x, text.pose.position.y, text.pose.position.z = xyz
                    text.pose.position.z += 0.55
                    text.pose.orientation.w = 1.0
                    text.scale.z = 0.16
                    text.color = marker.color
                    text.text = "{} {} {}".format(self.elevator.get("id", "E?"), floor, label)
                    markers.markers.append(text)
        self.elevator_markers_pub.publish(markers)

    def now(self) -> float:
        return time.monotonic()

    def fresh_odom(self) -> bool:
        return (self.latest_odom is not None and
                self.now() - self.latest_odom_receipt <= self.odom_timeout)

    def ready_for_planning(self) -> bool:
        if self.manual_start:
            return self.manual_start_z is not None
        if not self.fresh_odom():
            return False
        return not self.require_tracking or self.localization_status == "TRACKING"

    def ground_z(self) -> float:
        if self.manual_start_z is not None:
            return self.manual_start_z
        if self.latest_odom is None:
            raise ValueError("odometry is not available")
        return float(self.latest_odom.pose.pose.position.z) - self.body_height

    def current_floor(self) -> str:
        return floor_for_z(self.floors, self.ground_z())

    def display_floor(self) -> str:
        try:
            if self.manual_start_z is not None or self.fresh_odom():
                return self.current_floor()
            return self.start_floor
        except ValueError:
            return self.start_floor

    def publish_status(self) -> None:
        payload = {
            "state": self.state,
            "message": self.message,
            "current_floor": self.display_floor(),
            "alignment_floor": self.start_floor if self.state == ALIGN_CABIN else self.target_floor,
            "alignment_stamp_ns": self.cabin_alignment_stamp.to_nsec() if self.state in (ALIGN_CABIN, ALIGN_EXIT) else 0,
            "target_floor": self.target_floor,
            "elevator_id": str(self.elevator.get("id", "")),
            "stamp": rospy.Time.now().to_sec(),
        }
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=True)))

    def publish_result(self, status: str, message: str) -> None:
        payload = {
            "status": status,
            "message": message,
            "start_floor": self.start_floor,
            "target_floor": self.target_floor,
            "elevator_id": str(self.elevator.get("id", "")),
            "stamp": rospy.Time.now().to_sec(),
        }
        self.result_pub.publish(String(data=json.dumps(payload, ensure_ascii=True)))

    def transition(self, state: str, message: str) -> None:
        self.state = state
        self.message = message
        self.publish_status()
        rospy.loginfo("Elevator mission %s: %s", state, message)

    def fail(self, message: str) -> None:
        self.replacement_cancel_at = None
        self.transition(FAILED, message)
        self.publish_result("failed", message)
        self.cancel_pub.publish(Empty())
        rospy.logwarn("Elevator mission failed: %s", message)

    def on_manual_start(self, msg: Marker) -> None:
        with self.lock:
            if msg.header.frame_id != self.frame_id:
                return
            z = float(msg.pose.position.z)
            self.manual_start_z = z if msg.action == Marker.ADD and math.isfinite(z) else None
            if self.state == WAIT_POSE:
                self.try_start_mission()

    def request_preview(self, target: Optional[PoseStamped] = None,
                        anchors: Sequence[MissionLeg] = ()) -> None:
        request = Path(header=Header(stamp=rospy.Time.now(), frame_id=self.frame_id))
        for anchor in anchors:
            start = PoseStamped(header=request.header)
            start.pose.position.x, start.pose.position.y, start.pose.position.z = anchor.xyz
            start.pose.orientation.z = math.sin(anchor.yaw / 2.0)
            start.pose.orientation.w = math.cos(anchor.yaw / 2.0)
            request.poses.append(start)
        if target is not None:
            request.poses.append(target)
        self.preview_request_pub.publish(request)

    def on_goal(self, msg: PoseStamped) -> None:
        with self.lock:
            replacing = (not self.manual_start and not self.latest_elevator_mode and
                         (self.state == TRACK_SINGLE or
                          (self.state == TRACK_LEG2 and self.clear_of_exit_door())))
            if not self.manual_start and not replacing and self.state not in (
                    WAIT_GOAL, WAIT_POSE, ARRIVED, FAILED, CANCELED):
                rospy.logwarn("Elevator mission busy in state %s; goal rejected", self.state)
                return
            if msg.header.frame_id != self.frame_id:
                self.fail("goal frame must be {}".format(self.frame_id))
                return
            values = (
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                msg.pose.orientation.x, msg.pose.orientation.y,
                msg.pose.orientation.z, msg.pose.orientation.w,
            )
            if not all(math.isfinite(float(value)) for value in values):
                self.fail("goal contains non-finite pose values")
                return
            qnorm = sum(float(value) ** 2 for value in values[3:])
            if qnorm < 1.0e-9:
                self.fail("goal orientation is invalid")
                return
            self.final_goal = msg
            if self.preview_routes:
                self.request_preview(msg)
            self.final_ground_xyz = (
                float(msg.pose.position.x), float(msg.pose.position.y),
                float(msg.pose.position.z))
            self.final_yaw = yaw_from_quaternion(msg.pose.orientation)
            self.target_floor = floor_for_z(self.floors, self.final_ground_xyz[2])
            self.saw_elevator_mode = False
            self.elevator_stopped_at = None
            self.door_wait_started_at = None
            self.door_open = False
            self.source_door_seen_at = float("-inf")
            self.transit_started_at = None
            pose_source = "RViz 3D Pose Estimate" if self.manual_start else "fresh tracking odometry"
            if replacing:
                self.replacement_cancel_at = rospy.Time.now()
                self.active_segment_stamp = rospy.Time()
            message = ("replacing the current navigation goal" if self.replacement_cancel_at is not None
                       else "waiting for " + pose_source)
            self.transition(WAIT_POSE, message)
            if replacing:
                self.cancel_pub.publish(Empty())
            self.try_start_mission()

    def try_start_mission(self) -> None:
        if self.final_goal is None or self.replacement_cancel_at is not None or not self.ready_for_planning():
            return
        try:
            self.start_floor = self.current_floor()
            self.legs = build_mission_legs(
                self.floors, self.elevator, self.start_floor, self.target_floor,
                self.final_ground_xyz, self.final_yaw)
        except ValueError as exc:
            self.fail(str(exc))
            return
        self.leg_index = 0
        first_kind = self.legs[0].kind
        if getattr(self, "preview_routes", False) and first_kind == "LOBBY":
            ingress = self.legs[1]
            destination_cabin, exit_yaw = _inside_pose(self.elevator, self.target_floor)
            if not self.elevator.get("align_exit_yaw", False):
                exit_yaw = ingress.yaw
            self.request_preview(
                self.final_goal,
                (self.legs[0], ingress,
                 MissionLeg("CABIN", self.target_floor, destination_cabin, exit_yaw)))
        self.send_leg(self.legs[0])
        if first_kind == "FINAL":
            self.transition(TRACK_SINGLE, "tracking same-floor goal")
        else:
            self.transition(TRACK_LEG1, "tracking to elevator lobby pose")

    def send_leg(self, leg: MissionLeg) -> None:
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = leg.xyz
        msg.pose.orientation.z = math.sin(leg.yaw / 2.0)
        msg.pose.orientation.w = math.cos(leg.yaw / 2.0)
        self.active_segment_stamp = msg.header.stamp
        self.segment_goal_pub.publish(msg)
        rospy.loginfo("Elevator mission leg %s -> %s: [%.2f, %.2f, %.2f] yaw=%.2f",
                      leg.kind, leg.floor, leg.xyz[0], leg.xyz[1], leg.xyz[2], leg.yaw)

    def on_odom(self, msg: Odometry) -> None:
        with self.lock:
            self.latest_odom = msg
            self.latest_odom_receipt = self.now()
            if self.state == WAIT_POSE:
                self.try_start_mission()

    def on_localization(self, msg: String) -> None:
        with self.lock:
            self.localization_status = msg.data
            if self.state == WAIT_DOOR_OPEN and msg.data != "TRACKING":
                self.exit_door_since = None
            if self.state == WAIT_POSE:
                self.try_start_mission()

    def on_goal_reached(self, msg: Header) -> None:
        with self.lock:
            if self.state not in (TRACK_SINGLE, TRACK_LEG1, TRACK_INGRESS, TRACK_LEG2):
                return
            if msg.stamp < self.active_segment_stamp:
                return
            if self.state in (TRACK_SINGLE, TRACK_LEG2):
                self.transition(ARRIVED, "final goal reached")
                self.publish_result("succeeded", "final goal reached")
                return
            self.active_segment_stamp = rospy.Time()
            if self.state == TRACK_LEG1:
                self.door_wait_started_at = self.now()
                self.transition(WAIT_DOOR_OUTSIDE,
                                "at elevator lobby; waiting for door_open=true")
                self.try_start_ingress()
                return
            if self.state == TRACK_INGRESS:
                if self.latest_elevator_mode:
                    self.enter_transit()
                elif self.elevator.get("align_exit_yaw", False) and not self.manual_start:
                    self.start_cabin_alignment(ALIGN_CABIN)
                else:
                    self.transition(WAIT_ELEVATOR_MODE,
                                    "inside elevator; waiting for in_elevator=true")

    def start_cabin_alignment(self, state: str) -> None:
        self.cabin_alignment_stamp = rospy.Time.now()
        self.cabin_alignment_started_at = self.now()
        self.transition(state, "inside elevator; aligning to the exit-facing pose")

    def on_cabin_aligned(self, msg: Header) -> None:
        with self.lock:
            if (self.state not in (ALIGN_CABIN, ALIGN_EXIT) or self.latest_elevator_mode
                    or msg.stamp != self.cabin_alignment_stamp or not self.ready_for_planning()):
                return
            if self.state == ALIGN_CABIN:
                self.transition(WAIT_ELEVATOR_MODE, "exit-facing pose confirmed; waiting for elevator ride")
            elif self.current_floor() == self.target_floor:
                self.start_exit_leg()

    def clear_of_exit_door(self) -> bool:
        if (self.latest_elevator_mode or not self.ready_for_planning()
                or self.current_floor() != self.target_floor):
            return False
        stop = self.elevator["floors"][self.target_floor]
        door = stop.get("door_center_xy")
        if door is None:
            return False
        cx, cy, z, _ = stop["inside_pose"]
        if abs(self.ground_z() - z) > self.destination_height_tolerance:
            return False
        nx, ny = door[0] - cx, door[1] - cy
        depth = math.hypot(nx, ny)
        if depth < .05:
            return False
        p = self.latest_odom.pose.pose.position
        return ((p.x - door[0]) * nx + (p.y - door[1]) * ny) / depth >= .65

    def start_exit_leg(self) -> None:
        self.leg_index = 2
        self.send_leg(self.legs[self.leg_index])
        self.transition(TRACK_LEG2, "tracking destination-floor goal")

    def try_start_ingress(self) -> None:
        if (self.state != WAIT_DOOR_OUTSIDE or not self.door_open or
                self.latest_elevator_mode or self.now() - self.source_door_seen_at > self.odom_timeout):
            return
        if not self.ready_for_planning():
            self.message = "door open; waiting for fresh TRACKING localization"
            return
        if self.current_floor() != self.start_floor:
            return
        self.leg_index = 1
        self.send_leg(self.legs[self.leg_index])
        self.transition(TRACK_INGRESS, "door open; tracking short cabin ingress")

    def enter_transit(self) -> None:
        self.saw_elevator_mode = True
        self.transit_started_at = self.now()
        self.elevator_stopped_at = None
        self.cancel_pub.publish(Empty())
        self.transition(ELEVATOR_TRANSIT, "elevator transit; navigation frozen")

    def ingress_pose_inside_cabin(self) -> bool:
        if not self.fresh_odom() or self.current_floor() != self.start_floor:
            return False
        stop = self.elevator["floors"][self.start_floor]
        cx, cy, _, _ = stop["inside_pose"]
        door = stop.get("door_center_xy")
        if door is None:
            return False
        px, py = door
        position = self.latest_odom.pose.pose.position
        nx, ny = px - cx, py - cy
        door_depth = math.hypot(nx, ny)
        if door_depth < .05 or math.hypot(position.x - cx, position.y - cy) > .30:
            return False
        return ((px - position.x) * nx + (py - position.y) * ny) / door_depth >= .40

    def on_elevator_mode(self, msg: Bool) -> None:
        with self.lock:
            self.latest_elevator_mode = bool(msg.data)
            if msg.data and self.state in (WAIT_ELEVATOR_MODE, ALIGN_CABIN):
                self.enter_transit()
            elif (msg.data and self.state == TRACK_INGRESS
                  and self.elevator.get("align_exit_yaw", False)):
                if self.ingress_pose_inside_cabin():
                    # The lift may move before the local follower acknowledges
                    # arrival. Freeze immediately if the body is already inside.
                    self.enter_transit()
                else:
                    self.fail("elevator mode began before the body was inside the cabin")
            elif msg.data and self.state in (WAIT_DOOR_OPEN, ALIGN_EXIT):
                self.elevator_stopped_at = None
                self.transition(ELEVATOR_TRANSIT, "elevator resumed transit")
            elif (not msg.data and self.state == ELEVATOR_TRANSIT and
                  self.saw_elevator_mode):
                # Elevator-LIO can leave transit mode at a full stop before
                # the door opens. Wait for independent door evidence.
                self.exit_door_since = None
                self.exit_door_seen_at = float("-inf")
                self.elevator_stopped_at = self.now()
                self.transition(WAIT_DOOR_OPEN,
                                "elevator stopped; waiting for target height and open door")

    def on_door_state(self, msg: Bool) -> None:
        with self.lock:
            self.door_open = bool(msg.data)
            if self.state == WAIT_DOOR_OUTSIDE and self.door_open:
                self.try_start_ingress()

    def on_door_status(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(status, dict):
            return
        with self.lock:
            if self.state in (TRACK_LEG1, WAIT_DOOR_OUTSIDE):
                valid = (status.get("floor") == self.start_floor and status.get("side") == "lobby"
                         and status.get("open_confirmed") is True)
                self.source_door_seen_at = self.now() if valid else float("-inf")
                self.try_start_ingress()
                return
            if self.state != WAIT_DOOR_OPEN:
                return
            if (status.get("floor") != self.target_floor or status.get("side") != "cabin"
                    or status.get("open_confirmed") is not True or not self.ready_for_planning()):
                self.exit_door_since = None
                return
            now = self.now()
            if self.exit_door_since is None or now - self.exit_door_seen_at > self.odom_timeout:
                self.exit_door_since = now
            self.exit_door_seen_at = now

    def on_planner_result(self, msg: String) -> None:
        with self.lock:
            if self.state not in (TRACK_SINGLE, TRACK_LEG1, TRACK_INGRESS, TRACK_LEG2):
                return
            try:
                result = json.loads(msg.data)
                if float(result.get("stamp", self.active_segment_stamp.to_sec())) < self.active_segment_stamp.to_sec():
                    return
            except (AttributeError, TypeError, ValueError):
                return
            status = str(result.get("status", "")).lower()
            if self.preview_routes and status == "succeeded":
                self.transition(WAIT_GOAL, "global path preview ready")
                self.publish_result("planned", "global path preview ready")
            elif status == "failed":
                self.fail("planner failed: {}".format(result.get("message", "unknown")))
            elif status == "cancelled":
                self.transition(CANCELED, "planner canceled the active segment")

    def on_planner_status(self, msg: String) -> None:
        with self.lock:
            if self.state != WAIT_POSE or self.replacement_cancel_at is None:
                return
            try:
                status = json.loads(msg.data)
                stamp = float(status.get("stamp", 0.0))
            except (AttributeError, TypeError, ValueError):
                return
            if status.get("state") == "CANCELED" and stamp >= self.replacement_cancel_at.to_sec():
                self.replacement_cancel_at = None
                self.try_start_mission()

    def on_timer(self, _event=None) -> None:
        with self.lock:
            now = self.now()
            if (self.state == ELEVATOR_TRANSIT and
                    self.transit_started_at is not None and
                    now - self.transit_started_at > self.elevator_timeout_s):
                self.fail("elevator transit timeout")
                return
            if (self.state == WAIT_DOOR_OUTSIDE and self.door_wait_started_at is not None and
                    now - self.door_wait_started_at > self.door_open_timeout_s):
                self.fail("elevator door open timeout")
                return
            if self.state == WAIT_DOOR_OUTSIDE:
                self.try_start_ingress()
            if self.state == ALIGN_EXIT:
                if self.clear_of_exit_door():
                    rospy.loginfo("Elevator body is clear of the destination door; planning from current pose")
                    self.start_exit_leg()
                elif (self.cabin_alignment_started_at is not None and
                      now - self.cabin_alignment_started_at > self.door_plan_timeout_s):
                    self.fail("destination exit timed out while waiting for cabin alignment")
                return
            if (self.state == WAIT_DOOR_OPEN and self.elevator_stopped_at is not None and
                    now - self.elevator_stopped_at >= self.door_settle_s):
                if now - self.elevator_stopped_at > self.door_plan_timeout_s:
                    self.fail("destination exit timed out: " + self.message)
                    return
                if not self.ready_for_planning():
                    self.exit_door_since = None
                    self.message = "waiting for TRACKING localization after door open"
                    return
                target_cabin, _ = _inside_pose(self.elevator, self.target_floor)
                height_error = abs(self.ground_z() - target_cabin[2])
                if not height_error <= self.destination_height_tolerance:
                    self.message = "waiting for elevator to reach destination height"
                    return
                if (self.exit_door_since is None or now - self.exit_door_seen_at > self.odom_timeout
                        or now - self.exit_door_since < self.exit_door_confirm_s):
                    self.message = "at destination height; waiting for sustained door confirmation from cabin"
                    return
                if self.elevator.get("align_exit_yaw", False) and not self.manual_start:
                    self.start_cabin_alignment(ALIGN_EXIT)
                else:
                    self.start_exit_leg()

    def on_cancel(self, _msg: Empty) -> None:
        with self.lock:
            if self.preview_routes:
                self.request_preview()
                if self.state == WAIT_GOAL:
                    self.cancel_pub.publish(Empty())
            if self.state in (WAIT_GOAL, ARRIVED, FAILED, CANCELED):
                return
            self.transition(CANCELED, "mission canceled")
            self.publish_result("cancelled", "mission canceled")
            self.cancel_pub.publish(Empty())
            self.final_goal = None
            self.legs = []
            self.active_segment_stamp = rospy.Time()
            self.replacement_cancel_at = None


def main() -> int:
    rospy.init_node("elevator_mission_manager")
    try:
        ElevatorMissionManager()
    except Exception as exc:  # rospy startup must surface configuration errors.
        rospy.logfatal("Elevator mission manager: %s", exc)
        return 1
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
