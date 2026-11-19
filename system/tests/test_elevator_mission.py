#!/usr/bin/env python3
"""Pure route-selection tests for the elevator mission manager."""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "nav/src/dog_nav_bringup/scripts"))

from elevator_mission_manager import build_mission_legs, floor_for_z


FLOORS = {
    "floor1": {"height_z": 0.2},
    "floor2": {"height_z": 4.2},
}
ELEVATOR = {
    "id": "E1",
    "configured": True,
    "floors": {
        "floor1": {
            "lobby_pose": [2.0, 2.0, 0.2, 0.3],
            "inside_pose": [1.0, 2.0, 0.2, 0.3],
        },
        "floor2": {
            "lobby_pose": [2.1, 2.1, 4.2, -0.3],
            "inside_pose": [1.1, 2.1, 4.2, -0.3],
        },
    },
}


class ElevatorMissionRouteTest(unittest.TestCase):
    def test_floor_lookup_uses_support_height(self):
        self.assertEqual(floor_for_z(FLOORS, 0.2), "floor1")
        self.assertEqual(floor_for_z(FLOORS, 4.2), "floor2")
        self.assertEqual(floor_for_z(FLOORS, 4.0), "floor2")

    def test_same_floor_keeps_single_final_leg(self):
        legs = build_mission_legs(
            FLOORS, ELEVATOR, "floor1", "floor1", [5.0, 6.0, 0.2], 1.0
        )
        self.assertEqual(len(legs), 1)
        self.assertEqual(legs[0].kind, "FINAL")
        self.assertEqual(legs[0].xyz, (5.0, 6.0, 0.2))

    def test_cross_floor_uses_inside_pose_then_final_goal(self):
        legs = build_mission_legs(
            FLOORS, ELEVATOR, "floor1", "floor2", [7.0, 8.0, 4.2], -0.5
        )
        self.assertEqual([leg.kind for leg in legs], ["LOBBY", "INGRESS", "FINAL"])
        self.assertEqual(legs[0].xyz, (2.0, 2.0, 0.2))
        self.assertAlmostEqual(legs[0].yaw, 0.3)
        self.assertEqual(legs[1].xyz, (1.0, 2.0, 0.2))
        self.assertEqual(legs[2].xyz, (7.0, 8.0, 4.2))
        self.assertAlmostEqual(legs[2].yaw, -0.5)

    def test_unconfigured_elevator_is_rejected(self):
        elevator = dict(ELEVATOR, configured=False)
        with self.assertRaisesRegex(ValueError, "not configured"):
            build_mission_legs(
                FLOORS, elevator, "floor1", "floor2", [7.0, 8.0, 4.2], 0.0
            )

    def test_missing_destination_inside_pose_is_rejected(self):
        elevator = {
            "id": "E1",
            "configured": True,
            "floors": {"floor1": {
                "lobby_pose": [2.0, 2.0, 0.2, 0.0],
                "inside_pose": [1.0, 2.0, 0.2, 0.0],
            }},
        }
        with self.assertRaisesRegex(ValueError, "floor2"):
            build_mission_legs(
                FLOORS, elevator, "floor1", "floor2", [7.0, 8.0, 4.2], 0.0
            )


class ElevatorMissionSequenceTest(unittest.TestCase):
    def test_cross_floor_waits_for_target_height_and_open_door(self):
        from elevator_mission_manager import ElevatorMissionManager
        from std_msgs.msg import Bool, Header, String
        from unittest.mock import patch

        m = ElevatorMissionManager.__new__(ElevatorMissionManager)
        m.lock = __import__("threading").RLock()
        m.state = "WAIT_POSE"
        m.message = ""
        m.final_goal = object()
        m.final_ground_xyz = (7.0, 8.0, 4.2)
        m.final_yaw = -0.5
        m.target_floor = "floor2"
        m.start_floor = ""
        m.legs = []
        m.leg_index = 0
        m.active_segment_stamp = __import__("rospy").Time()
        m.saw_elevator_mode = False
        m.latest_elevator_mode = False
        m.elevator_stopped_at = None
        m.door_wait_started_at = None
        m.door_open = False
        m.source_door_seen_at = float("-inf")
        m.transit_started_at = None
        m.door_settle_s = 0.5
        m.exit_door_confirm_s = 1.0
        m.odom_timeout = 0.6
        m.destination_height_tolerance = 0.4
        m.ground_z = lambda: 3.0
        m.door_open_timeout_s = 180.0
        m.door_plan_timeout_s = 30.0
        m.elevator_timeout_s = 300.0
        m.elevator = ELEVATOR
        m.floors = FLOORS
        m.sent = []
        m.transitions = []
        m.results = []
        m.now = lambda: 100.0
        m.ready_for_planning = lambda: True
        m.current_floor = lambda: "floor1"
        m.publish_status = lambda: None
        m.publish_result = lambda status, message: m.results.append((status, message))
        m.transition = lambda state, message: (
            setattr(m, "state", state), setattr(m, "message", message),
            m.transitions.append((state, message))
        )
        m.cancel_pub = type("Pub", (), {"publish": lambda *args: None})()

        def send_leg(leg):
            m.sent.append(leg.kind)
            m.active_segment_stamp = __import__("rospy").Time(10 + 10 * len(m.sent))

        m.send_leg = send_leg
        m.try_start_mission()
        self.assertEqual(m.sent, ["LOBBY"])
        self.assertEqual(m.state, "TRACK_LEG1")

        m.on_goal_reached(Header(stamp=__import__("rospy").Time(20)))
        self.assertEqual(m.state, "WAIT_DOOR_OUTSIDE")
        m.on_door_state(Bool(data=True))
        self.assertEqual(m.state, "WAIT_DOOR_OUTSIDE")
        m.on_door_status(String(data='{"floor":"floor1","side":"cabin","open_confirmed":true}'))
        self.assertEqual(m.state, "WAIT_DOOR_OUTSIDE")
        m.ready_for_planning = lambda: False
        m.on_door_status(String(data='{"floor":"floor1","side":"lobby","open_confirmed":true}'))
        self.assertEqual(m.state, "WAIT_DOOR_OUTSIDE")
        m.ready_for_planning = lambda: True
        m.on_timer()
        self.assertEqual(m.state, "TRACK_INGRESS")
        self.assertEqual(m.sent, ["LOBBY", "INGRESS"])
        m.on_goal_reached(Header(stamp=__import__("rospy").Time(30)))
        self.assertEqual(m.state, "WAIT_ELEVATOR_MODE")
        m.on_elevator_mode(Bool(data=True))
        self.assertEqual(m.state, "ELEVATOR_TRANSIT")
        m.on_elevator_mode(Bool(data=False))
        self.assertEqual(m.state, "WAIT_DOOR_OPEN")

        m.elevator_stopped_at = 99.0
        with patch("elevator_mission_manager.rospy.Time.now", return_value=__import__("rospy").Time(30)):
            m.on_timer()
            self.assertEqual(m.state, "WAIT_DOOR_OPEN")
            self.assertEqual(m.sent, ["LOBBY", "INGRESS"])
            m.on_door_status(String(data='{"floor":"floor2","side":"cabin","open_confirmed":true}'))
            m.on_timer()
            self.assertEqual(m.sent, ["LOBBY", "INGRESS"])

            m.ground_z = lambda: 4.2
            for status in (
                '{"floor":"floor1","side":"cabin","open_confirmed":true}',
                '{"floor":"floor2","side":"lobby","open_confirmed":true}',
                '{"floor":"floor2","side":"cabin","open_confirmed":false}',
            ):
                m.on_door_status(String(data=status))
                m.on_timer()
                self.assertEqual(m.sent, ["LOBBY", "INGRESS"])

            opened = String(data='{"floor":"floor2","side":"cabin","open_confirmed":true}')
            m.on_door_status(opened)
            m.now = lambda: 100.5
            m.on_door_status(opened)
            m.on_localization(String(data="LOST_LIO_INPUT"))
            for current_time in (101.0, 101.5):
                m.now = lambda: current_time
                m.on_door_status(opened)
                m.on_timer()
                self.assertEqual(m.sent, ["LOBBY", "INGRESS"])
            m.now = lambda: 102.0
            m.on_door_status(opened)
            m.on_timer()
        self.assertEqual(m.sent, ["LOBBY", "INGRESS", "FINAL"])
        self.assertEqual(m.state, "TRACK_LEG2")

        m.on_goal_reached(Header(stamp=__import__("rospy").Time(50)))
        self.assertEqual(m.state, "ARRIVED")
        self.assertEqual(m.results[-1][0], "succeeded")


if __name__ == "__main__":
    unittest.main()
