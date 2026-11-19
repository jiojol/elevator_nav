#!/usr/bin/env python3
"""Drive the elevator mission manager without a chassis for planning smoke tests."""
from __future__ import annotations

import argparse
import json
import math
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Header, String


def vector(value: str):
    fields = [float(part) for part in value.split(",")]
    if len(fields) != 3 or not all(math.isfinite(item) for item in fields):
        raise argparse.ArgumentTypeError("expected x,y,z")
    return tuple(fields)


def main() -> int:
    parser = argparse.ArgumentParser(description="Elevator two-stage planning smoke test")
    parser.add_argument("--start-xyz", type=vector, default=vector("0,0,0"))
    parser.add_argument("--lobby-xyz", type=vector, default=vector("5.900,-5.709,0"))
    parser.add_argument("--inside-xyz", type=vector, default=vector("4.728,-5.709,0"))
    parser.add_argument("--destination-xyz", type=vector, default=vector("4.533,-5.478,7.25"))
    parser.add_argument("--goal-xyz", type=vector, default=vector("8.877,-4.334,6.85"))
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("elevator_planning_smoke", anonymous=True)
    paths = []

    def on_path(msg):
        if msg.poses and (not paths or msg.header.stamp > paths[-1].header.stamp):
            paths.append(msg)
            rospy.loginfo("smoke received global path %d: %d poses",
                          len(paths), len(msg.poses))

    rospy.Subscriber("/plan_map/global_path", Path, on_path, queue_size=10)
    odom_pub = rospy.Publisher("/dog_nav/odometry", Odometry, queue_size=10)
    localization_pub = rospy.Publisher(
        "/dog_nav/localization_status", String, queue_size=1, latch=True)
    goal_pub = rospy.Publisher("/elevator_nav/goal", PoseStamped, queue_size=1)
    reached_pub = rospy.Publisher("/planning/goal_reached", Header, queue_size=10)
    elevator_pub = rospy.Publisher(
        "/elevator_lio/in_elevator", Bool, queue_size=1)
    door_pub = rospy.Publisher("/elevator/door_open", Bool, queue_size=1, latch=True)
    door_status_pub = rospy.Publisher("/elevator/door_status", String, queue_size=1)
    floors = rospy.get_param("/elevator_mission_manager/floors")
    destination_floor = min(
        floors, key=lambda floor: abs(floors[floor]["height_z"] - args.goal_xyz[2]))
    rate = rospy.Rate(10)

    def publish_pose(xyz, tracking=True):
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = xyz
        msg.pose.pose.orientation.w = 1.0
        odom_pub.publish(msg)
        localization_pub.publish(String(data="TRACKING" if tracking else "REACQUIRING_MAP"))
        rate.sleep()

    def wait_paths(count, timeout, xyz):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while len(paths) < count and not rospy.is_shutdown() and rospy.Time.now() < deadline:
            publish_pose(xyz)
        return len(paths) >= count

    localization_pub.publish(String(data="TRACKING"))
    for _ in range(20):
        publish_pose(args.start_xyz)

    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"
    goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = args.goal_xyz
    goal.pose.orientation.w = 1.0
    goal_pub.publish(goal)
    if not wait_paths(1, args.timeout, args.start_xyz):
        rospy.logerr("first path was not generated")
        return 1

    reached = Header()
    reached.stamp = rospy.Time.now() + rospy.Duration(0.2)
    reached.frame_id = "map"
    reached_pub.publish(reached)
    for _ in range(15):
        publish_pose(args.lobby_xyz)

    source_floor = min(
        floors, key=lambda label: abs(floors[label]["height_z"] + .40 - args.start_xyz[2]))
    door_status_pub.publish(String(data=json.dumps({
        "floor": source_floor, "side": "lobby", "open_confirmed": True,
    })))
    door_pub.publish(Bool(data=True))
    if not wait_paths(2, args.timeout, args.lobby_xyz):
        rospy.logerr("cabin ingress path was not generated")
        return 1
    reached.stamp = rospy.Time.now() + rospy.Duration(0.2)
    reached_pub.publish(reached)
    for _ in range(15):
        publish_pose(args.inside_xyz)

    elevator_pub.publish(Bool(data=True))
    for _ in range(15):
        publish_pose(args.inside_xyz)

    # The manager must see the destination-floor odometry before normal mode is
    # restored, then wait for door settle and fresh TRACKING localization.
    for _ in range(20):
        publish_pose(args.destination_xyz)
    elevator_pub.publish(Bool(data=False))
    for _ in range(12):
        publish_pose(args.destination_xyz)
    if len(paths) != 2:
        rospy.logerr("final path was published before destination door opened")
        return 1

    deadline = rospy.Time.now() + rospy.Duration(args.timeout)
    while len(paths) < 3 and not rospy.is_shutdown() and rospy.Time.now() < deadline:
        door_status_pub.publish(String(data=json.dumps({
            "floor": destination_floor, "side": "cabin", "open_confirmed": True,
        })))
        publish_pose(args.destination_xyz)
    if len(paths) < 3:
        rospy.logerr("destination-floor final path was not generated")
        return 1

    for index, path in enumerate(paths[:3], 1):
        first = path.poses[0].pose.position
        last = path.poses[-1].pose.position
        length = sum(
            math.dist(
                (a.pose.position.x, a.pose.position.y, a.pose.position.z),
                (b.pose.position.x, b.pose.position.y, b.pose.position.z),
            )
            for a, b in zip(path.poses, path.poses[1:])
        )
        print(
            "PATH{} points={} length={:.3f} start=({:.3f},{:.3f},{:.3f}) "
            "end=({:.3f},{:.3f},{:.3f})".format(
                index, len(path.poses), length,
                first.x, first.y, first.z, last.x, last.y, last.z),
            flush=True,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
