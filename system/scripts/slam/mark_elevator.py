#!/usr/bin/env python3
"""Capture four poses from 3D Nav Goal in the elevator marking preview."""
import argparse
import math
import sys
from pathlib import Path

import numpy as np
import rospy
import yaml
from geometry_msgs.msg import Pose, PoseArray
from visualization_msgs.msg import Marker, MarkerArray

PROJECT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PROJECT / "nav/src/map_for_plan/scripts"))
from global_planner import GlobalPlanner, _load_graph, load_plan_meta


def ground_position(graph, name, point):
    radius = graph.resolution / math.sqrt(2) + 1e-5
    index = graph.nearest(point, radius)
    if index is None or abs(graph.points[index, 2] - point[2]) > .4:
        raise ValueError(f"{name}附近没有同层 FREE 体素，请点击地面")
    ground = point.copy()
    ground[2] = graph.points[index, 2]
    return ground


def describe_lobby(graph, mark):
    lobby, enter_heading, door = mark
    lobby = ground_position(graph, "候梯点", lobby)
    door = ground_position(graph, "门槛中点", door)
    distance = np.linalg.norm(lobby[:2] - door[:2])
    if distance < 1.25:
        raise ValueError(f"候梯点距门线只有 {distance:.2f} m，请沿门中心线向外约 1.5 m")
    if abs(lobby[2] - door[2]) > .35:
        raise ValueError("候梯点和门槛须在同一层地面")
    return lobby, enter_heading, door


def describe_stop(graph, lobby_mark, cabin_mark, width):
    """Use the first drag for the door center and the second for exit heading."""
    lobby, enter_heading, door = lobby_mark
    cabin, exit_heading, _ = cabin_mark
    cabin = ground_position(graph, "轿厢停车点", cabin)
    if max(lobby[2], door[2], cabin[2]) - min(lobby[2], door[2], cabin[2]) > .35:
        raise ValueError("轿厢停车点和门槛须在同一层地面")

    lobby_offset = lobby[:2] - door[:2]
    inside_offset = door[:2] - cabin[:2]
    depth = float(inside_offset @ (lobby_offset / np.linalg.norm(lobby_offset)))
    if depth <= .42:
        raise ValueError(f"停车点在门线内的深度约 {depth:.2f} m，须超过 0.42 m。请往轿厢内移，并给后壁和侧壁留余量")
    outward = inside_offset / np.linalg.norm(inside_offset)
    if lobby_offset @ outward < 1.25:
        raise ValueError("停车点须位于门中心线的轿厢一侧")
    across = np.array([-outward[1], outward[0]])
    if abs(lobby_offset @ across) >= min(.20, width / 4.):
        raise ValueError("停车点偏离门中心线，请沿候梯点和门槛延长线向轿厢内选点")
    for yaw, direction in ((enter_heading, -outward), (exit_heading, outward)):
        if np.dot((math.cos(yaw), math.sin(yaw)), direction) < math.cos(math.radians(20.)):
            raise ValueError("停车位姿要朝出口，请从轿厢内往门外拖，方向大致与候梯位姿相反")
    return float(door[2]), {
        "lobby_pose": lobby.tolist() + [enter_heading],
        "door_center_xy": door[:2].tolist(),
        "inside_pose": cabin.tolist() + [exit_heading],
        "ingress_brake_distance": .85,
    }


def validate_paths(graph, floor_name, floors, elevator):
    planner = GlobalPlanner.__new__(GlobalPlanner)
    planner.graph = graph
    planner.elevator = elevator
    planner.floors = floors
    planner.snap_radius = 2.0
    stop = elevator["floors"][floor_name]
    poses, indices = [], []
    for name in ("lobby_pose", "inside_pose"):
        xyz = stop[name][:3]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        pose.orientation.w = 1.
        poses.append(pose)
        indices.append(graph.nearest(xyz, 2.))
    planner.plan_ingress_preview(*poses)
    if not planner._exit_waypoints(graph.points[indices[1]], graph.points[indices[0]], floor_name):
        raise ValueError("出舱门线不与该楼层 FREE 地图相符")
    path, _ = planner._graph_route(indices[1], indices[0], floor_name)
    if path is None:
        raise ValueError("地图中没有通过门线到候梯点的同层出舱路径")


def capture_pose(label, parking=False, fixed_door=False):
    while not rospy.is_shutdown():
        if parking:
            answer = input(f"{label}：回车后从轿厢内拖向门外，也可输入 r 返回重标候梯点。")
            if answer.strip().lower() == "r":
                return None
        elif fixed_door:
            input(f"{label}：回车后从绿色候梯参考点附近拖向橙色门槛。")
        else:
            input(f"{label}：回车后从门外候梯点拖到同层门槛中点松开。")
        print("请使用标定预览里的 3D Nav Goal。", flush=True)
        message = rospy.wait_for_message("/elevator_mark/pose", PoseArray)
        if message.header.frame_id.lstrip("/") != "map" or len(message.poses) != 2:
            print("请将 RViz 的 Fixed Frame 设为 map，再拖拽一次。")
            continue
        pose, threshold = message.poses
        point = np.array([pose.position.x, pose.position.y, pose.position.z])
        door = np.array([threshold.position.x, threshold.position.y, threshold.position.z])
        qz, qw = pose.orientation.z, pose.orientation.w
        if not np.isfinite(np.r_[point, door, qz, qw]).all() or math.hypot(qz, qw) < .99:
            print("位姿或门槛坐标无效，请重新拖拽。")
            continue
        heading = math.atan2(2. * qz * qw, qw * qw - qz * qz)
        print(f"已记录 {tuple(round(value, 3) for value in point)}，朝向 {math.degrees(heading):.1f}°")
        return point, heading, door
    raise rospy.ROSInterruptException("标定已停止")


def show_guides(publisher, floor, door, reference=None):
    points = [(door, (1., .7, .1))]
    if reference is not None:
        points.extend([
            (reference["lobby_hint"], (.15, 1., .30)),
            (reference["cabin_hint"], (.15, .65, 1.)),
        ])
    markers = []
    for index, (position, color) in enumerate(points):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "calibration_door"
        marker.id = floor if index == 0 else 10 + floor * 3 + index
        marker.type = Marker.SPHERE
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
        marker.pose.orientation.w = 1.
        marker.scale.x = marker.scale.y = marker.scale.z = .18
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 1.
        markers.append(marker)
    publisher.publish(MarkerArray(markers=markers))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-dir", type=Path, required=True)
    args = parser.parse_args()
    map_dir = args.map_dir.resolve()
    floors_path, elevator_path = map_dir / "floors.yaml", map_dir / "elevator.yaml"
    if floors_path.exists() or elevator_path.exists():
        parser.error("该地图目录已有楼层或电梯配置，请先备份现有配置")
    graph = _load_graph(load_plan_meta(map_dir / "dog_plan.json"), .8, 6.)
    config_dir = PROJECT / "nav/src/dog_nav_bringup/config"
    defaults = yaml.safe_load((config_dir / "elevator.yaml").read_text())["elevator"]
    width = float(defaults["door_width"])
    height = float(defaults["door_height"])
    reference_path = map_dir / "elevator_door_reference.yaml"
    references = yaml.safe_load(reference_path.read_text())["floors"] if reference_path.is_file() else {}
    rospy.init_node("manual_elevator_marking", anonymous=True)
    guide = rospy.Publisher("/elevator_nav/markers", MarkerArray, queue_size=1, latch=True)
    elevator = {
        "id": "E1", "configured": True, "align_exit_yaw": True,
        "door_width": width, "door_height": height, "floors": {},
    }
    floors = {}
    for i, label in enumerate(("下层", "上层"), 1):
        name = f"floor{i}"
        reference = references.get(name)
        if reference is not None:
            door = np.array(reference["door"], dtype=float)
            show_guides(guide, i, door, reference)
            print(f"{label}已显示录包门槛参考 {tuple(door)}，橙色为门槛，绿色为候梯参考，蓝色为停车参考。")
        while True:
            try:
                mark = capture_pose(f"{label}候梯位姿", fixed_door=reference is not None)
                if reference is not None:
                    mark = (mark[0], mark[1], door)
                lobby = describe_lobby(graph, mark)
                if i == 2 and lobby[2][2] - floors["floor1"]["height_z"] < 1.:
                    raise ValueError("请在上层点选，楼层的地面高度应高于下层")
            except ValueError as error:
                print(f"{label}候梯点或门槛校验失败：{error}。")
                continue
            print(f"门槛已锁定 {tuple(round(value, 3) for value in lobby[2])}，RViz 中显示为橙色球。")
            show_guides(guide, i, lobby[2], reference)
            while True:
                cabin = capture_pose(f"{label}轿厢停车位姿", parking=True)
                if cabin is None:
                    break
                try:
                    z, stop = describe_stop(graph, lobby, cabin, width)
                    floors[name] = {"height_z": z, "goal_topic": f"/map{i}/goal"}
                    elevator["floors"][name] = stop
                    validate_paths(graph, name, floors, elevator)
                    break
                except ValueError as error:
                    print(f"{label}停车位姿校验失败：{error}。请重标停车位姿。")
            if cabin is not None:
                break
    floors_config = yaml.safe_load((config_dir / "floors.yaml").read_text())
    floors_config["floors"] = floors
    print(yaml.safe_dump({"floors": floors, "elevator": elevator}, sort_keys=False))
    if input("请核对显示的点位，输入 yes 保存：").strip().lower() != "yes":
        return
    with floors_path.open("x") as stream:
        yaml.safe_dump(floors_config, stream, sort_keys=False)
    with elevator_path.open("x") as stream:
        yaml.safe_dump({"elevator": elevator}, stream, sort_keys=False)
    print(f"已保存 {floors_path} 和 {elevator_path}")


if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, EOFError, rospy.ROSInterruptException):
        sys.exit("标定结束")
