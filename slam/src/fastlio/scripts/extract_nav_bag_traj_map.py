#!/usr/bin/env python3

"""Extract a walked TUM path and a voxel map from a navigation bag.

测绘学院副楼34楼.bag has no /matecam/odom and stores /livox/lidar as PointCloud2
without per-point timestamps. Fast-LIO offline mapping stalled on that bag.
The localized /odom poses and /aligned_points clouds already live in map
and are the consistent source for the FREE-corridor pipeline.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Extract /odom TUM and a voxelized /aligned_points map from a ROS1 bag."
    )
    parser.add_argument(
        "--bag",
        type=Path,
        default=Path("/media/xzq/数据/斯打第/数据集合/测绘学院副楼34楼.bag"),
    )
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--cloud-topic", default="/aligned_points")
    parser.add_argument(
        "--traj",
        type=Path,
        default=Path("/home/xzq/fast-lio-loop（复件）/PCD/pcd/fastlio_traj_matecam_测绘学院副楼34.tum"),
    )
    parser.add_argument(
        "--map",
        type=Path,
        default=Path("/home/xzq/fast-lio-loop（复件）/PCD/pcd/fastlio_map_matecam_测绘学院副楼34.pcd"),
    )
    parser.add_argument("--resolution", type=float, default=0.05)
    return parser


def iter_xyz(msg) -> np.ndarray:
    off_x = off_y = off_z = None
    for field in msg.fields:
        if field.name == "x":
            off_x = field.offset
        elif field.name == "y":
            off_y = field.offset
        elif field.name == "z":
            off_z = field.offset
    if None in (off_x, off_y, off_z):
        return np.zeros((0, 3), dtype=np.float32)
    step = msg.point_step
    n = msg.width * msg.height
    if off_x == 0 and off_y == 4 and off_z == 8:
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, step)
        return np.asarray(raw[:, :12].view(np.float32), dtype=np.float32).reshape(n, 3)
    pts = np.empty((n, 3), dtype=np.float32)
    data = msg.data
    for i in range(n):
        base = i * step
        pts[i, 0] = np.frombuffer(data, dtype="<f4", count=1, offset=base + off_x)[0]
        pts[i, 1] = np.frombuffer(data, dtype="<f4", count=1, offset=base + off_y)[0]
        pts[i, 2] = np.frombuffer(data, dtype="<f4", count=1, offset=base + off_z)[0]
    return pts


def write_pcd_xyz(path: Path, points: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    count = int(points.shape[0])
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        "WIDTH {c}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {c}\nDATA binary\n"
    ).format(c=count)
    with path.open("wb") as stream:
        stream.write(header.encode("ascii"))
        stream.write(np.asarray(points, dtype=np.float32).tobytes(order="C"))


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    try:
        import rosbag
    except ImportError:
        print("error: source the ROS1 environment before extraction", file=sys.stderr)
        return 2
    if not args.bag.is_file():
        print("error: bag does not exist: {}".format(args.bag), file=sys.stderr)
        return 2
    if args.resolution <= 0.0:
        print("error: resolution must be positive", file=sys.stderr)
        return 2

    args.traj.parent.mkdir(parents=True, exist_ok=True)
    voxels = {}
    n_odom = 0
    n_clouds = 0
    bag = rosbag.Bag(str(args.bag))
    try:
        with args.traj.open("w", encoding="utf-8") as traj:
            traj.write("# timestamp tx ty tz qx qy qz qw\n")
            for topic, msg, _stamp in bag.read_messages(
                topics=[args.odom_topic, args.cloud_topic]
            ):
                if topic == args.odom_topic or topic == args.odom_topic.lstrip("/"):
                    pose = msg.pose.pose
                    traj.write(
                        "{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(
                            msg.header.stamp.to_sec(),
                            pose.position.x,
                            pose.position.y,
                            pose.position.z,
                            pose.orientation.x,
                            pose.orientation.y,
                            pose.orientation.z,
                            pose.orientation.w,
                        )
                    )
                    n_odom += 1
                    continue
                pts = iter_xyz(msg)
                n_clouds += 1
                finite = pts[np.isfinite(pts).all(axis=1)]
                if finite.size == 0:
                    continue
                keys = np.floor(finite / args.resolution).astype(np.int32)
                for key, point in zip(keys, finite):
                    voxel = (int(key[0]), int(key[1]), int(key[2]))
                    acc = voxels.get(voxel)
                    if acc is None:
                        voxels[voxel] = [float(point[0]), float(point[1]), float(point[2]), 1]
                    else:
                        acc[0] += float(point[0])
                        acc[1] += float(point[1])
                        acc[2] += float(point[2])
                        acc[3] += 1
    finally:
        bag.close()

    if n_odom < 2:
        print("error: not enough poses on {}".format(args.odom_topic), file=sys.stderr)
        return 2
    if not voxels:
        print("error: no points on {}".format(args.cloud_topic), file=sys.stderr)
        return 2

    points = np.array(
        [[acc[0] / acc[3], acc[1] / acc[3], acc[2] / acc[3]] for acc in voxels.values()],
        dtype=np.float64,
    )
    write_pcd_xyz(args.map, points)
    print(
        "wrote {} poses={}  {} points={} clouds={}".format(
            args.traj, n_odom, args.map, len(points), n_clouds
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
