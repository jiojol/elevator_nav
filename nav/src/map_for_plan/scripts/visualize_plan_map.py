#!/usr/bin/env python3
"""在 RViz 里发布规划地图（FREE 可通行体素）。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from plan_io import load_plan_meta, read_pcd_xyz


def to_cloud(points: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    msg = PointCloud2()
    msg.header = Header(stamp=stamp, frame_id=frame_id)
    msg.height, msg.width = 1, int(points.shape[0])
    msg.fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * msg.width
    msg.is_dense = True
    msg.data = np.asarray(points, dtype=np.float32).reshape(-1).tobytes()
    return msg


def main() -> int:
    parser = argparse.ArgumentParser(description="发布规划地图到 RViz")
    parser.add_argument("--meta", type=Path, required=True, help="包含 free_pcd、resolution 的规划地图元数据")
    parser.add_argument("--frame-id", default="map")
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    meta_path = args.meta.expanduser().resolve()
    meta = load_plan_meta(meta_path)
    free_pts = read_pcd_xyz(Path(meta["free_pcd"]))

    rospy.init_node("visualize_plan_map")
    frame_id = rospy.get_param("~frame_id", args.frame_id)
    rospy.set_param("/plan_map/resolution", float(meta["resolution"]))
    free_pub = rospy.Publisher("/plan_map/free", PointCloud2, queue_size=1, latch=True)
    rospy.loginfo("FREE=%d res=%.2f", len(free_pts), float(meta.get("resolution", 0.2)))

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        stamp = rospy.Time.now()
        free_pub.publish(to_cloud(free_pts, frame_id, stamp))
        rate.sleep()
    return 0


if __name__ == "__main__":
    sys.exit(main())
