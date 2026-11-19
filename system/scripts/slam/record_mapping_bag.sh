#!/bin/bash
# Record the sensor streams required to replay mapping, plus the gimbal camera.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../env.sh"

dog_nav_require_slam
dog_nav_source_slam

if ! rostopic list >/dev/null 2>&1; then
  echo "错误: ROS 未运行。请先启动建图，再在另一个终端录制。" >&2
  exit 1
fi

bag_dir="${DOG_NAV_HOME}/system/bags"
mkdir -p "$bag_dir"
stamp="$(date +%Y%m%d_%H%M%S)"
bag_path="${1:-${bag_dir}/mapping_${stamp}.bag}"

topics=(
  /livox/lidar
  /livox/imu
  /go2/odom
  /camera/image_raw/compressed
)

for topic in "${topics[@]}"; do
  publishers="$(rostopic info "$topic" 2>/dev/null | awk '/Publishers:/{p=1;next} /Subscribers:/{p=0} p && $1=="*"{c++} END{print c+0}')"
  if [ "$publishers" -eq 0 ]; then
    echo "警告: ${topic} 当前没有发布者，这一路会录成空话题。" >&2
  fi
done

echo "录制到: ${bag_path}"
echo "话题: ${topics[*]}"
echo "按 Ctrl+C 结束，等 rosbag 写完索引再关闭终端。"
exec rosbag record --lz4 --buffsize=256 -O "$bag_path" "${topics[@]}"
