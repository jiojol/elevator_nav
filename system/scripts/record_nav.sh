#!/bin/bash
# Record navigation inputs, outputs, ROS diagnostics and optional camera video.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.sh"

make_video=false
output=""
camera_topic="${DOG_NAV_CAMERA_TOPIC:-/camera/image_raw/compressed}"
while [ "$#" -gt 0 ]; do
  case "$1" in
    --video) make_video=true; shift ;;
    --output) output="${2:?--output requires a directory}"; shift 2 ;;
    --camera-topic) camera_topic="${2:?--camera-topic requires a topic}"; shift 2 ;;
    -h|--help)
      echo 'Usage: bash system/scripts/record_nav.sh [--video] [--output DIR] [--camera-topic TOPIC]'
      exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

dog_nav_require_nav
dog_nav_source_nav
if ! rostopic list >/dev/null 2>&1; then
  echo '错误: ROS 未运行。请先启动导航，再在另一个终端录制。' >&2
  exit 1
fi
stamp="$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)"
output="${output:-${DOG_NAV_HOME}/system/bags/nav_${stamp}_$$}"
mkdir -p "$(dirname "$output")"
mkdir "$output"
output="$(cd "$output" && pwd)"

snapshot() {
  local phase="$1"
  rosparam dump "${output}/params_${phase}.yaml" 2>>"${output}/snapshot_errors.log" || true
  rostopic list -v >"${output}/topics_${phase}.txt" 2>>"${output}/snapshot_errors.log" || true
  rosnode list >"${output}/nodes_${phase}.txt" 2>>"${output}/snapshot_errors.log" || true
  local log_root="${ROS_LOG_DIR:-${ROS_HOME:-${HOME}/.ros}/log}"
  local run_id log_dir
  run_id="$(rosparam get /run_id 2>/dev/null || true)"
  log_dir="${log_root}/latest"
  if [[ "$run_id" =~ ^[a-zA-Z0-9_-]+$ ]] && [ -d "${log_root}/${run_id}" ]; then
    log_dir="${log_root}/${run_id}"
  fi
  if [ -d "$log_dir" ]; then
    mkdir -p "${output}/ros_logs_${phase}"
    cp -a "${log_dir}/." "${output}/ros_logs_${phase}/" 2>>"${output}/snapshot_errors.log" || true
  fi
}

snapshot start
cp "${DOG_NAV_HOME}/nav/src/dog_nav_bringup/config/prior_local.yaml" "${output}/scan.yaml"
cp "${DOG_NAV_HOME}/nav/src/dog_nav_bringup/launch/navigation.launch" "${output}/navigation.launch"
if [ -f "$DOG_NAV_PLAN_META" ]; then
  cp "$DOG_NAV_PLAN_META" "${output}/plan_meta.json"
fi

topics=(
  /tf /tf_static /rosout /rosout_agg
  /livox/lidar /livox/imu /livox/points
  /fastlio_nav/odometry /fastlio_nav/cloud_body /fastlio_nav/path
  /dog_nav/odometry /dog_nav/sensor_pose /dog_nav/localization_status /dog_nav/goal_pose
  /planning/body_odometry
  /navigation/obstacle_points /globalmap
  /initialpose /move_base_simple/goal /initial_path
  /plan_map/start_pose /plan_map/goal_pose /plan_map/global_path
  /plan_map/free /plan_map/path /plan_map/query_start /plan_map/query_goal /plan_map/cancel_scan
  /planning/tracking_path /planning/bspline /planning/data_display /planning/go2_execution_frozen /planning/tracking_status /planning/local_planner_status
  /planning/goal_position_reached /planning/goal_position_lost /planning/goal_reached /planning/goal_failed
  /planning/reference_refresh /planning/start_alignment /planning/start_alignment_done
  /dog_nav/safety_stop /dog_nav/safety/reference /dog_nav/safety/bspline
  /dog_nav/safety/odometry /dog_nav/safety/obstacle_points
  /scan_planner_node/goal_point /scan_planner_node/global_list
  /scan_planner_node/init_list /scan_planner_node/optimal_list /scan_planner_node/a_star_list
  /scan_planner_node/self_inflation
  /grid_map/occupancy /grid_map/occupancy_inflate /grid_map/sliding_map_bbox
  /cmd_vel /planning/scan_cmd_vel /go2/odom /go2/status /go2/sportmodestate /go2/imu
  /map_manager/current_map /map_manager/status /map_manager/result /map_manager/cancel
  /camera/camera_info "$camera_topic"
)
printf '%s\n' "${topics[@]}" >"${output}/recorded_topics.txt"
echo "录制目录: ${output}"
echo '记录 bag（LZ4 压缩，每 2 GB 分包）、运行参数及 ROS 日志。'
echo '按 Ctrl+C 结束并等待收尾；--video 会在收尾时导出摄像头 MP4。'

record_pid=""
finish() {
  local status="$?"
  trap - EXIT INT TERM
  if [ -n "$record_pid" ] && kill -0 "$record_pid" 2>/dev/null; then
    kill -INT "$record_pid" 2>/dev/null || true
    wait "$record_pid" || true
  fi
  snapshot end
  if [ "$make_video" = true ]; then
    python3 "${SCRIPT_DIR}/export_nav_video.py" "$output" --topic "$camera_topic" || \
      echo '摄像头视频未导出；bag 和诊断日志已保留。' >&2
  fi
  echo "录制已结束: ${output}"
  exit "$status"
}
trap finish EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
rosbag record --lz4 --split --size=2048 --buffsize=256 \
  -O "${output}/navigation.bag" "${topics[@]}" >"${output}/recorder.log" 2>&1 &
record_pid="$!"
wait "$record_pid"
