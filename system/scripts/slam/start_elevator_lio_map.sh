#!/bin/bash
# Build a multi-floor map with Elevator-LIO ROS 1 and immediately generate the
# segmented FREE planning map. Ctrl-C stops Elevator-LIO gracefully so it can
# write its final TUM and merged PCD.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../env.sh"

START_LIDAR=true
LIO_CONFIG="${ELEVATOR_LIO_CONFIG:-root_config.yaml}"
LIO_SETUP="${ELEVATOR_LIO_SETUP:-/home/nvidia/ws_elevator_lio_ros1/devel/setup.bash}"
MARKER="$(mktemp)"
FINISHED=0
LIO_PID=""
LIDAR_PID=""
MASTER_PID=""

usage() {
  cat <<'USAGE'
用法:
  bash system/scripts/slam/start_elevator_lio_map.sh [start_lidar:=true|false] [config_path:=FILE]

环境变量:
  ELEVATOR_LIO_SETUP   Elevator-LIO ROS 1 devel/setup.bash
  ELEVATOR_LIO_CONFIG  Elevator-LIO yaml/ 下的根配置，默认 root_config.yaml
  DOG_NAV_MAP_PREFIX   输出地图前缀，默认 system/map/dog
USAGE
}

for arg in "$@"; do
  case "$arg" in
    start_lidar:=true|start_lidar:=1) START_LIDAR=true ;;
    start_lidar:=false|start_lidar:=0) START_LIDAR=false ;;
    config_path:=*) LIO_CONFIG="${arg#config_path:=}" ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数: $arg" >&2; usage >&2; exit 2 ;;
  esac
done

dog_nav_require_ros
dog_nav_source_nav
if [ ! -f "$LIO_SETUP" ]; then
  echo "错误: 找不到 Elevator-LIO ROS 1 环境: $LIO_SETUP" >&2
  echo "请设置 ELEVATOR_LIO_SETUP 指向编译后的 devel/setup.bash。" >&2
  exit 1
fi
dog_nav_source_file "$LIO_SETUP"

if ! rospack find lio >/dev/null 2>&1; then
  echo "错误: 当前 ROS 环境中找不到 lio 包。" >&2
  exit 1
fi
LIO_PACKAGE_ROOT="$(rospack find lio)"
PCD_DIR="${LIO_PACKAGE_ROOT}/PCD"
TEMP_DIR="${LIO_PACKAGE_ROOT}/temp"
TRAJ_FILE="${LIO_PACKAGE_ROOT}/traj.tum"
mkdir -p "$PCD_DIR" "$TEMP_DIR" "$DOG_NAV_MAP_DIR"

start_master_if_needed() {
  if rosnode list >/dev/null 2>&1; then
    return
  fi
  echo "未检测到 ROS master，启动 roscore..."
  roscore >"${DOG_NAV_MAP_DIR}/.elevator_lio_roscore.log" 2>&1 &
  MASTER_PID=$!
  for _ in $(seq 1 30); do
    rosnode list >/dev/null 2>&1 && return
    sleep 0.2
  done
  echo "错误: roscore 未能在 6 秒内启动。" >&2
  exit 1
}

stop_process() {
  local pid="${1:-}"
  if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
    kill -INT "$pid" 2>/dev/null || true
    for _ in $(seq 1 40); do
      kill -0 "$pid" 2>/dev/null || { wait "$pid" 2>/dev/null || true; return; }
      sleep 0.25
    done
    kill -TERM "$pid" 2>/dev/null || true
  fi
}

newest_file_since_marker() {
  local directory="$1"
  shift
  find "$directory" -maxdepth 1 -type f "$@" -newer "$MARKER" \
    -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2-
}

finish_map() {
  [ "$FINISHED" = 0 ] || return 0
  FINISHED=1
  echo "Elevator-LIO 已退出，正在收集本次建图结果..."

  SESSION_DIR="$(find "$TEMP_DIR" -mindepth 1 -maxdepth 1 -type d -newer "$MARKER" \
    -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2-)"
  if [ -z "$SESSION_DIR" ]; then
    echo "错误: 没找到本次 Elevator-LIO 日志目录。" >&2
    return 1
  fi

  MODE_LOG="${SESSION_DIR}/debug/elevator_debug.csv"
  if [ ! -s "$MODE_LOG" ]; then
    echo "错误: 本次运行没有电梯模式日志: $MODE_LOG" >&2
    echo "请确认 logging/default.yaml 中 elevator_debug.enabled=true。" >&2
    return 1
  fi
  if [ ! -s "$TRAJ_FILE" ]; then
    echo "错误: Elevator-LIO 未生成轨迹: $TRAJ_FILE" >&2
    return 1
  fi

  PCD_FILE="$(newest_file_since_marker "$PCD_DIR" -name '*_scans.pcd')"
  if [ -z "$PCD_FILE" ] || [ ! -s "$PCD_FILE" ]; then
    PCD_FILE="$(newest_file_since_marker "$PCD_DIR" -name '*.pcd' ! -name '*ikdtree*')"
  fi
  if [ -z "$PCD_FILE" ] || [ ! -s "$PCD_FILE" ]; then
    echo "错误: Elevator-LIO 未生成全局地图 PCD。" >&2
    return 1
  fi

  local mode_copy="${DOG_NAV_MAP_PREFIX}_elevator_debug.csv"
  local pcd_copy="${DOG_NAV_BUILDING_PCD}"
  local tum_copy="${DOG_NAV_TUM_FILE}"
  cp -f -- "$MODE_LOG" "$mode_copy"
  cp -f -- "$PCD_FILE" "$pcd_copy"
  cp -f -- "$TRAJ_FILE" "$tum_copy"
  echo "电梯模式日志: $(dog_nav_relative_path "$mode_copy")"
  echo "全局定位 PCD: $(dog_nav_relative_path "$pcd_copy")"
  echo "TUM 轨迹: $(dog_nav_relative_path "$tum_copy")"

  DOG_NAV_ELEVATOR_MODE_LOG="$mode_copy" \
  DOG_NAV_BUILDING_PCD="$pcd_copy" \
  DOG_NAV_TUM_FILE="$tum_copy" \
  DOG_NAV_MAP_PREFIX="$DOG_NAV_MAP_PREFIX" \
    bash "${SCRIPT_DIR}/../build_planning_map.sh"
  echo "Elevator-LIO 建图与分段 FREE 地图生成完成。"
}

cleanup() {
  stop_process "$LIO_PID"
  stop_process "$LIDAR_PID"
  if [ -n "$MASTER_PID" ]; then
    stop_process "$MASTER_PID"
  fi
}

trap 'echo; echo "收到中断，正在停止 Elevator-LIO 并保存地图..."; cleanup' INT TERM
trap cleanup EXIT

start_master_if_needed
if [ "$START_LIDAR" = true ]; then
  echo "启动 Livox MID-360 ROS 1 驱动..."
  roslaunch livox_ros_driver2 msg_MID360s.launch xfer_format:=1 \
    >"${DOG_NAV_MAP_DIR}/.elevator_lio_lidar.log" 2>&1 &
  LIDAR_PID=$!
  sleep 2
fi

echo "启动 Elevator-LIO: config=${LIO_CONFIG}"
rosrun lio lio "_config_path:=${LIO_CONFIG}" \
  >"${DOG_NAV_MAP_DIR}/.elevator_lio_mapping.log" 2>&1 &
LIO_PID=$!

for _ in $(seq 1 100); do
  if rosnode list 2>/dev/null | grep -Fxq '/lio_node'; then
    break
  fi
  if ! kill -0 "$LIO_PID" 2>/dev/null; then
    echo "错误: Elevator-LIO 启动失败，日志：" >&2
    tail -80 "${DOG_NAV_MAP_DIR}/.elevator_lio_mapping.log" >&2 || true
    exit 1
  fi
  sleep 0.1
done
if ! rosnode list 2>/dev/null | grep -Fxq '/lio_node'; then
  echo "错误: Elevator-LIO 节点未上线。" >&2
  exit 1
fi

cat <<'MESSAGE'
Elevator-LIO 正在建图。
请完成采集，然后按 Ctrl-C 停止。脚本会自动保存全局 PCD、TUM、
电梯模式日志，并生成不分层电梯竖直通道的 FREE 规划地图。
MESSAGE
while kill -0 "$LIO_PID" 2>/dev/null; do
  sleep 1
done
wait "$LIO_PID" || true
cleanup
finish_map
trap - EXIT
