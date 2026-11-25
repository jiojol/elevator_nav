#!/bin/bash
# Build a multi-floor map with Elevator-LIO ROS 1 and immediately generate the
# segmented FREE planning map. Ctrl-C stops Elevator-LIO gracefully so it can
# write its final TUM and merged PCD.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../env.sh"

START_LIDAR=true
START_RVIZ=true
REFINE_START_LOOP=false
LIO_CONFIG="${ELEVATOR_LIO_CONFIG:-root_config.yaml}"
LIO_SETUP="${ELEVATOR_LIO_SETUP:-$HOME/ws_elevator_lio_ros1/devel/setup.bash}"
BAG_DIR="${ELEVATOR_MAPPING_BAG_DIR:-$HOME/bags/elevator_mapping}"
STOP_REQUESTED=0
LIO_PID=""
LIDAR_PID=""
MASTER_PID=""
BAG_PID=""
RVIZ_PID=""

usage() {
  cat <<'USAGE'
用法:
  bash system/scripts/slam/start_elevator_lio_map.sh [start_lidar:=true|false] [rviz:=true|false] [loop_closure:=true|false] [config_path:=FILE]
  有图形显示时默认打开建图 RViz。
  loop_closure:=true 在保存后校验起点回访，输出单独的回环修正地图。

环境变量:
  ELEVATOR_LIO_SETUP   Elevator-LIO ROS 1 devel/setup.bash
  ELEVATOR_LIO_CONFIG  Elevator-LIO yaml/ 下的根配置，默认 root_config.yaml
  ELEVATOR_MAPPING_BAG_DIR  电梯建图录包目录，默认 ~/bags/elevator_mapping
  DOG_NAV_MAP_DIR      输出地图目录，默认 system/map
  DOG_NAV_MAP_PREFIX   输出地图前缀，默认 system/map/dog
USAGE
}

for arg in "$@"; do
  case "$arg" in
    start_lidar:=true|start_lidar:=1) START_LIDAR=true ;;
    start_lidar:=false|start_lidar:=0) START_LIDAR=false ;;
    rviz:=true|rviz:=1) START_RVIZ=true ;;
    rviz:=false|rviz:=0) START_RVIZ=false ;;
    loop_closure:=true|loop_closure:=1) REFINE_START_LOOP=true ;;
    loop_closure:=false|loop_closure:=0) REFINE_START_LOOP=false ;;
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
set +u
source "$LIO_SETUP" --extend
set -u

if ! rospack find lio >/dev/null 2>&1; then
  echo "错误: 当前 ROS 环境中找不到 lio 包。" >&2
  exit 1
fi
LIO_PACKAGE_ROOT="$(rospack find lio)"
PCD_DIR="${LIO_PACKAGE_ROOT}/PCD"
TEMP_DIR="${LIO_PACKAGE_ROOT}/temp"
TRAJ_FILE="${LIO_PACKAGE_ROOT}/traj.tum"
mkdir -p "$PCD_DIR" "$TEMP_DIR" "$DOG_NAV_MAP_DIR" "$BAG_DIR"

exec 9>"$BAG_DIR/.mapping.lock"
if ! flock -n 9; then
  echo "错误: 已有电梯建图在使用录包目录: $BAG_DIR" >&2
  exit 1
fi
BAG_PATH="$BAG_DIR/elevator_mapping_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)_$$.bag"
BAG_PENDING="${BAG_PATH%.bag}.pending.bag"
MARKER="$(mktemp)"

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
  local pid="$1"
  if [ -n "$pid" ]; then
    kill -INT "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  fi
}

start_mapping_rviz() {
  [ "$START_RVIZ" = true ] || return 0
  local config="$LIO_PACKAGE_ROOT/rviz/LIO.rviz"
  if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
    echo "当前终端无图形显示。可在机器狗桌面执行 rviz -d \"$config\" 查看建图。"
    return
  fi
  if ! command -v rviz >/dev/null || [ ! -r "$config" ]; then
    echo "警告: 找不到 RViz 或建图显示配置 $config。" >&2
    return
  fi
  echo "打开建图 RViz: $config"
  rviz -d "$config" __name:=elevator_mapping_rviz \
    >"${DOG_NAV_MAP_DIR}/.elevator_lio_rviz.log" 2>&1 9>&- &
  RVIZ_PID=$!
}

finish_recording() {
  if [ ! -s "$BAG_PENDING" ] || ! rosbag info --yaml "$BAG_PENDING" | python3 -c '
import sys
import yaml
info = yaml.safe_load(sys.stdin) or {}
topics = {t["topic"] for t in info.get("topics", []) if t["messages"] > 0}
sys.exit(not {"/livox/lidar", "/livox/imu"}.issubset(topics))
'; then
    echo "错误: 本次录包未完成或缺少雷达/IMU 消息。保留此前录包，请检查 $BAG_PENDING" >&2
    return 1
  fi
  mv -- "$BAG_PENDING" "$BAG_PATH" || return
  echo "建图录包: $BAG_PATH"
  local -a bags
  local i
  mapfile -t bags < <(
    find "$BAG_DIR" -maxdepth 1 -regextype posix-extended -type f \
      -regex '.*/elevator_mapping_[0-9]{8}_[0-9]{6}_[0-9]+\.bag' \
      -printf '%T@ %f\n' | sort -nr | cut -d' ' -f2-
  )
  for ((i = 2; i < ${#bags[@]}; i++)); do
    rm -- "$BAG_DIR/${bags[i]}" || return
  done
}

newest_file_since_marker() {
  local directory="$1"
  shift
  find "$directory" -maxdepth 1 -type f "$@" -newer "$MARKER" \
    -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2-
}

finish_map() {
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
  if [ ! -s "$TRAJ_FILE" ] || [ ! "$TRAJ_FILE" -nt "$MARKER" ]; then
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

refine_start_loop() {
  local sensor_config
  sensor_config="$(python3 - "${SESSION_DIR}/yaml/$LIO_CONFIG" <<'PY'
from pathlib import Path
import sys
import yaml

print(yaml.safe_load(Path(sys.argv[1]).read_text())["sensor_config"])
PY
)" || return
  OPENBLAS_NUM_THREADS=1 python3 "${SCRIPT_DIR}/refine_elevator_start_loop.py" \
    --bag "$BAG_PATH" \
    --map-prefix "$DOG_NAV_MAP_PREFIX" \
    --sensor-config "${SESSION_DIR}/yaml/$sensor_config" \
    --output-dir "${DOG_NAV_MAP_DIR%/}_loop"
}

cleanup() {
  trap '' INT TERM
  if [ -n "$RVIZ_PID" ]; then
    kill -TERM "$RVIZ_PID" 2>/dev/null || true
  fi
  stop_process "$LIO_PID"
  stop_process "$LIDAR_PID"
  stop_process "$BAG_PID"
  stop_process "$MASTER_PID"
}

trap 'STOP_REQUESTED=1' INT TERM
trap 'cleanup; rm -f -- "$MARKER"' EXIT

start_master_if_needed
if rosnode list | grep -Exq '/lio(_node)?'; then
  echo "错误: 已有 Elevator-LIO 节点在运行，请先结束已有建图或导航。" >&2
  exit 1
fi
echo "建图同步录包: $BAG_PATH"
rosbag record --lz4 --buffsize=256 -O "$BAG_PENDING" \
  /livox/lidar /livox/imu /LIO/odom_imu /LIO/in_elevator /LIO/elevator_state \
  /LIO/set_elevator_flag /tf /tf_static \
  >"${DOG_NAV_MAP_DIR}/.elevator_lio_recording.log" 2>&1 &
BAG_PID=$!
for _ in $(seq 1 50); do
  [ "$STOP_REQUESTED" = 0 ] || exit 130
  [ ! -e "$BAG_PENDING.active" ] || break
  kill -0 "$BAG_PID" 2>/dev/null || break
  sleep 0.1
done
if [ ! -e "$BAG_PENDING.active" ]; then
  echo "错误: 建图录包未能启动，请检查 ${DOG_NAV_MAP_DIR}/.elevator_lio_recording.log" >&2
  exit 1
fi

if [ "$START_LIDAR" = true ]; then
  echo "启动 Livox MID-360 ROS 1 驱动..."
  roslaunch livox_ros_driver2 msg_MID360.launch xfer_format:=1 \
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

start_mapping_rviz

cat <<'MESSAGE'
Elevator-LIO 正在建图。
请完成采集，然后按 Ctrl-C 停止，等待录包写完索引并保存地图。
脚本会保存全局 PCD、TUM、电梯模式日志，并生成排除乘梯竖直段的 FREE 地图。
MESSAGE
while [ "$STOP_REQUESTED" = 0 ] && kill -0 "$LIO_PID" 2>/dev/null; do
  if ! kill -0 "$BAG_PID" 2>/dev/null; then
    echo "错误: 建图录包进程退出，请检查 ${DOG_NAV_MAP_DIR}/.elevator_lio_recording.log" >&2
    exit 1
  fi
  sleep 0.25 || true
done
echo "正在关闭录包并保存地图..."
cleanup
trap - INT TERM
trap 'rm -f -- "$MARKER"' EXIT
bag_status=0
finish_recording || bag_status=$?
finish_map
if [ "$REFINE_START_LOOP" = true ] && [ "$bag_status" = 0 ]; then
  echo "正在校验起点回访并重建回环地图..."
  if ! refine_start_loop; then
    echo "回环修正未生成，原始地图和录包已保存。" >&2
  fi
fi
exit "$bag_status"
