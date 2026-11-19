#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../env.sh"

dog_nav_require_slam
dog_nav_source_slam
for node_spec in 'fastlio map_builder_node 回环建图' 'livox_ros_driver2 livox_ros_driver2_node Livox'; do
  read -r node_package node_name node_label <<< "$node_spec"
  node_path="$(catkin_find --libexec "$node_package" "$node_name" 2>/dev/null | head -n 1 || true)"
  if [ -z "$node_path" ] || [ ! -x "$node_path" ]; then
    echo "错误: 找不到 ${node_label} 可执行文件，请在本机重新编译 slam 工作空间。" >&2
    exit 1
  fi
  missing_libraries="$(ldd "$node_path" 2>/dev/null | grep 'not found' || true)"
  if [ -n "$missing_libraries" ]; then
    echo "错误: ${node_label} 可执行文件缺少运行库，请在本机重新编译：" >&2
    echo "$missing_libraries" >&2
    exit 1
  fi
done

if rosnode ping -c 1 /map_builder_node 2>/dev/null | grep -q 'xmlrpc reply' \
   || rosnode ping -c 1 /laserMapping 2>/dev/null | grep -q 'xmlrpc reply'; then
  echo "错误: 建图节点已在运行，请先在原建图终端按 Ctrl+C。" >&2
  exit 1
fi

if rosnode ping -c 1 /livox_lidar_publisher2 2>/dev/null | grep -q 'xmlrpc reply'; then
  for launch_arg in "$@"; do
    case "$launch_arg" in
      start_lidar:=true|start_lidar:=1)
        echo "错误: Livox 驱动已在运行，请先关闭原驱动，或使用 start_lidar:=false 复用它。" >&2
        exit 1
        ;;
      start_lidar:=false|start_lidar:=0)
        break
        ;;
    esac
  done
  if ! printf '%s\n' "$@" | grep -Eq '^start_lidar:=(false|0)$'; then
    set -- "$@" 'start_lidar:=false'
  fi
  echo "检测到运行中的 Livox 驱动，将复用其 /livox/lidar 和 /livox/imu 话题。"
fi

has_rviz_arg=false
for launch_arg in "$@"; do
  case "$launch_arg" in
    rviz:=*) has_rviz_arg=true ;;
  esac
done
if [ "$has_rviz_arg" = false ]; then
  if [ -n "${DISPLAY:-}" ] || [ -n "${WAYLAND_DISPLAY:-}" ]; then
    set -- "$@" 'rviz:=true'
  else
    set -- "$@" 'rviz:=false'
    echo "当前终端没有图形显示，建图继续运行；可在图形桌面打开 RViz："
    echo "rviz -d ${DOG_NAV_HOME}/slam/src/fastlio/rviz/mapping.rviz"
  fi
fi

if [ "${DOG_NAV_SLAM_NEW_TERMINAL:-0}" = 1 ] && [ -z "${DOG_NAV_SLAM_IN_TERMINAL:-}" ] && [ -n "${DISPLAY:-}" ] && command -v gnome-terminal >/dev/null 2>&1; then
  DOG_NAV_SLAM_IN_TERMINAL=1 gnome-terminal --title="FAST-LIO Loop Mapping" -- \
    bash -c 'bash "$@"; result=$?; read -r -p "按回车关闭建图窗口..."; exit "$result"' \
    _ "$0" "$@"
  exit 0
fi

mkdir -p "$DOG_NAV_MAP_DIR"
mkdir -p "$(dirname "$DOG_NAV_BUILDING_PCD")" "$(dirname "$DOG_NAV_TUM_FILE")" "$(dirname "$DOG_NAV_MAP_PREFIX")" "$(dirname "$DOG_NAV_PLAN_META")"
session_dir="$(mktemp -d "${DOG_NAV_MAP_DIR}/.slam-odom.XXXXXX")"
session_pcd="${session_dir}/scans.pcd"
session_tum="${session_dir}/trajectory.tum"
session_prefix="${session_dir}/$(basename "$DOG_NAV_MAP_PREFIX")"
final_tum="$DOG_NAV_TUM_FILE"
session_published=0

finish_map() {
  if [ "$session_published" = 1 ]; then
    return 0
  fi
  if [ ! -s "$session_pcd" ] || [ ! -s "$session_tum" ]; then
    echo "错误: 回环建图未保存本次点云或 TUM 轨迹。" >&2
    echo "PCD: $session_pcd; TUM: $session_tum" >&2
    return 1
  fi
  if ! DOG_NAV_TUM_FILE="$session_tum" \
    DOG_NAV_MAP_PREFIX="$session_prefix" \
    DOG_NAV_PLAN_META="${session_prefix}_plan.json" \
    bash "${SCRIPT_DIR}/../build_planning_map.sh"; then
    echo "错误: 规划地图生成失败，本次点云和轨迹保留在 $session_dir" >&2
    return 1
  fi
  python3 - "${session_prefix}_plan.json" "$final_tum" <<'PY'
import json
import sys
from pathlib import Path

meta_path = Path(sys.argv[1])
meta = json.loads(meta_path.read_text(encoding="utf-8"))
meta["traj"] = str(Path(sys.argv[2]).resolve())
meta_path.write_text(json.dumps(meta, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
PY
  mv -f -- "$session_pcd" "$DOG_NAV_BUILDING_PCD"
  mv -f -- "$session_tum" "$final_tum"
  mv -f -- "${session_prefix}_free.pcd" "${DOG_NAV_MAP_PREFIX}_free.pcd"
  mv -f -- "${session_prefix}_path.tum" "${DOG_NAV_MAP_PREFIX}_path.tum"
  mv -f -- "${session_prefix}_plan.json" "$DOG_NAV_PLAN_META"
  session_published=1
  echo "定位点云: $(dog_nav_relative_path "$DOG_NAV_BUILDING_PCD")"
  echo "TUM 轨迹: $(dog_nav_relative_path "$final_tum")"
  echo "规划地图: $(dog_nav_relative_path "$DOG_NAV_PLAN_META")"
  echo "现在可运行: bash system/scripts/start_nav_standalone.sh"
}

cleanup() {
  if [ -s "$session_pcd" ] && [ -s "$session_tum" ]; then
    finish_map || true
  fi
  if [ "$session_published" = 1 ] || [ ! -s "$session_pcd" ]; then
    rm -rf -- "$session_dir"
  else
    echo "本次地图未写入导航目录，文件留在 $session_dir" >&2
  fi
}
trap cleanup EXIT
# Ctrl+C must stop roslaunch without aborting this script, otherwise the
# EXIT trap deletes the map before it is copied into system/map.
trap 'echo "收到 Ctrl+C，正在保存并更新导航地图..."' INT

echo "启动回环建图：回环开启，融合 /go2/odom，标定仍用 Mid360 LI-Init。"
echo "走完后在建图窗口按 Ctrl+C，等终端打印地图路径后再启动导航。"
launch_status=0
roslaunch fastlio mapping_mid360_loop.launch "$@" \
  "pcd_save_path:=${session_pcd}" \
  "tum_path:=${session_tum}" || launch_status=$?
trap - INT
finish_map
