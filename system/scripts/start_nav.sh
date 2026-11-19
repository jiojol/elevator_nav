#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/navigation_env.sh"

NAV_LAUNCH_PACKAGE=dog_nav_bringup
NAV_LAUNCH_FILE=navigation.launch
PLANNER_BACKEND="${DOG_NAV_PLANNER_BACKEND:-free}"
LOCALIZATION_BACKEND="${DOG_NAV_LOCALIZATION_BACKEND:-fastlio}"
USE_CALIBRATED_MOUNT=false
LOOP_NAV=false
REQUESTED_LINEAR=0.80
REQUESTED_YAW=0.80
LAUNCH_ARGS=()
for arg in "$@"; do
  case "$arg" in
    scan_max_vel:=*) REQUESTED_LINEAR="${arg#scan_max_vel:=}"; continue ;;
    scan_max_yaw:=*) REQUESTED_YAW="${arg#scan_max_yaw:=}"; continue ;;
    planner_backend:=*) PLANNER_BACKEND="${arg#planner_backend:=}"; continue ;;
    localization_backend:=*) LOCALIZATION_BACKEND="${arg#localization_backend:=}" ;;
    use_calibrated_mount:=true|use_calibrated_mount:=1) USE_CALIBRATED_MOUNT=true ;;
    use_calibrated_mount:=false|use_calibrated_mount:=0) USE_CALIBRATED_MOUNT=false ;;
    loop_nav:=true|loop_nav:=1) LOOP_NAV=true ;;
    loop_nav:=false|loop_nav:=0) LOOP_NAV=false ;;
    start_goal_queue:=true|start_goal_queue:=1)
      echo '错误: 当前多点队列停用；起终点往返请使用 loop_nav:=true。' >&2
      exit 1 ;;
    plan_meta:=*)
      DOG_NAV_PLAN_META="$(_dog_nav_resolve_path "${arg#plan_meta:=}")"
      # Forward only the canonical path below. ROS nodes start in ~/.ros,
      # where an overriding relative path cannot find the repository map.
      continue ;;
  esac
  LAUNCH_ARGS+=("$arg")
done
set -- "${LAUNCH_ARGS[@]}"
# Accept the old backend name as an alias; navigation never loads its trajectory.
[ "$PLANNER_BACKEND" != tum ] || PLANNER_BACKEND=free
if [ "$PLANNER_BACKEND" != free ] || [ "$LOCALIZATION_BACKEND" != fastlio ]; then
  echo '错误: 当前版本仅保留 planner_backend:=free、localization_backend:=fastlio。' >&2
  exit 1
fi
LIDAR_INPUT_TOPIC=/livox/lidar

dog_nav_require_nav
if [ ! -f "$DOG_NAV_BUILDING_PCD" ]; then
  echo "错误: 找不到定位点云: $DOG_NAV_BUILDING_PCD" >&2
  exit 1
fi
if [ ! -s "$DOG_NAV_PLAN_META" ]; then
  echo "错误: 找不到规划地图元数据: $DOG_NAV_PLAN_META" >&2
  echo "请先建图，或运行: bash ${SCRIPT_DIR}/build_planning_map.sh" >&2
  exit 1
fi
if ! python3 - "$DOG_NAV_PLAN_META" "${DOG_NAV_HOME}/nav/src/map_for_plan/scripts" <<'PY'
import sys
from pathlib import Path
sys.path.insert(0, sys.argv[2])
from plan_io import load_plan_meta
meta = load_plan_meta(Path(sys.argv[1]))
if not Path(meta["free_pcd"]).is_file():
    raise SystemExit("规划地图缺少 free_pcd: %s" % meta["free_pcd"])
PY
then
  exit 1
fi

dog_nav_source_nav

# Starting a second copy registers the same planner and /rviz names,
# forcing ROS to tear down the first process while OGRE is still rendering.
# On some NVIDIA drivers that race ends in an RViz SIGSEGV.
ACTIVE_ROS_NODES="$(rosnode list 2>/dev/null || true)"
nav_node_alive() {
  timeout 3 rosnode ping -c 1 "$1" 2>/dev/null | grep -q 'xmlrpc reply'
}
if printf '%s\n' "$ACTIVE_ROS_NODES" | grep -Fxq '/global_planner' && \
   nav_node_alive /global_planner; then
  echo '错误: 导航已在运行，请不要重复执行 start_nav.sh。' >&2
  echo '如需重启，请先在原启动终端按 Ctrl-C。' >&2
  exit 1
fi

PLANNING_ONLY=false
HAS_PLANNING_ONLY_ARG=false
HAS_START_LIDAR_ARG=false
START_LIDAR_DISABLED=false
for arg in "$@"; do
  case "$arg" in
    start_lidar:=false|start_lidar:=0)
      HAS_START_LIDAR_ARG=true
      START_LIDAR_DISABLED=true
      ;;
    start_lidar:=*)
      HAS_START_LIDAR_ARG=true
      ;;
    planning_only:=*)
      HAS_PLANNING_ONLY_ARG=true
      case "$arg" in
        planning_only:=true|planning_only:=1) PLANNING_ONLY=true ;;
        planning_only:=false|planning_only:=0) PLANNING_ONLY=false ;;
      esac
      ;;
  esac
done
if [ "$HAS_PLANNING_ONLY_ARG" = false ] && [ "$START_LIDAR_DISABLED" = true ]; then
  PLANNING_ONLY=true
  set -- "$@" 'planning_only:=true'
fi
if [ "$LOOP_NAV" = true ] && [ "$PLANNING_ONLY" = true ]; then
  echo '错误: loop_nav 需要完整导航，不能与 planning_only 一起使用。' >&2
  exit 1
fi

# The web/HDL stack owns the same lidar, TF and chassis command topic.
# It must be stopped by the operator; never silently kill or reconfigure it.
if [ "$PLANNING_ONLY" = false ]; then
  if command -v systemctl >/dev/null 2>&1 && \
     systemctl is-active --quiet modern-visual-nav.service; then
    echo '错误: 旧网页导航 modern-visual-nav.service 正在运行，占用雷达和底盘控制。' >&2
    echo '旧导航使用 PointCloud2；当前 FAST-LIO 需要 Livox CustomMsg，不能直接复用。' >&2
    echo '请先执行：sudo systemctl stop modern-visual-nav.service' >&2
    echo '再重新执行本启动命令；新导航会自行以 xfer_format:=1 启动雷达。' >&2
    exit 1
  fi
  if printf '%s\n' "$ACTIVE_ROS_NODES" | grep -Fxq '/move_base' && nav_node_alive /move_base; then
    echo '错误: 旧 move_base 导航仍在运行，请先在原终端关闭它，再启动 FAST-LIO + SCAN。' >&2
    exit 1
  fi
fi

# Plan with speeds the live bridge can actually execute. Keep the launch
# arguments and the recorded parameter snapshot consistent with that limit.
SPEED_ARGS="$(python3 "${SCRIPT_DIR}/resolve_control_limits.py" \
  "$REQUESTED_LINEAR" "$REQUESTED_YAW" "$PLANNING_ONLY")"
mapfile -t SPEED_LAUNCH_ARGS <<< "$SPEED_ARGS"
set -- "$@" "${SPEED_LAUNCH_ARGS[@]}"

if [ "$PLANNING_ONLY" = false ]; then
  python3 "${SCRIPT_DIR}/verify_scan_build.py"
  if printf '%s\n' "$ACTIVE_ROS_NODES" | grep -Eq '^/(laserMapping|fastlio_nav/laserMapping|fastlio_map_localizer)$'; then
    echo '错误: 建图或定位已在运行，请先在原启动终端按 Ctrl-C，避免两个定位实例同时工作。' >&2
    exit 1
  fi
  if [ "$LOCALIZATION_BACKEND" = fastlio ]; then
    FASTLIO_BINARY="${DOG_NAV_HOME}/slam/devel/lib/fast_lio/fastlio_mapping"
    LOCALIZER_BINARY="${DOG_NAV_HOME}/nav/devel/lib/dog_nav_bringup/fastlio_map_localizer"
    for required_binary in "$FASTLIO_BINARY" "$LOCALIZER_BINARY"; do
      if [ ! -x "$required_binary" ]; then
        echo "错误: 缺少 FAST-LIO 导航程序: $required_binary" >&2
        exit 1
      fi
      missing_libs="$(LD_LIBRARY_PATH="${DOG_NAV_HOME}/slam/devel/lib:${LD_LIBRARY_PATH:-}" ldd "$required_binary" 2>/dev/null | grep 'not found' || true)"
      if [ -n "$missing_libs" ]; then
        echo "错误: $required_binary 缺少运行库，请在本机重新编译：" >&2
        echo "$missing_libs" >&2
        exit 1
      fi
    done
  fi
fi

REUSE_LIDAR=false
if [ "$PLANNING_ONLY" = false ] && [ "$USE_CALIBRATED_MOUNT" = true ]; then
  if [ "$START_LIDAR_DISABLED" = true ] || \
     nav_node_alive /livox_lidar_publisher2; then
    echo '错误: 试用新安装外参时，不能复用旧雷达进程及其 TF。' >&2
    echo '请先关闭独立雷达驱动，再用 start_nav.sh use_calibrated_mount:=true 启动。' >&2
    exit 1
  fi
fi
if [ "$PLANNING_ONLY" = false ] && [ "$HAS_START_LIDAR_ARG" = false ] && \
   nav_node_alive /livox_lidar_publisher2; then
  set -- "$@" 'start_lidar:=false' 'planning_only:=false'
  REUSE_LIDAR=true
fi
if [ "$PLANNING_ONLY" = false ] && [ "$START_LIDAR_DISABLED" = true ]; then
  REUSE_LIDAR=true
fi
if [ "$PLANNING_ONLY" = false ] && [ "$HAS_START_LIDAR_ARG" = true ] && \
   [ "$START_LIDAR_DISABLED" = false ] && \
   nav_node_alive /livox_lidar_publisher2; then
  echo '错误: Livox 驱动已在运行，请使用默认启动命令复用现有驱动。' >&2
  exit 1
fi
if [ "$REUSE_LIDAR" = true ]; then
  PUBLISHED_TOPICS="$(rostopic list -p 2>/dev/null || true)"
  if ! printf '%s\n' "$PUBLISHED_TOPICS" | grep -Fxq "$LIDAR_INPUT_TOPIC" || \
     ! printf '%s\n' "$PUBLISHED_TOPICS" | grep -Fxq '/livox/imu'; then
    echo "错误: ${LOCALIZATION_BACKEND} 定位需要 ${LIDAR_INPUT_TOPIC} 和 /livox/imu 两路实时话题。" >&2
    exit 1
  fi
  CURRENT_LIDAR_TYPE="$(rostopic type "$LIDAR_INPUT_TOPIC" 2>/dev/null || true)"
  if [ "$LOCALIZATION_BACKEND" = fastlio ] && [ "$CURRENT_LIDAR_TYPE" != livox_ros_driver2/CustomMsg ]; then
    echo "错误: 已运行的 ${LIDAR_INPUT_TOPIC} 类型为 ${CURRENT_LIDAR_TYPE:-UNKNOWN}，不能供 FAST-LIO 使用。" >&2
    echo '需要 livox_ros_driver2/CustomMsg；请先关闭该雷达的原启动进程，再重新执行本命令。' >&2
    echo '当前导航入口会自行以 xfer_format:=1 启动雷达；仅修改运行中的参数不会切换消息格式。' >&2
    exit 1
  fi
  echo "复用已运行且消息格式正确的 Livox 驱动，定位输入: ${LIDAR_INPUT_TOPIC} + /livox/imu。"
fi

HAS_RVIZ_ARG=false
for arg in "$@"; do
  case "$arg" in
    start_rviz:=*) HAS_RVIZ_ARG=true ;;
  esac
done
if [ "$HAS_RVIZ_ARG" = false ] && [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
  set -- "$@" "start_rviz:=false"
  echo "当前终端没有图形显示，导航继续运行。请在可连接同一 ROS master 的图形桌面打开："
  echo "rviz -d ${DOG_NAV_HOME}/nav/src/dog_nav_bringup/config/navigation_free.rviz"
fi

if [ "$PLANNING_ONLY" = false ] && [ "$REUSE_LIDAR" = false ]; then
  LIVOX_DRIVER_NODE="$(catkin_find --libexec livox_ros_driver2 livox_ros_driver2_node 2>/dev/null | head -n 1 || true)"
  if [ -z "$LIVOX_DRIVER_NODE" ] || [ ! -x "$LIVOX_DRIVER_NODE" ]; then
    echo "错误: 完整导航需要 livox_ros_driver2_node，但 nav 工作空间尚未构建该可执行文件。" >&2
    echo "请先安装 Livox-SDK2，再执行: cd nav && catkin_make -DCATKIN_WHITELIST_PACKAGES= -j2" >&2
    echo "只验证规划可使用 start_lidar:=false。" >&2
    exit 1
  fi

  LIVOX_HOST_IP="${DOG_NAV_LIVOX_HOST_IP:-192.168.123.18}"
  if command -v ip >/dev/null 2>&1; then
    HOST_IPV4="$(ip -4 -o addr show 2>/dev/null || true)"
    case "$HOST_IPV4" in
      *"${LIVOX_HOST_IP}/"*) ;;
      *)
        echo "警告: 本机未配置 Mid360 主机地址 ${LIVOX_HOST_IP}，雷达可能无数据。" >&2
        ;;
    esac
  fi
fi

if [ "$PLANNING_ONLY" = true ]; then
  echo "启动 ${PLANNER_BACKEND} 规划（无雷达，不跑定位和 SCAN）..."
else
  echo "启动 ${LOCALIZATION_BACKEND} 定位 + ${PLANNER_BACKEND} + SCAN 三维导航..."
  echo '在先验 FREE 区域内规划，实时障碍触发局部重规划；执行端完成到点朝向。取消导航使用 /map_manager/cancel。'
fi
echo "仓库: ."
echo "定位 PCD: $(dog_nav_relative_path "$DOG_NAV_BUILDING_PCD")"
echo "1) RViz 用 3D Pose Estimate 点起点"
if [ "$PLANNING_ONLY" = false ] && [ "$LOCALIZATION_BACKEND" = fastlio ]; then
  echo '   初始化时保持静止；终端显示 FAST-LIO localization: TRACKING 后再给目标。'
  echo '   状态查看: rostopic echo /dog_nav/localization_status'
fi
echo "规划地图: $(dog_nav_relative_path "$DOG_NAV_PLAN_META")"
echo "2) 再用 3D Nav Goal 点终点，路径在 /plan_map/global_path"

exec python3 "${SCRIPT_DIR}/navigation_session.py" --workspace "${DOG_NAV_HOME}" -- \
  roslaunch "${NAV_LAUNCH_PACKAGE}" "${NAV_LAUNCH_FILE}" \
  "globalmap_pcd:=${DOG_NAV_BUILDING_PCD}" \
  "planner_backend:=${PLANNER_BACKEND}" \
  "localization_backend:=${LOCALIZATION_BACKEND}" \
  "plan_meta:=${DOG_NAV_PLAN_META}" \
  "$@"
