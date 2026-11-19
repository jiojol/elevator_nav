#!/bin/bash
# Operator entry point for the standalone FAST-LIO/SCAN session.
# This file only changes services when the operator explicitly runs it.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/navigation_env.sh"
dog_nav_require_nav
dog_nav_source_nav

PLANNING_ONLY=false
EXPLICIT_PLANNING=false
LIDAR_DISABLED=false
for arg in "$@"; do
  case "$arg" in
    planning_only:=true|planning_only:=1) PLANNING_ONLY=true; EXPLICIT_PLANNING=true ;;
    planning_only:=false|planning_only:=0) PLANNING_ONLY=false; EXPLICIT_PLANNING=true ;;
    start_lidar:=false|start_lidar:=0) LIDAR_DISABLED=true ;;
  esac
done
if [ "$PLANNING_ONLY" = true ] || { [ "$LIDAR_DISABLED" = true ] && [ "$EXPLICIT_PLANNING" = false ]; }; then
  exec bash "${SCRIPT_DIR}/start_nav.sh" "$@"
fi

# Check the candidate's files before interrupting the existing web stack.
for required in "$DOG_NAV_BUILDING_PCD" "$DOG_NAV_PLAN_META" \
  "${DOG_NAV_HOME}/nav/devel/lib/dog_nav_bringup/fastlio_map_localizer" \
  "${DOG_NAV_HOME}/slam/devel/lib/fast_lio/fastlio_mapping"; do
  if [ ! -s "$required" ]; then
    echo "错误: 缺少导航文件，未切换服务：$required" >&2
    exit 1
  fi
done
python3 "${SCRIPT_DIR}/verify_scan_build.py"
if timeout 3 rosnode ping -c 1 /global_planner 2>/dev/null | grep -q 'xmlrpc reply'; then
  echo '错误: 独立导航已在运行，请先在原启动终端按 Ctrl-C。' >&2
  exit 1
fi
mkdir -p "${DOG_NAV_HOME}/.runtime"
exec 9>"${DOG_NAV_HOME}/.runtime/standalone_navigation.lock"
if ! flock -n 9; then
  echo '错误: 已有独立导航会话在启动或运行。' >&2
  exit 1
fi

# A runtime drop-in works even for custom unit files under /etc/systemd/system.
# The condition is inactive when this user-owned session marker is absent.
MARKER="${DOG_NAV_HOME}/.runtime/standalone_navigation.active"
DROPIN_DIR=/run/systemd/system/modern-visual-nav.service.d
DROPIN="${DROPIN_DIR}/90-dog-nav-standalone.conf"
NAV_PID=""
cleanup() {
  # Keep ownership until the recorder/navigation supervisor has shut down.
  trap '' INT TERM
  if [ -n "$NAV_PID" ] && kill -0 "$NAV_PID" 2>/dev/null; then
    kill -TERM -- "-$NAV_PID" 2>/dev/null || true
    wait "$NAV_PID" || true
  fi
  rm -f -- "$MARKER"
  echo '独立导航已退出；已解除本轮对旧网页导航的启动限制。'
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
sudo -v
printf '%s\n' "$$" > "$MARKER"
sudo install -d "$DROPIN_DIR"
printf '[Unit]\nConditionPathExists=!%s\n' "$MARKER" | sudo tee "$DROPIN" >/dev/null
sudo systemctl daemon-reload
printf '%s\n' '切换到独立导航：停止旧网页导航及其雷达。'
sudo systemctl stop modern-visual-nav.service
printf '%s\n' '重新加载底盘桥配置，等待新限速生效。'
sudo systemctl restart modern-visual-state.service
python3 "${SCRIPT_DIR}/wait_robot_bridge.py"
# Do not let navigation children retain the wrapper's ownership lock.
# No job control: setsid's child PID is also the session/process-group ID.
set +m
setsid bash "${SCRIPT_DIR}/start_nav.sh" "$@" 9>&- &
NAV_PID=$!
set +e
wait "$NAV_PID"
NAV_EXIT=$?
set -e
exit "$NAV_EXIT"
