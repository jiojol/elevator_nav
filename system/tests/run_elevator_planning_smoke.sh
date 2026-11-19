#!/bin/bash
# One-command planning-only test for the two-stage elevator mission flow.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/system/scripts/env.sh"

PLAN_META="${DOG_NAV_PLAN_META}"
FLOORS="$ROOT/nav/src/dog_nav_bringup/config/floors_test11.yaml"
ELEVATOR="$ROOT/nav/src/dog_nav_bringup/config/elevator_test11.yaml"
MISSION="$ROOT/nav/src/dog_nav_bringup/config/elevator_mission.yaml"
START_XYZ="0,0,0"
LOBBY_XYZ="6.700,-5.709,0"
INSIDE_XYZ="4.728,-5.709,0"
DESTINATION_XYZ="4.533,-5.478,7.25"
GOAL_XYZ="8.877,-4.334,6.85"
MASTER_PID=""
RUN_DIR="$(mktemp -d "${TMPDIR:-/tmp}/elevator_planning_smoke.XXXXXX")"

for arg in "$@"; do
  case "$arg" in
    plan_meta:=*) PLAN_META="${arg#plan_meta:=}" ;;
    floors:=*) FLOORS="${arg#floors:=}" ;;
    elevator:=*) ELEVATOR="${arg#elevator:=}" ;;
    mission:=*) MISSION="${arg#mission:=}" ;;
    start_xyz:=*) START_XYZ="${arg#start_xyz:=}" ;;
    lobby_xyz:=*) LOBBY_XYZ="${arg#lobby_xyz:=}" ;;
    inside_xyz:=*) INSIDE_XYZ="${arg#inside_xyz:=}" ;;
    destination_xyz:=*) DESTINATION_XYZ="${arg#destination_xyz:=}" ;;
    goal_xyz:=*) GOAL_XYZ="${arg#goal_xyz:=}" ;;
    -h|--help)
      echo "用法: $0 plan_meta:=FILE [goal_xyz:=x,y,z] [destination_xyz:=x,y,z]"
      exit 0 ;;
    *) echo "未知参数: $arg" >&2; exit 2 ;;
  esac
done

if [ ! -s "$PLAN_META" ]; then
  echo "错误: 找不到规划元数据: $PLAN_META" >&2
  exit 1
fi
for file in "$FLOORS" "$ELEVATOR" "$MISSION"; do
  [ -s "$file" ] || { echo "错误: 缺少配置: $file" >&2; exit 1; }
done

dog_nav_require_ros
dog_nav_source_nav

if ! rosnode list >/dev/null 2>&1; then
  roscore >"${RUN_DIR}/roscore.log" 2>&1 &
  MASTER_PID=$!
  for _ in $(seq 1 30); do
    rosnode list >/dev/null 2>&1 && break
    sleep 0.2
  done
fi

cleanup() {
  local pid
  for pid in "${DRIVER_PID:-}" "${MISSION_PID:-}" "${PLANNER_PID:-}" "$MASTER_PID"; do
    [ -n "$pid" ] || continue
    kill -INT "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

rosparam load "$FLOORS" /global_planner
rosparam load "$ELEVATOR" /global_planner
rosparam set /global_planner/planning_only true
rosparam set /global_planner/mission_segment_only true
rosparam set /global_planner/body_height 0.40
rosparam set /global_planner/path_z_offset 0.0
python3 "$ROOT/nav/src/map_for_plan/scripts/global_planner.py" \
  --meta "$PLAN_META" --frame-id map --snap-radius 2.0 \
  --max-step 0.8 --center-weight 6.0 \
  >"${RUN_DIR}/planner.log" 2>&1 &
PLANNER_PID=$!

rosparam load "$FLOORS" /elevator_mission_manager
rosparam load "$ELEVATOR" /elevator_mission_manager
rosparam load "$MISSION" /elevator_mission_manager
python3 "$ROOT/nav/src/dog_nav_bringup/scripts/elevator_mission_manager.py" \
  >"${RUN_DIR}/mission.log" 2>&1 &
MISSION_PID=$!

sleep 4
python3 "$SCRIPT_DIR/elevator_planning_smoke_node.py" \
  --start-xyz "$START_XYZ" \
  --lobby-xyz "$LOBBY_XYZ" \
  --inside-xyz "$INSIDE_XYZ" \
  --destination-xyz "$DESTINATION_XYZ" \
  --goal-xyz "$GOAL_XYZ" &
DRIVER_PID=$!
if wait "$DRIVER_PID"; then
  STATUS=0
else
  STATUS=$?
  echo "规划烟测失败，最近日志：" >&2
  tail -80 "${RUN_DIR}/planner.log" >&2 || true
  tail -80 "${RUN_DIR}/mission.log" >&2 || true
fi
exit "$STATUS"
