#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh"

if [ ! -s "$DOG_NAV_TUM_FILE" ]; then
  echo "错误: 找不到 TUM 轨迹: $DOG_NAV_TUM_FILE" >&2
  echo "请先运行 system/scripts/slam/start_slam_odom.sh 建图，或设置 DOG_NAV_TUM_FILE。" >&2
  exit 1
fi

mkdir -p "$(dirname "$DOG_NAV_MAP_PREFIX")"
echo "从 TUM 生成三维 FREE 规划地图..."
echo "输入: $(dog_nav_relative_path "$DOG_NAV_TUM_FILE")"
echo "输出前缀: $(dog_nav_relative_path "$DOG_NAV_MAP_PREFIX")"
if [ -n "$DOG_NAV_ELEVATOR_MODE_LOG" ]; then
  if [ ! -s "$DOG_NAV_ELEVATOR_MODE_LOG" ]; then
    echo "错误: 找不到电梯模式日志: $DOG_NAV_ELEVATOR_MODE_LOG" >&2
    exit 1
  fi
  echo "电梯模式日志: $(dog_nav_relative_path "$DOG_NAV_ELEVATOR_MODE_LOG")"
  echo "乘梯上下行轨迹段将被排除，不生成竖直 FREE 体素。"
  python3 "${DOG_NAV_HOME}/nav/src/map_for_plan/scripts/segmented_tum_to_plan_map.py" \
    --traj "$DOG_NAV_TUM_FILE" \
    --mode-log "$DOG_NAV_ELEVATOR_MODE_LOG" \
    --output-prefix "$DOG_NAV_MAP_PREFIX" \
    --z-offset "${DOG_NAV_TUM_Z_OFFSET:--0.40}" \
    "$@"
else
  python3 "${DOG_NAV_HOME}/nav/src/map_for_plan/scripts/tum_to_plan_map.py" \
    --traj "$DOG_NAV_TUM_FILE" \
    --output-prefix "$DOG_NAV_MAP_PREFIX" \
    --z-offset "${DOG_NAV_TUM_Z_OFFSET:--0.40}" \
    "$@"
fi

if [ ! -s "$DOG_NAV_PLAN_META" ] || [ ! -s "${DOG_NAV_MAP_PREFIX}_free.pcd" ]; then
  echo "错误: 未生成规划地图: $DOG_NAV_PLAN_META" >&2
  exit 1
fi
echo "规划地图已保存: $(dog_nav_relative_path "$DOG_NAV_PLAN_META")"
