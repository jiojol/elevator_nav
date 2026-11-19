# Shared paths for slam / planning-map / navigation scripts.
# Usage: source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

_DOG_NAV_SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -n "${DOG_NAV_HOME:-}" ]; then
  DOG_NAV_HOME="$(cd "${DOG_NAV_HOME}" && pwd)"
else
  DOG_NAV_HOME="$(cd "${_DOG_NAV_SCRIPTS_DIR}/../.." && pwd)"
fi

_dog_nav_resolve_path() {
  case "$1" in
    /*) printf '%s\n' "$1" ;;
    *) printf '%s/%s\n' "$DOG_NAV_HOME" "$1" ;;
  esac
}

dog_nav_relative_path() {
  realpath -m --relative-to="$DOG_NAV_HOME" "$1" 2>/dev/null || printf '%s\n' "$1"
}

ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
SLAM_SETUP="${DOG_NAV_HOME}/slam/devel/setup.bash"
NAV_SETUP="${DOG_NAV_HOME}/nav/devel/setup.bash"

DOG_NAV_MAP_DIR="$(_dog_nav_resolve_path "${DOG_NAV_MAP_DIR:-system/map}")"
DOG_NAV_BUILDING_PCD="$(_dog_nav_resolve_path "${DOG_NAV_BUILDING_PCD:-${DOG_NAV_MAP_DIR}/dog.pcd}")"
DOG_NAV_MAP_PREFIX="$(_dog_nav_resolve_path "${DOG_NAV_MAP_PREFIX:-${DOG_NAV_MAP_DIR}/dog}")"
DOG_NAV_TUM_FILE="$(_dog_nav_resolve_path "${DOG_NAV_TUM_FILE:-${DOG_NAV_MAP_PREFIX}.tum}")"
DOG_NAV_PLAN_META="$(_dog_nav_resolve_path "${DOG_NAV_PLAN_META:-${DOG_NAV_MAP_PREFIX}_plan.json}")"
if [ -n "${DOG_NAV_ELEVATOR_MODE_LOG:-}" ]; then
  DOG_NAV_ELEVATOR_MODE_LOG="$(_dog_nav_resolve_path "$DOG_NAV_ELEVATOR_MODE_LOG")"
else
  DOG_NAV_ELEVATOR_MODE_LOG=""
fi

_dog_nav_prepend_ld() {
  local dir="$1"
  if [ -d "$dir" ]; then
    case ":${LD_LIBRARY_PATH:-}:" in
      *":${dir}:"*) ;;
      *) export LD_LIBRARY_PATH="${dir}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" ;;
    esac
  fi
}

# Loop mapping uses the retained native GTSAM dependency.
_dog_nav_prepend_ld "${DOG_NAV_HOME}/third_party/gtsam/lib"
if [ -n "${GTSAM_DIR:-}" ]; then
  _dog_nav_prepend_ld "$(realpath -m "${GTSAM_DIR}/../../../lib")"
fi

export DOG_NAV_HOME
export DOG_NAV_MAP_DIR
export DOG_NAV_BUILDING_PCD
export DOG_NAV_MAP_PREFIX
export DOG_NAV_TUM_FILE
export DOG_NAV_PLAN_META
export DOG_NAV_ELEVATOR_MODE_LOG

dog_nav_source_file() {
  local file="$1"
  if [ ! -f "$file" ]; then
    echo "错误: 找不到环境文件: $file" >&2
    return 1
  fi
  set +u
  # shellcheck disable=SC1090
  source "$file"
  set -u
}

dog_nav_source_ros() {
  dog_nav_source_file "$ROS_SETUP"
}

dog_nav_source_slam() {
  dog_nav_source_ros
  if [ -f "$SLAM_SETUP" ]; then
    dog_nav_source_file "$SLAM_SETUP"
  fi
  # Archived workspaces can retain a ROS_PACKAGE_PATH from their old location.
  export ROS_PACKAGE_PATH="${DOG_NAV_HOME}/slam/src${ROS_PACKAGE_PATH:+:${ROS_PACKAGE_PATH}}"
}

dog_nav_source_nav() {
  dog_nav_source_ros
  if [ -f "$NAV_SETUP" ]; then
    dog_nav_source_file "$NAV_SETUP"
  fi
}

dog_nav_require_ros() {
  if [ ! -f "$ROS_SETUP" ]; then
    echo "错误: 找不到 ROS 环境: $ROS_SETUP" >&2
    return 1
  fi
}

dog_nav_require_nav() {
  dog_nav_require_ros || return 1
  if [ ! -f "$NAV_SETUP" ]; then
    echo "错误: 找不到导航工作空间: $NAV_SETUP" >&2
    echo "请先执行: cd ${DOG_NAV_HOME}/nav && catkin_make" >&2
    return 1
  fi
}

dog_nav_require_slam() {
  dog_nav_require_ros || return 1
  if [ ! -f "$SLAM_SETUP" ]; then
    echo "错误: 找不到建图工作空间: $SLAM_SETUP" >&2
    echo "请先执行: cd ${DOG_NAV_HOME}/slam && catkin_make" >&2
    return 1
  fi
}
