#!/bin/bash
# Build only the retained hardware mapping/navigation chain.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.sh"
dog_nav_source_ros
MODE="${1:-all}"
case "$MODE" in all|nav|slam) ;; *) echo 'Usage: build_current.sh [all|nav|slam]' >&2; exit 2 ;; esac
JOBS="${DOG_NAV_BUILD_JOBS:-2}"
if ! [[ "$JOBS" =~ ^[1-9][0-9]*$ ]]; then
  echo 'DOG_NAV_BUILD_JOBS must be a positive integer.' >&2
  exit 2
fi
# Do not replace binaries in use by a navigation process.
python3 - "$DOG_NAV_HOME" <<'PY'
import os
from pathlib import Path
import sys
root = Path(sys.argv[1]).resolve()
for process in Path('/proc').glob('[0-9]*'):
    try:
        exe = Path(os.readlink(process / 'exe').split(' (deleted)', 1)[0])
        if any((root / area / 'devel') in exe.parents for area in ('nav', 'slam')):
            raise SystemExit('Stop this workspace\'s ROS nodes before building: ' + str(exe))
    except OSError:
        continue
PY
COMMON=(-DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
        -DCATKIN_ENABLE_TESTING=OFF -DROS_EDITION=ROS1
        "-DCMAKE_CXX_COMPILER=${DOG_NAV_HOME}/nav/cmake_overlay/cxx17-g++.sh")
if [ "$MODE" = all ] || [ "$MODE" = slam ]; then
  (
    cd "${DOG_NAV_HOME}/slam"
    catkin_make "${COMMON[@]}" \
      -DCATKIN_WHITELIST_PACKAGES=livox_ros_driver2\;fast_lio\;fastlio \
      "-DGTSAM_DIR=${GTSAM_DIR:-${DOG_NAV_HOME}/third_party/gtsam/lib/cmake/GTSAM}" \
      -j"$JOBS" -l"$JOBS"
  )
fi
if [ "$MODE" = all ] || [ "$MODE" = nav ]; then
  (
    cd "${DOG_NAV_HOME}/nav"
    catkin_make livox_ros_driver2_generate_messages reference_path_transform fastlio_map_localizer dog_nav_rviz_plugins livox_ros_driver2_node "${COMMON[@]}" \
      -DCATKIN_WHITELIST_PACKAGES=livox_ros_driver2\;fast_gicp\;plan_env\;path_searching\;bspline_opt\;traj_utils\;scan_planner\;dog_nav_bringup\;map_for_plan\;dog_nav_rviz_plugins \
      "-DCMAKE_PREFIX_PATH=${DOG_NAV_HOME}/nav/cmake_overlay;/opt/ros/noetic" \
      -j"$JOBS" -l"$JOBS"
  )
  # Write the source/artifact manifest through the existing clean build path.
  bash "${SCRIPT_DIR}/rebuild_scan.sh"
fi
