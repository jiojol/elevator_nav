#!/usr/bin/env bash
set -eo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOG_NAV_HOME="$(cd "$script_dir/../.." && pwd)"
source "$script_dir/env.sh"
dog_nav_require_nav
dog_nav_source_nav
map_set=yuki
planning_only=
planning_live_pose=
mark_elevator=false
start_lio=true
record_nav=
nav_bag=
plan_meta=
launch_options=()
for arg in "$@"; do
  case "$arg" in
    map_set:=*) map_set="${arg#map_set:=}" ;;
    planning_only:=*) planning_only="${arg#planning_only:=}" ;;
    planning_live_pose:=*) planning_live_pose="${arg#planning_live_pose:=}" ;;
    mark_elevator:=true|mark_elevator:=1) mark_elevator=true ;;
    start_lio:=*) start_lio="${arg#start_lio:=}" ;;
    record_nav:=*) record_nav="${arg#record_nav:=}" ;;
    nav_bag:=*) nav_bag="${arg#nav_bag:=}" ;;
    plan_meta:=*) plan_meta="${arg#plan_meta:=}" ;;
  esac
done
if [ "$mark_elevator" = true ]; then
  planning_only=true
  planning_live_pose=false
  launch_options+=(planning_only:=true planning_live_pose:=false)
fi
if [ "$map_set" = test11 ]; then
  planning_only="${planning_only:-true}"
  planning_live_pose="${planning_live_pose:-true}"
fi
planning_live_pose="${planning_live_pose:-false}"
set +u
case "${planning_only,,}" in
  true|1)
    case "${planning_live_pose,,}" in false|0) start_lio=false ;; esac
    ;;
  *)
    upstream_setup="$DOG_NAV_HOME/scan_upstream/devel/setup.bash"
    if [ ! -f "$upstream_setup" ]; then
      echo "找不到电梯项目内的 SCAN 构建产物：$upstream_setup" >&2
      exit 1
    fi
    source "$upstream_setup" --extend
    if [ -z "$record_nav" ]; then
      record_nav=true
      launch_options+=(record_nav:=true)
    fi
    ;;
esac
case "$start_lio" in
  false|0) ;;
  *)
    lio_setup="${ELEVATOR_LIO_SETUP:-$HOME/ws_elevator_lio_ros1/devel/setup.bash}"
    if [ ! -f "$lio_setup" ]; then
      echo "找不到 Elevator-LIO 构建产物：$lio_setup" >&2
      exit 1
    fi
    source "$lio_setup" --extend
    ;;
esac
case "${record_nav,,}" in
  true|1)
    if [ -z "$nav_bag" ]; then
      stamp="$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)"
      bag_label="$map_set"
      if [ -n "$plan_meta" ]; then
        bag_label="$(basename -- "$(dirname -- "$plan_meta")")"
      fi
      nav_bag="$HOME/bags/elevator_nav_${bag_label}_${stamp}_$$.bag"
      launch_options+=(nav_bag:="$nav_bag")
    fi
    mkdir -p "$(dirname "$nav_bag")"
    echo "导航 bag 前缀：${nav_bag%.bag}（按 Ctrl+C 结束并保存）"
    ;;
esac
exec roslaunch dog_nav_bringup elevator_navigation.launch "$@" "${launch_options[@]}"
