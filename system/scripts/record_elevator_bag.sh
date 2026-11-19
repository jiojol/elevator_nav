#!/usr/bin/env bash
set -eo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOG_NAV_HOME="$(cd "$script_dir/../.." && pwd)"
source "$script_dir/env.sh"
dog_nav_require_nav
dog_nav_source_nav
mkdir -p "$HOME/bags"
stamp="$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)"
output="$HOME/bags/go2_mid360_${stamp}_$$.bag"
echo "录包路径：$output。按 Ctrl+C 结束录制。"
exec roslaunch dog_nav_bringup elevator_recording.launch output:="$output" "$@"
