#!/usr/bin/env bash
# Run in the locked ROS1 build environment. Put binaries outside the checkout.
set -euo pipefail
base=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
out=$(realpath -m "${1:?Provide an external test output directory}")
case "$out/" in "$base/"*) echo 'Test output must be outside the checkout' >&2; exit 2;; esac
mkdir -p "$out"
for name in terrain_support terrain_terminal terrain_tracking go2_tracking_policy reference_ingress; do
  c++ -std=c++14 -O2 -I/usr/include/eigen3 \
    -I"$base/nav/src/prior_local_planner/src/planner/plan_env/include" \
    -I"$base/nav/src/prior_local_planner/src/planner/plan_manage/include" \
    "$base/system/tests/${name}_test.cpp" -o "$out/${name}_test"
  "$out/${name}_test"
done
