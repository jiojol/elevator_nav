# Isolated original SCAN-Planner

This workspace vendors the five planning packages from
https://github.com/wuyi2121/SCAN-Planner.git at commit
`348e8a590a50a5a6bbab8d8c6dcfd171f009be26` (Apache-2.0).
`UPSTREAM.json` records original source hashes. The simulator is not included.

Most upstream C++ algorithm files are unchanged except the mechanical replacement
of the `scan_planner` C++/ROS message namespace with `scan_upstream_planner`.
Package and library names are isolated from `nav/src/prior_local_planner`.
CMake uses C++17 and disables debug symbols for the target PCL/toolchain.

Build on the ROS Noetic robot, without starting navigation:

```bash
cd /home/unitree/replan_in_global_map
bash system/scripts/build_scan_upstream.sh
```

Default navigation now selects this workspace:

```bash
bash system/scripts/start_nav.sh
# Explicit equivalent:
bash system/scripts/start_nav.sh local_planner:=upstream
```

The current modified planner sources remain in their original directory. Select
that implementation with `local_planner:=modified`. Its existing build verifier
still requires a matching native build; if its manifest is absent or stale,
rebuild it using `bash system/scripts/rebuild_scan.sh` before switching. Never
run both navigation variants simultaneously. The build and navigation locks
serialize compiling against an active session.

The local recovery patch changes the FSM and closed-loop controller. It rejects stair
candidates that double back along the reference, reanchors large tracking errors
to odometry, and treats the constant emergency spline as a zero-command hold.
UPSTREAM.json records the adapted source digests as well as the original pin.

## Inputs and outputs

- Existing FAST-LIO map registration, calibration and FREE-only global route
  generation remain in the main `nav` workspace.
- `reference_path_transform` converts `/plan_map/global_path` body-height output
  to ground-height `/initial_path`; original SCAN adds `body_height` once.
- The adapter rotates the standard body-frame twist to the world-frame
  derivatives expected by upstream, on `/scan_upstream/body_odometry`.
  Pose remains the calibrated body pose. Original pose/localization is untouched.
- `/dog_nav/sensor_pose` already describes the lidar in `map`. Upstream
  `need_extrinsic=false` prevents applying its example sensor extrinsics again.
- World point clouds enter original SCAN's rolling occupancy/raycast mapper.
  Optional explicit `free_endpoint` samples are removed before mapping, because
  upstream does not support that flag. The present localizer publishes hit points.
- Original A*, rebound B-spline optimization, timing and original closed-loop
  controller drive `/planning/scan_upstream_cmd_vel` through the thin adapter
  to `/cmd_vel` and the existing Unitree bridge.

This is the original 3-D obstacle/inflation model. The modified FREE-layer
projection, capsule optimizer, extra local FREE constraints, timed handoff
certificates, approach-speed limiter, reobservation state machine, forward guard
and `scan_command_gate.py` are **not loaded** by the upstream launch. FREE remains
an upstream global-route prior; original local SCAN does not enforce the FREE
map as its local boundary. That architectural difference is intentional here.

Launch adaptation retains the requested two-point radius 0.18 m and separation
0.36 m rather than the upstream example radius 0.25 m. Other upstream optimizer
weights, occupancy logic and emergency collision checks remain upstream defaults.
The original speed profile is restored: forward/backward 0.75 m/s, lateral
0.35 m/s and yaw 1.0 rad/s. The bridge forwards bounded lateral commands
through SDK Move(vx, vy, yaw_rate). The modified profile still resolves vy=0.

## Remaining stops

No added obstacle-distance/comfort limit or repeated geometry check exists in
this adapter. It forwards finite controller commands unchanged while an active
goal owns a newly generated curve and command, pose, cloud and localization
inputs are current. Canceling a goal or losing those inputs sends zero; hardware
SDK StopMove acknowledgement, stale-command protection and physical emergency
stop remain active. A canceled curve cannot resume on a second goal.

Original SCAN itself still stops/rotates when yaw error exceeds its default
0.8 rad, stops when its own collision emergency triggers, and stops at its local
trajectory endpoint. Those are upstream behavior, not previous custom rules.
No guarantee of nonstop navigation or effective 10 Hz is inferred from compiling.

Completion uses measured XY + Z proximity and low translational speed for
0.3 s. The adapter does not perform a separate final-yaw turn, so requested goal
orientation is not guaranteed. Cancel and completion suppress the previous
controller curve even though original upstream ignores an empty `/initial_path`.

## Preserved version

The full local working tree before this integration is retained as Git branch
`archive/local-planner-before-upstream-20261118`, commit
`0ecfbcc4ddd68dbe85178e45dedd9a7f3b39a18a`. It includes then-uncommitted source
and test changes without changing the user's index or checkout. The robot has
its own same-named preservation branch for its pre-integration working tree;
its hash is recorded in `logs/upstream_switch_20261118/preserved_version.json`.
