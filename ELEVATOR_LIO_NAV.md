# Elevator-LIO multi-floor navigation

This branch adds a door-aware, single-elevator navigation flow to the existing
ROS 1 FREE/SCAN stack. The implementation keeps the final goal out of the
ordinary global A* unless the start and goal are on the same floor.

## Source checkout

The navigation workspace uses ROS 1 Noetic. Build its local navigation and
SCAN packages with `bash system/scripts/build_current.sh nav` and
`bash system/scripts/rebuild_scan.sh` when they are not built yet.
Use an existing Elevator-LIO catkin workspace or build the bundled
`elevator_lio_ros1_source/lio` package in a separate catkin workspace and point
`ELEVATOR_LIO_SETUP` to its `devel/setup.bash`.

The source checkout omits site scans and recordings. For `map_set:=test11`,
provide the localization and FREE maps and their associated trajectory files
under `system/map/test11/`, or regenerate them as described below.

## Data flow

```text
Livox MID-360
  -> Elevator-LIO ROS 1
       /LIO/odom_imu
       /LIO/clouds_lidar/effect
       /LIO/in_elevator

  -> elevator_lio_adapter
       /elevator_lio/odometry
       /elevator_lio/cloud_body
       /elevator_lio/in_elevator

  -> fastlio_map_localizer (generic LIO map registration)
       /dog_nav/odometry
       /navigation/obstacle_points
       /dog_nav/localization_status
       map -> lio_odom -> base_link

  -> reference_path_transform / SCAN
```

During `in_elevator=true`, map registration corrections are frozen while
Elevator-LIO odometry continues to be published. The test11 navigation launch additionally checks the elevator's acceleration,
braking and stationary period using a separately bias-corrected IMU estimate.
One bounded vertical step (up to 1.0 m)
consistent with the Elevator-LIO exit ZUPT is allowed during the ride without
using that frame as a velocity sample. On the next normal-mode message, the
localizer reacquires a submap and waits for two consecutive scan matches to
agree before reporting `TRACKING` again. Post-elevator reacquisition can
correct up to 3.0 m. Normal tracking uses the 0.35 m limit.

## FREE map generation

Use the segmented builder when a mode log is available:

```bash
export DOG_NAV_ELEVATOR_MODE_LOG=/path/to/elevator_debug.csv
bash system/scripts/build_planning_map.sh
```

The CSV must have a timestamp column and `in_elevator`/`in_ele` flag column.
Elevator intervals are removed before per-segment resampling and inflation. The
normal segments are never reconnected, so the FREE map cannot contain a
vertical elevator shaft corridor.

The full localization PCD remains multi-floor. Only the planning graph removes
the transit geometry.


## Door waiting behavior

The first cross-floor mission leg stops at `lobby_pose`. The door monitor projects
live `/navigation/obstacle_points` into the configured door plane and publishes
`/elevator/door_open` only after the aperture remains clear with cabin-side
returns for `open_hold_s`. The mission manager then sends the short same-floor
lobby-to-cabin ingress segment. It never drives into a closed door merely because
the static FREE map contains the cabin.

The ingress segment is intentionally a third execution path, but only the first
and final paths are long-range global missions. No ladder/call-button control is
implemented; the current behavior is wait-only.

## Elevator configuration

Edit `nav/src/dog_nav_bringup/config/elevator.yaml`:

```yaml
elevator:
  id: E1
  configured: true
  floors:
    floor1:
      inside_pose: [x, y, z, yaw]
    floor2:
      inside_pose: [x, y, z, yaw]
```

`inside_pose` is the in-cabin pose in the global `map` frame. Its Z is the floor
support height used by the FREE graph. Cross-floor missions are rejected while
`configured` is false.

## Mission flow

```text
same floor:
  final goal -> /plan_map/segment_goal

different floors:
  current floor -> lobby_pose -> /plan_map/segment_goal
  wait for /elevator/door_open=true
  lobby_pose -> inside_pose -> /plan_map/segment_goal (short ingress)
  Elevator-LIO estimates vertical displacement during transit
  wait for destination cabin height and TRACKING localization
  wait for confirmed open-door point cloud from the cabin side
  -> destination-floor final goal -> /plan_map/segment_goal
```

The mission manager subscribes to the normal RViz 3D Nav Goal
(`/move_base_simple/goal`) and also exposes `/elevator_nav/goal`.
Status and result topics:

- `/elevator_nav/status`
- `/elevator_nav/result`
- `/elevator_nav/cancel`

The global planner runs with `mission_segment_only=true` in the elevator launch.
It rejects any segment whose target floor differs from the measured current
floor. An elevator goal up to 0.50 m above or below the FREE support in the
same XY cell and floor can be projected onto that support.

## Map elevator points

The start and goal floors are inferred from the map-frame Z of the body
(relative to ground) and the selected goal. Supply the map's support heights
with `floors_config` and mark the elevator using `lobby_pose` and `inside_pose`
in `elevator_config`. RViz displays the marked lobby and cabin points on
`/elevator_nav/markers`.

Selecting `system/map/yuki/dog_plan.json` automatically loads the yuki floor
and elevator poses. The marked ride near y=-5.3 serves the -33.3 m and -11.1 m
support levels. Its poses are trajectory estimates and the door plane still
needs verification on site.

## Launch

From this project root:

```bash
bash system/scripts/start_elevator_nav.sh
```

The default launches the yuki map, MID-360, Elevator-LIO, RViz, the local
SCAN stack and the Unitree chassis bridge together. In yuki, the bridge and SCAN share default limits of 0.20 m/s forward,
0.10 m/s lateral and 0.30 rad/s yaw.
Use the 3D Pose Estimate tool and wait for `TRACKING` before sending a goal.
For an existing component, use `start_lidar:=false`, `start_lio:=false`
or `start_bridge:=false` as applicable.

The local planner, closed-loop controller and ROS I/O adapter are a local
copy of the default upstream SCAN backend from `/home/unitree/replan_in_global_map`.
The copied source and upstream revision record are under `scan_upstream/`.
To rebuild that workspace with ROS Noetic:

```bash
source /opt/ros/noetic/setup.bash
catkin_make -C scan_upstream -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 \
  -DCMAKE_CXX_COMPILER="$PWD/nav/cmake_overlay/cxx17-g++.sh" -j4 -l5
```

## Global planning only

```bash
bash system/scripts/start_elevator_nav.sh planning_only:=true
```

Select a start on the FREE map with RViz 3D Pose Estimate and then select
a 3D Nav Goal. For a cross-floor goal, RViz shows the route to the departure
lobby in yellow, the door-gated ingress from lobby to cabin in orange, and
the route from the destination cabin to the green final goal in magenta. The destination preview starts
at the target-floor cabin position with the departure cabin's goal yaw.
During real navigation the final route uses the newly localized pose and heading.
A
person opens the door, and the robot waits in the lobby until the point
cloud monitor confirms the aperture is clear. If a click hits a wall or overhead
return, the preview can snap it to reachable ground within 0.75 m horizontally
and 2.50 m vertically on the selected floor. The marker shows the snapped
goal. An unreachable click is shown in red. This mode does not require live localization.
Add `planning_live_pose:=true` to use the real tracked robot pose instead.
To exercise the complete elevator sequence without moving a robot, use the
planning smoke test below with an isolated ROS master.

## Current assumptions

- One configured elevator (`E1`).
- Automatic elevator mode follows Elevator-LIO's existing semantics.
- `in_elevator=false` after transit indicates a stop. The mission independently
  checks destination height and live door evidence before planning the final leg.
- Calling/waiting for an elevator is not implemented yet.
- Initial localization still requires a manual 3D Pose Estimate.

## Validation on 2026-11-18

- Elevator-LIO ROS 1 node built successfully under Noetic in the
  `robot-lab-ros1:baseline-001` container.
- The full `go2_mid360_test11.bag` (154 s) was replayed through the ROS 1
  Elevator-LIO adapter. The adapter produced 1,541 synchronized odometry/body
  cloud pairs and reproduced both elevator transitions.
- The actual `test11` trajectory/mode log generated three normal segments. The
  segmented FREE map contained no points in the elevator vertical corridor
  between the floor support levels.
- The navigation package and launch graph build/parse successfully in Noetic.
- GICP startup was verified against the generated global map (initial candidate
  100% overlap, 0.076 m RMSE). Destination-floor reacquisition still needs a
  full closed-loop hardware/target-map test.

## test11 map profile

```bash
bash system/scripts/start_elevator_nav.sh map_set:=test11
```

This profile loads the reconstruction, segmented FREE map and elevator
coordinates for `go2_mid360_test11.bag`. It runs the MID-360 and Elevator-LIO
for live localization while keeping navigation in global-planning mode.
In RViz, set the robot's actual position and heading in the mapped space
using 3D Pose Estimate, then wait for `TRACKING` before sending a goal.
Add `planning_live_pose:=false` for a manual-start preview without sensors.
For that preview, the example start is near (0, 0, -0.40) and the example
destination is near (8.877, -4.334, 6.85), in the map frame.

## Chassis navigation and standalone recording

To enable the local planner and chassis bridge with the test11 map in the
recorded environment:

```bash
bash system/scripts/start_elevator_nav.sh map_set:=test11 planning_only:=false
```

The test11 corridor limits match the source SCAN launch: 0.75 m/s forward,
0.35 m/s lateral and 1.00 rad/s yaw. Within 1.60 m of a same-floor cabin the
adapter caps planar speed, lateral speed and yaw rate at 0.20 m/s, 0.10 m/s
and 0.30 rad/s respectively. An authorized ingress route uses the configured
corridor forward speed as its planar cap (0.75 m/s for test11), with the same
near-door lateral and yaw limits. Within 0.45 m of the marked lobby, with
fresh tracking, cloud and an active SCAN trajectory, the robot stops translating
and turns to the published lobby yaw. Test11 uses 0.45–0.55 rad/s for this
stationary turn, within the configured chassis yaw limit. It acknowledges
arrival after settling within 0.10 rad. The test11 lobby points
are set back about 1.39 m and 1.56 m from their configured door planes.
Arrival still requires the body center to be at least 0.80 m outside the
plane. These are map estimates and require on-site clearance verification.
Ingress requires fresh door-open evidence from the lobby side. Test11 also
checks the start of each SCAN ingress spline against the door-facing heading.
A candidate that doubles back is discarded and SCAN can try a shorter
collision-checked trajectory. The robot
holds still during the ride. In test11, it first tries to align to the
exit-facing pose after reaching the cabin parking point. If the lift moves
first, alignment resumes after arrival and reacquisition. Both alignments
require fresh tracking and odometry, cabin-side door-open evidence and
a clear footprint sweep. At the destination, the exit is planned after
sustained door confirmation and verified alignment.
From inside the cabin, the exit reference passes through the door center and a point just
outside it. Test11 starts with a 0.35 m/s exit speed limit and ramps to the
corridor limit as the body clears 0.65 m beyond the door plane. Both points must have same-floor FREE support. SCAN still validates
the route against live obstacles. If the exit stalls after the body has moved
at least 0.25 m, the adapter can request a bounded replan from fresh odometry.

Navigation uses a prebuilt map and disables Elevator-LIO's incremental
mapping PCD writes by default. Outside elevator transitions, navigation
discards old LiDAR work when newer frames are already queued over 0.15 s ahead.
The adaptive LiDAR point budget is capped at the nominal radar frame rate
so a skipped scan cannot increase the processing cost of the next scan.
The mapping workflow retains its own settings. Full navigation automatically
records a split ROS bag in
`~/bags`, including
the LiDAR/IMU, global and local paths, SCAN status, `/cmd_vel` and chassis
feedback. Ctrl-C finalizes the current bag file.

To record synchronized MID-360 inputs without running navigation:

```bash
bash system/scripts/record_elevator_bag.sh
```

Ctrl-C finalizes the bag in `~/bags`. Use `start_lidar:=false` to reuse an
already running driver, or `duration:=10` for a timed recording.

## test11 multi-stage planning smoke test

Local test assets are under:

`/home/yuki/elevator_lio_data/test11_nav_map/`

The test uses `test11` Elevator-LIO output and the derived segmented map:

- Lower-floor elevator candidate: `[4.728, -5.709, -0.40]`
- Upper-floor elevator candidate: `[4.533, -5.478, 6.85]`
- Upper-floor test goal: approximately `[8.877, -4.334, 6.85]`

Results:

- Direct lower-to-upper A*: correctly rejected, no path through the shaft.
- Global approach start -> floor-1 lobby: 15.21 m.
- Door-open cabin ingress lobby -> inside: 1.23 m.
- Destination-floor final path: 4.83 m.
- ROS topic integration generated all three execution segments:
  - Approach path: 158 poses
  - Cabin ingress path: 14 poses
  - Final path: 50 poses

Test-specific configs:

- `nav/src/dog_nav_bringup/config/floors_test11.yaml`
- `nav/src/dog_nav_bringup/config/elevator_test11.yaml`

### One-command planning smoke test

After sourcing a ROS 1 Noetic workspace:

```bash
bash system/tests/run_elevator_planning_smoke.sh \
  plan_meta:=/home/yuki/elevator_lio_data/test11_nav_map/test11_plan.json
```

This starts the global planner and mission manager in planning-only mode, feeds
synthetic start/lobby/inside/destination odometry, door-open state and elevator
mode transitions, and requires three paths to be generated. It does not run SCAN
or a real chassis.
