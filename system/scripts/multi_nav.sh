#!/bin/bash
set -euo pipefail

usage() {
  cat <<'HELP'
Usage: bash system/scripts/multi_nav.sh COMMAND [ARGS]
  add X Y Z YAW_DEG [map|floor1|floor2]  Append a map-frame waypoint.
  save NAME                            Save the current queue as NAME.json.
  load NAME                            Stop the old queue and load NAME.
  start [ROUNDS]                       Run the loaded queue (default: once).
  run NAME [ROUNDS]                    Load and run a saved queue.
  pause                                Cancel the active goal and pause.
  resume                               Send the paused goal again.
  stop                                 Stop the queue and cancel navigation.
  clear                                Stop and empty the in-memory queue.
  list                                 Show saved queues, once.
  check FILE.json                      Validate a route without publishing.
ROUNDS: 1 = once, 2 = twice, -1 = indefinitely. YAW_DEG is in degrees.
Coordinates use the saved map; Z is the planning-map surface height.
Start navigation and initialize localization before start/run.
Queue execution continues after this command returns; use stop to cancel.
HELP
}

case "${1:---help}" in -h|--help|help) usage; exit 0 ;; esac
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh"
dog_nav_require_nav
dog_nav_source_nav

validate_file() {
  python3 - "$1" <<'PY'
import json
import math
import sys
from pathlib import Path

try:
    path = Path(sys.argv[1])
    route = json.loads(path.read_text())
    points = route.get("points") if isinstance(route, dict) else None
    if not isinstance(points, list) or not points:
        raise ValueError("route must contain a nonempty points array")
    for i, point in enumerate(points, 1):
        if not isinstance(point, dict):
            raise ValueError("point %d must be an object" % i)
        for key in ("x", "y", "z", "qx", "qy", "qz", "qw"):
            value = point.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError("point %d: %s must be a finite number" % (i, key))
        if sum(point[k] ** 2 for k in ("qx", "qy", "qz", "qw")) < 1e-12:
            raise ValueError("point %d: invalid zero quaternion" % i)
        if point.get("record", 0) != 0:
            raise ValueError("point %d: record must be 0 for unattended navigation" % i)
        if point.get("map_id", 0) not in (0, 1, 2) or point.get("map_label", "map") not in ("map", "floor1", "floor2"):
            raise ValueError("point %d: unsupported map label/id" % i)
    print("Validated %s: %d waypoints" % (path, len(points)))
except (OSError, ValueError, OverflowError) as exc:
    print("Invalid route: %s" % exc, file=sys.stderr)
    sys.exit(2)
PY
}

valid_name() {
  if [[ ! "$1" =~ ^[A-Za-z0-9_-]+$ ]]; then
    echo "Queue name must contain only letters, numbers, '_' or '-'." >&2
    return 2
  fi
}

rounds_value() {
  if [[ "$1" != -1 && ! "$1" =~ ^[1-9][0-9]*$ ]]; then
    echo "ROUNDS must be a positive integer or -1." >&2
    return 2
  fi
  python3 - "$1" <<'PY'
import sys
value = int(sys.argv[1])
if value > 2147483647:
    raise SystemExit("ROUNDS exceeds Int32 range")
print(0 if value == 1 else value)
PY
}

pub() {
  timeout 15s rostopic pub -1 "$1" "$2" "$3" >/dev/null
}

load_queue() {
  valid_name "$1"
  local queue_dir
  queue_dir="$(rosparam get /goal_queue/queue_dir)"
  validate_file "${queue_dir}/$1.json"
  pub /goal_queue/stop std_msgs/Empty '{}'
  pub /goal_queue/load_named std_msgs/String "{data: '$1'}"
}

command="$1"
shift
case "$command:$#" in
  check:1) validate_file "$1"; exit 0 ;;
  add:4|add:5|save:1|load:1|run:1|run:2|start:0|start:1|pause:0|resume:0|stop:0|clear:0|list:0) ;;
  *) usage >&2; exit 2 ;;
esac

# Validate all user arguments before changing an existing task.
payload=""
case "$command" in
  add)
    payload="$(python3 - "$@" <<'PY'
import json
import math
import sys
try:
    x, y, z, degrees = map(float, sys.argv[1:5])
    if not all(math.isfinite(v) for v in (x, y, z, degrees)):
        raise ValueError("coordinates and yaw must be finite")
    label = sys.argv[5] if len(sys.argv) > 5 else "map"
    if label not in ("map", "floor1", "floor2"):
        raise ValueError("map must be map, floor1 or floor2")
    half = math.radians(degrees) / 2
    print(json.dumps({"header": {"frame_id": label}, "pose": {
        "position": dict(x=x, y=y, z=z),
        "orientation": dict(x=0., y=0., z=math.sin(half), w=math.cos(half))}}))
except ValueError as exc:
    raise SystemExit(str(exc))
PY
)" ;;
  save|load) valid_name "$1" ;;
  run) valid_name "$1"; rounds="$(rounds_value "${2:-1}")" ;;
  start) rounds="$(rounds_value "${1:-1}")" ;;
esac

if [ "$command" = stop ]; then
  pub /map_manager/cancel std_msgs/Empty '{}'
  if timeout 2s rosnode ping -c 1 /goal_queue >/dev/null 2>&1; then
    pub /goal_queue/stop std_msgs/Empty '{}'
  fi
  exit 0
fi
if ! timeout 5s rosnode ping -c 1 /goal_queue >/dev/null 2>&1; then
  echo "goal_queue is unavailable. Start navigation with start_goal_queue:=true." >&2
  exit 1
fi
case "$command" in
  start|run)
    if [ "$(rosparam get /goal_queue/use_map_manager)" != true ]; then
      echo "Require /goal_queue/use_map_manager=true to wait for final navigation results." >&2
      exit 1
    fi
    timeout 7s python3 - <<'PY'
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
rospy.init_node("multi_nav_ready_check", anonymous=True, disable_signals=True)
try:
    status = rospy.wait_for_message("/dog_nav/localization_status", String, timeout=3)
    if status.data != "TRACKING":
        raise ValueError("localization is %s, not TRACKING" % status.data)
    odom = rospy.wait_for_message("/dog_nav/odometry", Odometry, timeout=3)
    age = (rospy.Time.now() - odom.header.stamp).to_sec()
    if odom.header.stamp.is_zero() or not -.1 <= age <= .5:
        raise ValueError("navigation odometry is stale")
except (ValueError, rospy.ROSException) as exc:
    raise SystemExit("Navigation not ready: %s" % exc)
PY
    ;;
esac

case "$command" in
  add) pub /goal_queue/add_pose geometry_msgs/PoseStamped "$payload" ;;
  save) pub /goal_queue/save_named std_msgs/String "{data: '$1'}" ;;
  load) load_queue "$1" ;;
  start)
    pub /goal_queue/set_loop_count std_msgs/Int32 "{data: $rounds}"
    pub /goal_queue/start std_msgs/Empty '{}' ;;
  run)
    load_queue "$1"
    pub /goal_queue/set_loop_count std_msgs/Int32 "{data: $rounds}"
    pub /goal_queue/start std_msgs/Empty '{}' ;;
  pause) pub /goal_queue/pause std_msgs/Empty '{}' ;;
  resume) pub /goal_queue/continue std_msgs/Empty '{}' ;;
  stop)
    pub /goal_queue/stop std_msgs/Empty '{}'
    pub /map_manager/cancel std_msgs/Empty '{}' ;;
  clear)
    pub /goal_queue/stop std_msgs/Empty '{}'
    pub /goal_queue/clear std_msgs/Empty '{}' ;;
  list) timeout 5s rostopic echo -n 1 /goal_queue/list ;;
esac
