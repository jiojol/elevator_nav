#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.sh"
CHECK_ONLY=false
if [ "${1:-}" = check ]; then
  CHECK_ONLY=true
  shift
fi
MODE="${1:-body}"
case "$MODE" in
  body) IMU_TOPIC=/go2/imu; ACC_NORM=9.805 ;;
  internal) IMU_TOPIC=/livox/imu; ACC_NORM=1 ;;
  -h|--help)
    printf '%s\n' 'Usage: bash system/scripts/start_lidar_calibration.sh [check] [body|internal]' \
      'body: LiDAR -> Go2 base_imu; stock Unitree body/IMU geometry is saved for candidate composition.' \
      'internal: LiDAR -> internal Livox IMU. Does not calibrate robot mounting.' \
      'No driver, robot motion, or automatic navigation parameter update is started.'
    exit 0 ;;
  *) echo "Unknown IMU mode: $MODE" >&2; exit 2 ;;
esac
if [ "$#" -gt 1 ]; then
  echo 'Too many arguments; use --help.' >&2
  exit 2
fi

WORKSPACE="${DOG_NAV_LI_INIT_WS:-/home/nvidia/go2_nav/lidar_imu_init_ws}"
if [ ! -f "$WORKSPACE/devel/setup.bash" ] || \
   [ ! -x "$WORKSPACE/devel/lib/lidar_imu_init/li_init" ]; then
  echo "Built LI-Init workspace not found: $WORKSPACE" >&2
  exit 1
fi
dog_nav_source_nav
set +u
source "$WORKSPACE/devel/setup.bash"
set -u
PACKAGE="$(rospack find lidar_imu_init)"
LAUNCH="$DOG_NAV_HOME/nav/src/dog_nav_bringup/launch/lidar_imu_calibration.launch"
RESULT="$PACKAGE/result/Initialization_result.txt"
BODY_IMU="$DOG_NAV_HOME/nav/src/dog_nav_bringup/config/go2_body_imu.yaml"
if [ "$MODE" = body ]; then
  python3 - "$BODY_IMU" <<'PY'
import sys
import yaml
with open(sys.argv[1]) as handle:
    config = yaml.safe_load(handle)
print('Stock Go2 base_link -> base_imu: xyz=%s, quaternion_xyzw=%s' %
      (config['translation'], config['rotation_xyzw']))
PY
fi
roslaunch --dump-params "$LAUNCH" "imu_topic:=$IMU_TOPIC" "mean_acc_norm:=$ACC_NORM" >/dev/null

for spec in '/livox/lidar livox_ros_driver2/CustomMsg' "$IMU_TOPIC sensor_msgs/Imu"; do
  read -r topic expected <<< "$spec"
  actual="$(timeout 4 rostopic type "$topic" 2>/dev/null || true)"
  if [ "$actual" != "$expected" ]; then
    echo "Need live input: $topic ($expected); found: ${actual:-none}" >&2
    echo 'Start the LiDAR driver and the IMU publisher, not navigation.' >&2
    exit 1
  fi
done
printf 'LI-Init: %s\nLiDAR: /livox/lidar\nIMU: %s\nStationary acceleration norm setting: %s\n' \
  "$WORKSPACE" "$IMU_TOPIC" "$ACC_NORM"
echo 'The output transforms LiDAR points into the selected IMU frame, NOT automatically into base_link.'
if [ "$CHECK_ONLY" = true ]; then
  echo 'Topic/configuration check passed. No calibration node started; no files or navigation parameters changed.'
  exit 0
fi

NODES="$(rosnode list)"
if printf '%s\n' "$NODES" | rg -q '^/(global_planner|scan_planner_node|fastlio_map_localizer|fastlio_nav/laserMapping|laserMapping|lidar_mount_calibration/laserMapping)$'; then
  echo 'Stop navigation, mapping, and other calibration nodes before calibration.' >&2
  exit 1
fi
python3 - "$IMU_TOPIC" "$ACC_NORM" <<'PY'
import math
import sys
import rospy
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import Imu

rospy.init_node('lidar_calibration_input_check', anonymous=True, disable_signals=True)
try:
    cloud = rospy.wait_for_message('/livox/lidar', CustomMsg, timeout=4)
    imu = rospy.wait_for_message(sys.argv[1], Imu, timeout=4)
except rospy.ROSException as error:
    raise SystemExit('Missing fresh calibration input: %s' % error)
for label, msg in [('LiDAR', cloud), ('IMU', imu)]:
    age = (rospy.Time.now() - msg.header.stamp).to_sec()
    if not -1. <= age <= 1.:
        raise SystemExit('%s timestamp is stale or from a different clock: age=%.3f s' % (label, age))
if not cloud.point_num or not cloud.points:
    raise SystemExit('LiDAR scan is empty')
if sys.argv[1] == '/go2/imu' and imu.header.frame_id.lstrip('/') != 'base_imu':
    raise SystemExit('Expected /go2/imu in base_imu, found: %s' % imu.header.frame_id)
a = imu.linear_acceleration
norm = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
expected = float(sys.argv[2])
if not math.isfinite(norm) or not .6 * expected < norm < 1.4 * expected:
    raise SystemExit('Hold still and check IMU units: acceleration norm %.4f, expected near %.4f' % (norm, expected))
print('Fresh input verified. IMU frame=%s, acceleration norm=%.4f' % (imu.header.frame_id, norm))
PY
OUT="$DOG_NAV_HOME/system/log/lidar_calibration_$(date +%Y%m%d_%H%M%S)_$$_$MODE"
mkdir -p "$OUT"
if [ "$MODE" = body ]; then
  cp -p "$BODY_IMU" "$OUT/body_imu_geometry.yaml"
fi
if [ -f "$RESULT" ]; then
  cp -p "$RESULT" "$OUT/previous_Initialization_result.txt"
fi
roslaunch --dump-params "$LAUNCH" "imu_topic:=$IMU_TOPIC" "mean_acc_norm:=$ACC_NORM" > "$OUT/params.yaml"
save_result() {
  if [ -f "$RESULT" ]; then
    cp -p "$RESULT" "$OUT/candidate_Initialization_result.txt"
    if [ "$MODE" = body ] && ! cmp -s "$RESULT" "$OUT/previous_Initialization_result.txt"; then
      if ! python3 "$SCRIPT_DIR/lidar_mount_candidate.py" "$OUT/candidate_Initialization_result.txt" \
          --body-imu "$OUT/body_imu_geometry.yaml" > "$OUT/candidate_mount.yaml"; then
        rm -f "$OUT/candidate_mount.yaml"
      fi
    fi
  fi
  echo "Calibration logs/candidate result: $OUT"
  echo 'A candidate file is not proof of convergence. Review refinement and excitation before use.'
}
trap save_result EXIT
echo 'Keep the platform stationary for at least 5 seconds, then follow LI-Init excitation prompts.'
echo 'No robot motion commands are issued. Existing navigation calibration will NOT be updated.'
roslaunch "$LAUNCH" "imu_topic:=$IMU_TOPIC" "mean_acc_norm:=$ACC_NORM" 2>&1 | tee "$OUT/console.log"
