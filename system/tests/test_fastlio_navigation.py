#!/usr/bin/env python3
"""Exercise the actual GICP/localizer on a private master, with no robot drivers.

Tests base/IMU/lidar frame composition, climbing/turning, correction smoothing,
stale input, bad map matches and LIO resets. Does not publish cmd_vel.
"""
import argparse
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import xmlrpc.client

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import yaml


def transform(position=(0, 0, 0), rpy=(0, 0, 0)):
    out = np.eye(4)
    out[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    out[:3, 3] = position
    return out


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mount-file', type=Path, help='test a candidate body-to-LiDAR transform instead of the legacy fixture')
    parser.add_argument('--prefix', type=Path, default=root/'nav/devel')
    args = parser.parse_args()
    logs = Path(tempfile.mkdtemp(prefix="dog-nav-fastlio-test-"))
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    os.environ.update(ROS_MASTER_URI="http://127.0.0.1:%d" % port, ROS_HOSTNAME="127.0.0.1", OMP_NUM_THREADS="2")
    os.environ.pop("ROS_IP", None)
    os.environ.pop("ROS_NAMESPACE", None)
    processes, handles = [], []

    def launch(name, argv):
        handle = (logs / (name + ".log")).open("w")
        handles.append(handle)
        p = subprocess.Popen(argv, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(p)
        return p

    try:
        launch("master", ["roscore", "-p", str(port)])
        master = xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"])
        deadline = time.monotonic() + 15
        while True:
            try:
                if master.getPid("/test")[0] == 1:
                    break
            except OSError:
                pass
            assert time.monotonic() < deadline, "private ROS master failed"
            time.sleep(.1)
        import rospy
        import tf2_ros
        from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2, PointField
        from sensor_msgs.point_cloud2 import read_points
        from std_msgs.msg import Header, String

        rospy.init_node("fastlio_navigation_test", disable_signals=True)
        config = yaml.safe_load((root / "nav/src/dog_nav_bringup/config/fastlio_localization.yaml").read_text())
        config.update(registration_period=.30, global_match_timeout=1.5)
        for key, value in config.items():
            rospy.set_param("/fastlio_map_localizer/" + key, value)
        calibration = yaml.safe_load((root / "slam/src/FAST_LIO/config/mid360.yaml").read_text())["mapping"]
        for key in ("extrinsic_R", "extrinsic_T"):
            rospy.set_param("/fastlio_nav/mapping/" + key, calibration[key])
        imu_lidar = np.eye(4)
        imu_lidar[:3, :3] = np.array(calibration["extrinsic_R"]).reshape(3, 3)
        imu_lidar[:3, 3] = calibration["extrinsic_T"]
        base_lidar = np.eye(4)
        base_lidar[:3, :3] = Rotation.from_quat([.008341340828, .078508271507, .011504925819, .996812173892]).as_matrix()
        base_lidar[:3, 3] = [.220036, .007602, .121942]
        if args.mount_file:
            candidate = yaml.safe_load(args.mount_file.read_text())
            assert (candidate['parent_frame'], candidate['child_frame']) == ('base_link', 'livox_frame')
            base_lidar[:3, :3] = Rotation.from_quat(candidate['rotation_xyzw']).as_matrix()
            base_lidar[:3, 3] = candidate['translation']
        base_imu = base_lidar @ np.linalg.inv(imu_lidar)
        static = tf2_ros.StaticTransformBroadcaster()
        mount = TransformStamped(header=Header(stamp=rospy.Time.now(), frame_id="base_link"), child_frame_id="livox_frame")
        mount.transform.translation.x, mount.transform.translation.y, mount.transform.translation.z = base_lidar[:3, 3]
        mount.transform.rotation.x, mount.transform.rotation.y, mount.transform.rotation.z, mount.transform.rotation.w = Rotation.from_matrix(base_lidar[:3, :3]).as_quat()
        static.sendTransform(mount)

        rng = np.random.RandomState(42)
        surfaces = []
        for axis, level in [(0, -4), (0, 7), (1, -5), (1, 6), (2, -.4), (2, 3)]:
            pts = rng.uniform([-4, -5, -.4], [7, 6, 3], size=(1800, 3))
            pts[:, axis] = level + rng.normal(0, .005, len(pts))
            surfaces.append(pts)
        for center in [(2, 3), (-2, -2)]:
            phi = rng.uniform(0, 2 * math.pi, 700)
            surfaces.append(np.column_stack((center[0] + .4 * np.cos(phi), center[1] + .4 * np.sin(phi), rng.uniform(0, 3, len(phi)))))
        world = np.concatenate(surfaces)
        map_path = logs / "room.pcd"
        data = np.column_stack((world, np.ones(len(world)))).astype("<f4")
        header = "VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\nPOINTS %d\nDATA binary\n" % (len(data), len(data))
        map_path.write_bytes(header.encode() + data.tobytes())
        rospy.set_param("/fastlio_map_localizer/globalmap_pcd", str(map_path))
        node = launch("localizer", [str(args.prefix / "lib/dog_nav_bringup/fastlio_map_localizer")])
        odom_pub = rospy.Publisher("/fastlio_nav/odometry", Odometry, queue_size=5)
        cloud_pub = rospy.Publisher("/fastlio_nav/cloud_body", PointCloud2, queue_size=5)
        initial_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        outputs, sensor_poses, clouds, statuses, occupancy = [], [], [], [], []
        rospy.Subscriber("/dog_nav/odometry", Odometry, outputs.append)
        rospy.Subscriber("/dog_nav/sensor_pose", Odometry, sensor_poses.append)
        rospy.Subscriber("/navigation/obstacle_points", PointCloud2, clouds.append)
        rospy.Subscriber("/dog_nav/localization_status", String, lambda msg: statuses.append(msg.data))
        # Use the real consumer as well as checking publisher data, so a ROS
        # message-type mismatch cannot pass by copying it into the test.
        scan_params = yaml.safe_load(subprocess.check_output([
            "roslaunch", "--dump-params", str(root/"nav/src/dog_nav_bringup/launch/prior_local.launch")], text=True))
        for key, value in scan_params.items():
            if key.startswith("/scan_planner_node/") or key == "/body_pose_topic":
                rospy.set_param(key, value)
        rospy.set_param('/body_pose_topic', '/dog_nav/odometry')
        planner = launch("scan", [str(args.prefix / "lib/scan_planner/scan_planner_node"),
                          "/grid_map/body_pose:=/dog_nav/odometry",
                          "/grid_map/sensor_pose:=/dog_nav/sensor_pose",
                          "/grid_map/cloud:=/navigation/obstacle_points",
                          "/cmd_vel:=/test_output/cmd_vel"])
        rospy.Subscriber("/grid_map/occupancy", PointCloud2, lambda msg: occupancy.append(msg.width))
        deadline = time.monotonic() + 15
        while not (odom_pub.get_num_connections() and cloud_pub.get_num_connections() and initial_pub.get_num_connections()):
            assert node.poll() is None, "localizer exited; see " + str(logs)
            assert time.monotonic() < deadline, "localizer subscriptions unavailable"
            time.sleep(.1)
        map_local = transform([1.2, -.6, .25], [0, 0, .3])
        local_base = transform([0, 0, .2], [.03, -.04, 0])
        fields = [PointField(name, i * 4, PointField.FLOAT32, 1) for i, name in enumerate(["x", "y", "z", "intensity"])]
        expected = {}
        previous = [None, None]

        def pose(target, matrix):
            target.position.x, target.position.y, target.position.z = matrix[:3, 3]
            target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w = Rotation.from_matrix(matrix[:3, :3]).as_quat()

        def send(base, shift=None, bad=False):
            stamp = rospy.Time.now()
            local_imu = base @ base_imu
            map_imu = map_local @ local_imu
            points = (world - map_imu[:3, 3]) @ map_imu[:3, :3]
            if shift is not None:
                points += shift @ map_imu[:3, :3]
            if bad:
                points += [0, 0, 12]
            values = np.column_stack((points, np.ones(len(points)))).astype("<f4")
            odom = Odometry(header=Header(stamp=stamp, frame_id="camera_init"), child_frame_id="body")
            pose(odom.pose.pose, local_imu)
            cloud = PointCloud2(header=Header(stamp=stamp, frame_id="body"), height=1, width=len(values), fields=fields,
                                point_step=16, row_step=16 * len(values), data=values.tobytes(), is_dense=True)
            velocity = np.zeros(3)
            if previous[0] is not None:
                velocity = base[:3, :3].T @ (base[:3, 3] - previous[1][:3, 3]) / (stamp - previous[0]).to_sec()
            previous[:] = stamp, base.copy()
            expected[stamp.to_nsec()] = (map_local @ base, map_imu @ imu_lidar, velocity)
            odom_pub.publish(odom)
            cloud_pub.publish(cloud)
            time.sleep(.12)

        for _ in range(3):
            send(local_base)
        assert not outputs, "navigation odometry exists before initial map alignment"
        initial = PoseWithCovarianceStamped(header=Header(stamp=rospy.Time.now(), frame_id="map"))
        pose(initial.pose.pose, map_local @ local_base @ transform([.12, -.08, .08], [0, 0, .04]))
        initial_pub.publish(initial)
        for _ in range(40):
            send(local_base)
            if len(outputs) >= 3:
                break
        assert len(outputs) >= 3, "initial map registration failed; see " + str(logs)
        print("PASS: navigation waits for a successful map alignment", flush=True)

        # Reproduce the recorded 5.5-degree map/LIO gravity difference. Surface
        # clicks vary in XYZ and yaw, but must recover the same measured pose.
        map_local = transform([1.2, -.6, .25], [.03, .095, .3])
        for offset, turn in [([0., 0., -.4], -.3), ([.45, -.4, -.5], .35),
                             ([-.5, .4, .3], -.4), ([.2, .3, 0.], .2)]:
            first = len(outputs)
            initial.header.stamp = rospy.Time.now()
            pose(initial.pose.pose, map_local @ local_base @ transform(offset, [0, 0, turn]))
            initial_pub.publish(initial)
            for _ in range(70):
                send(local_base)
                if len(outputs) >= first + 4 and outputs[-1].header.stamp > initial.header.stamp:
                    break
            assert len(outputs) >= first + 4, 'nearby initial hint failed: %s; logs: %s' % (offset, logs)
            true = map_local @ local_base
            p = outputs[-1].pose.pose.position
            assert np.linalg.norm(np.array([p.x,p.y,p.z])-true[:3,3]) < .06, 'incorrect nearby relocalization'
        print('PASS: repeated nearby XYZ/yaw hints converge with a 5.7-degree map tilt', flush=True)

        # A wide tilt bound is not permission to localize against unrelated
        # scans. Reinitialization must withhold odometry until geometry agrees.
        first = len(outputs)
        initial.header.stamp = rospy.Time.now()
        pose(initial.pose.pose, map_local @ local_base)
        initial_pub.publish(initial)
        for _ in range(30):
            send(local_base,bad=True)
        assert len(outputs) <= first + 1, 'bad initialization produced fresh navigation odometry'
        assert statuses[-1] == 'ALIGNING', statuses
        for _ in range(60):
            send(local_base)
            if len(outputs) >= first + 4:
                break
        assert len(outputs) >= first + 4, 'valid initialization did not recover after bad scans'
        print('PASS: unrelated initial scans remain blocked; valid scans recover', flush=True)

        first = len(outputs)
        for i in range(35):
            local_base = transform([.025 * i, .003 * i, .2 + .006 * i], [.03, -.04 + .003 * i, .012 * i])
            send(local_base)
        errors = []
        for msg in outputs[first:]:
            true, _, velocity = expected[msg.header.stamp.to_nsec()]
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            estimated_rot = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            errors.append([np.linalg.norm([p.x - true[0, 3], p.y - true[1, 3], p.z - true[2, 3]]),
                           Rotation.from_matrix(estimated_rot.T @ true[:3, :3]).magnitude()])
            actual_v = msg.twist.twist.linear
            assert np.linalg.norm(np.array([actual_v.x, actual_v.y, actual_v.z]) - velocity) < 1e-5, "twist is not base-frame local velocity"
        assert len(errors) >= 30, "map registration blocked continuous LIO output"
        maximum = np.max(errors, axis=0)
        assert maximum[0] < .05 and maximum[1] < .015, "frame/calibration or motion mismatch: " + str(maximum)
        last_sensor = sensor_poses[-1]
        assert last_sensor.child_frame_id == "livox_frame" and last_sensor.header.frame_id == "map"
        _, true_sensor, _ = expected[last_sensor.header.stamp.to_nsec()]
        p = last_sensor.pose.pose.position
        assert np.linalg.norm(np.array([p.x, p.y, p.z]) - true_sensor[:3, 3]) < .05, "lidar ray origin transformed twice"
        world_scan = np.array(list(read_points(clouds[-1], field_names=("x", "y", "z"), skip_nans=True)))
        assert np.percentile(cKDTree(world).query(world_scan)[0], 95) < .06, "deskewed cloud frame mismatch"
        print("PASS: continuous translation/turn/climb, max %.3f m / %.2f deg; mount and deskewed cloud correct" % (maximum[0], math.degrees(maximum[1])), flush=True)
        assert planner.poll() is None and any(n > 0 for n in occupancy), "real SCAN did not fuse lidar pose/cloud into its occupancy map"
        print("PASS: real SCAN consumes Odometry sensor pose and updates obstacle occupancy", flush=True)

        # A map displacement may slowly adjust map->odom, but must never become
        # a fictitious body velocity while FAST-LIO's local pose is stationary.
        first = len(outputs)
        for _ in range(28):
            send(local_base, shift=np.array([.12, 0, 0]))
        corrected = outputs[first:]
        assert len(corrected) >= 23, "correction interrupted continuous output"
        distance = abs(corrected[-1].pose.pose.position.x - corrected[0].pose.pose.position.x)
        assert .04 < distance < .16, "map correction did not apply smoothly: " + str(distance)
        for a, b in zip(corrected, corrected[1:]):
            dt = (b.header.stamp - a.header.stamp).to_sec()
            pa, pb = a.pose.pose.position, b.pose.pose.position
            assert np.linalg.norm([pb.x - pa.x, pb.y - pa.y, pb.z - pa.z]) < .10 * dt + .001, "abrupt global pose correction"
            v = b.twist.twist.linear
            assert np.linalg.norm([v.x, v.y, v.z]) < 1e-6, "global correction leaked into body velocity"
        print("PASS: gradual global correction preserves local velocity", flush=True)
        for _ in range(25):
            send(local_base, bad=True)
        assert statuses[-1] == "LOST_MAP_MATCH", statuses
        count = len(outputs)
        for _ in range(4):
            send(local_base, bad=True)
        assert len(outputs) == count, "invalid map matching refreshed navigation odometry"
        for _ in range(30):
            send(local_base)
            if statuses[-1] == "TRACKING":
                break
        assert statuses[-1] == "TRACKING", "valid map match did not recover"
        print("PASS: failed map matches withhold fresh odometry and valid matches recover", flush=True)
        time.sleep(.8)
        assert statuses[-1] == "LOST_LIO_INPUT", "stale LIO input not detected"
        count = len(outputs)
        jumped = local_base.copy()
        jumped[0, 3] += 10
        for _ in range(4):
            send(jumped)
        assert statuses[-1] == "LIO_RESET_SET_INITIAL_POSE" and len(outputs) == count, "LIO reset accepted without reinitialization"
        print("PASS: stale input and LIO reset stop fresh navigation output", flush=True)
        print("All FAST-LIO adapter integration checks passed. Logs: " + str(logs), flush=True)
    finally:
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
                try:
                    p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGKILL)
                    p.wait(timeout=3)
        for handle in handles:
            handle.close()
        print("Private test processes stopped. Logs: " + str(logs), flush=True)


if __name__ == "__main__":
    main()
