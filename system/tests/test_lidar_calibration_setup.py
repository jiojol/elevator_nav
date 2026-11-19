#!/usr/bin/env python3
"""Check calibration configuration and interlocks without running LI-Init."""
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import threading
import time
import xmlrpc.client

import yaml


def main():
    root = Path(__file__).resolve().parents[2]
    workspace = Path(os.environ.get('DOG_NAV_LI_INIT_WS', '/home/nvidia/go2_nav/lidar_imu_init_ws'))
    result = workspace / 'src/LiDAR_IMU_Init/result/Initialization_result.txt'
    original = result.read_bytes() if result.exists() else None
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % port, ROS_HOSTNAME='127.0.0.1')
    os.environ.pop('ROS_IP', None)
    logs = Path(tempfile.mkdtemp(prefix='dog-nav-calibration-setup-'))
    with (logs / 'master.log').open('w') as handle:
        master = subprocess.Popen(['roscore', '-p', str(port)], stdout=handle,
                                  stderr=subprocess.STDOUT, start_new_session=True)
        try:
            server = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
            deadline = time.monotonic() + 15
            while True:
                try:
                    if server.getPid('/calibration_setup_test')[0] == 1:
                        break
                except OSError:
                    pass
                assert time.monotonic() < deadline
                time.sleep(.1)
            import rospy
            from livox_ros_driver2.msg import CustomMsg, CustomPoint
            from sensor_msgs.msg import Imu
            rospy.init_node('calibration_setup_test', disable_signals=True)
            # Register only a fake publisher, never an actual navigation node.
            server.registerPublisher('/global_planner', '/test/nav_guard', 'std_msgs/Bool', rospy.get_node_uri())
            pubs = [rospy.Publisher(topic, msg, queue_size=1) for topic, msg in (
                ('/livox/lidar', CustomMsg), ('/go2/imu', Imu), ('/livox/imu', Imu))]
            script = root / 'system/scripts/start_lidar_calibration.sh'
            for mode, topic, norm in [('body', '/go2/imu', 9.805), ('internal', '/livox/imu', 1.)]:
                completed = subprocess.run(['bash', str(script), 'check', mode], cwd=root,
                                           text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30)
                assert completed.returncode == 0, completed.stdout
                assert 'IMU: ' + topic in completed.stdout
                command = ['bash', '-c', 'source "$1/devel/setup.bash" && roslaunch --dump-params "$2" "imu_topic:=$3" "mean_acc_norm:=$4"',
                           'calibration-test', str(workspace),
                           str(root / 'nav/src/dog_nav_bringup/launch/lidar_imu_calibration.launch'), topic, str(norm)]
                params = yaml.safe_load(subprocess.check_output(command, text=True, timeout=20))
                prefix = '/lidar_mount_calibration/'
                assert params[prefix + 'common/imu_topic'] == topic
                assert params[prefix + 'initialization/mean_acc_norm'] == norm
                assert params[prefix + 'preprocess/scan_line'] == 32
                print('PASS: %s mode uses %s with acceleration norm %s' % (mode, topic, norm), flush=True)
            blocked = subprocess.run(['bash', str(script), 'body'], cwd=root, text=True,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30)
            assert blocked.returncode != 0 and 'Stop navigation' in blocked.stdout, blocked.stdout
            server.unregisterPublisher('/global_planner', '/test/nav_guard', rospy.get_node_uri())
            state, finished = {'stale': False}, threading.Event()
            def publish_bad_inputs():
                while not finished.is_set():
                    cloud = CustomMsg()
                    cloud.header.stamp = rospy.Time.now() - rospy.Duration(5 if state['stale'] else 0)
                    cloud.point_num = 1
                    cloud.points = [CustomPoint(x=1.)]
                    imu = Imu()
                    imu.header.stamp = rospy.Time.now()
                    imu.header.frame_id = 'base_imu'
                    imu.linear_acceleration.z = 1.  # Wrong units for body mode.
                    pubs[0].publish(cloud)
                    pubs[1].publish(imu)
                    finished.wait(.05)
            thread = threading.Thread(target=publish_bad_inputs, daemon=True)
            thread.start()
            try:
                for stale, expected in [(False, 'check IMU units'), (True, 'timestamp is stale')]:
                    state['stale'] = stale
                    rejected = subprocess.run(['bash', str(script), 'body'], cwd=root, text=True,
                                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30)
                    assert rejected.returncode != 0 and expected in rejected.stdout, rejected.stdout
                    print('PASS: refuses %s before starting LI-Init' % expected, flush=True)
            finally:
                finished.set()
                thread.join(timeout=2)
            assert (result.read_bytes() if result.exists() else None) == original
            assert not any('lidar_mount_calibration' in name for name in rospy.get_param_names())
            assert len(pubs) == 3
            print('PASS: active-navigation interlock refuses calibration, preserving result and ROS parameters', flush=True)
        finally:
            os.killpg(master.pid, signal.SIGINT)
            try:
                master.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(master.pid, signal.SIGKILL)
                master.wait(timeout=3)
    print('All isolated calibration setup checks passed. Logs:', logs)


if __name__ == '__main__':
    main()
