#!/usr/bin/env python3
"""Validate the opt-in calibrated mount without launching hardware or nodes."""
from pathlib import Path
import shlex
import subprocess

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def main():
    root = Path(__file__).resolve().parents[2]
    scene = root / 'system/log/lidar_calibration_20261016_065841_62760_body'
    candidate = yaml.safe_load((scene / 'candidate_mount.yaml').read_text())
    for launch, options in [('lidar.launch', ['xfer_format:=1']),
                            ('navigation.launch', ['start_rviz:=false'])]:
        for enabled in (False, True):
            files = ['dog_nav_bringup', launch, 'use_calibrated_mount:=' + str(enabled).lower()] + options
            nodes = subprocess.check_output(['roslaunch', '--nodes'] + files, text=True).splitlines()
            assert nodes.count('/base2livox') == 1, nodes
            assert nodes.count('/livox_lidar_publisher2') == 1, nodes
            command = shlex.split(subprocess.check_output(['roslaunch', '--args', '/base2livox'] + files, text=True))
            xyz, quat = np.array(command[1:4], dtype=float), np.array(command[4:8], dtype=float)
            assert [item.lstrip('/') for item in command[8:10]] == ['base_link', 'livox_frame']
            np.testing.assert_allclose(xyz, candidate['translation'] if enabled else [0, 0, 0], atol=1e-9)
            expected = Rotation.from_quat(candidate['rotation_xyzw']).as_matrix() if enabled else np.eye(3)
            np.testing.assert_allclose(Rotation.from_quat(quat).as_matrix(), expected, atol=1e-10)
            print('PASS: %s %s calibrated=%s has one correct mount TF and one driver' % (launch, options, enabled), flush=True)
    params = yaml.safe_load(subprocess.check_output([
        'roslaunch', '--dump-params', 'dog_nav_bringup', 'navigation.launch',
        'use_calibrated_mount:=true', 'start_rviz:=false'], text=True))
    calibration = yaml.safe_load((root / 'slam/src/FAST_LIO/config/mid360.yaml').read_text())
    for key in ('extrinsic_R', 'extrinsic_T'):
        assert params['/fastlio_nav/mapping/' + key] == calibration['mapping'][key]
    assert params['/fastlio_nav/common/imu_topic'] == '/livox/imu'
    assert params['/fastlio_nav/common/time_offset_lidar_to_imu'] == calibration['common']['time_offset_lidar_to_imu']
    print('PASS: body mounting trial leaves FAST-LIO internal IMU selection, extrinsics and time offset unchanged', flush=True)
    print('All launch-only checks passed; no ROS master, drivers, or navigation nodes were started.', flush=True)


if __name__ == '__main__':
    main()
