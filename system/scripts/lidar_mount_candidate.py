#!/usr/bin/env python3
"""Compose a reviewed LI-Init body-IMU refinement with stock Go2 geometry.

Outputs an unvalidated candidate only; never publishes TF or updates navigation.
"""
import argparse
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def validate_transform(value):
    value = np.asarray(value, dtype=float)
    if value.shape != (4, 4) or not np.isfinite(value).all():
        raise ValueError('transform must be a finite 4x4 matrix')
    if not np.allclose(value[3], [0, 0, 0, 1], atol=1e-6):
        raise ValueError('invalid homogeneous last row')
    rotation = value[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-4) or abs(np.linalg.det(rotation) - 1.) > 2e-4:
        raise ValueError('rotation is not a proper orthogonal matrix')
    return value


def read_refinement(text):
    # LI-Init exports two labeled records; never substitute the initial estimate
    # or silently accept an unfinished refinement as the final result.
    marker = 'Refinement result:'
    if marker not in text:
        raise ValueError('no Refinement result record')
    lines = text.rsplit(marker, 1)[1].splitlines()
    for index, line in enumerate(lines):
        if line.strip().startswith('Homogeneous Transformation Matrix from LiDAR to IMU:'):
            rows = [row.split() for row in lines[index + 1:index + 5]]
            return validate_transform(rows)
    raise ValueError('refinement has no complete LiDAR-to-IMU matrix')


def compose_candidate(text, config):
    if (config['parent_frame'], config['child_frame'], config['imu_topic']) != ('base_link', 'base_imu', '/go2/imu'):
        raise ValueError('expected Go2 base_link/base_imu geometry for /go2/imu')
    translation = np.asarray(config['translation'], dtype=float)
    quaternion = np.asarray(config['rotation_xyzw'], dtype=float)
    if translation.shape != (3,) or not np.isfinite(translation).all():
        raise ValueError('invalid body-IMU translation')
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all() or abs(np.linalg.norm(quaternion) - 1.) > 1e-4:
        raise ValueError('invalid body-IMU unit quaternion')
    base_imu = np.eye(4)
    base_imu[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    base_imu[:3, 3] = translation
    # T_A_B maps B points into A: T_B_L = T_B_I @ T_I_L, without inversion.
    base_lidar = validate_transform(base_imu @ read_refinement(text))
    return {
        'status': 'unvalidated_candidate',
        'parent_frame': 'base_link', 'child_frame': 'livox_frame',
        'calibration_imu_topic': '/go2/imu',
        'translation': base_lidar[:3, 3].tolist(),
        'rotation_xyzw': Rotation.from_matrix(base_lidar[:3, :3]).as_quat().tolist(),
        'matrix': base_lidar.tolist(),
        'body_imu_geometry': config,
    }


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('result', type=Path, help='LI-Init refinement obtained specifically using /go2/imu')
    parser.add_argument('--body-imu', type=Path,
                        default=root / 'nav/src/dog_nav_bringup/config/go2_body_imu.yaml')
    args = parser.parse_args()
    try:
        candidate = compose_candidate(args.result.read_text(), yaml.safe_load(args.body_imu.read_text()))
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(1, 'Cannot generate mounting candidate: %s\n' % error)
    candidate['source_result'] = str(args.result.resolve())
    print(yaml.safe_dump(candidate, sort_keys=False), end='')


if __name__ == '__main__':
    main()
