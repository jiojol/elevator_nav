#!/usr/bin/env python3
"""Check stock Go2 IMU geometry and LI-Init mounting composition offline."""
import copy
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'system/scripts'))
from lidar_mount_candidate import compose_candidate


def record(matrix, refined=True):
    label = 'Refinement result:' if refined else 'Initialization result:'
    return label + '\nHomogeneous Transformation Matrix from LiDAR to IMU: \n' + '\n'.join(
        ' '.join(str(value) for value in row) for row in matrix) + '\n'


class MountCandidateTest(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load((ROOT / 'nav/src/dog_nav_bringup/config/go2_body_imu.yaml').read_text())

    def test_geometry_matches_go2_urdf(self):
        urdf = ET.parse(ROOT / 'third_party/go2_geometry.urdf')
        joint = urdf.find("joint[@name='imu_joint']")
        self.assertEqual(joint.find('parent').get('link'), 'base')
        self.assertEqual(joint.find('child').get('link'), 'imu')
        origin = joint.find('origin')
        np.testing.assert_allclose(self.config['translation'], np.fromstring(origin.get('xyz'), sep=' '))
        expected = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy'), sep=' ')).as_matrix()
        np.testing.assert_allclose(Rotation.from_quat(self.config['rotation_xyzw']).as_matrix(), expected)
        self.assertGreater(np.linalg.norm(self.config['translation']), .04)

    def test_refined_result_includes_body_offset_not_inverse(self):
        imu_lidar = np.eye(4)
        imu_lidar[:3, :3] = Rotation.from_euler('z', 90, degrees=True).as_matrix()
        imu_lidar[:3, 3] = [.30, -.02, .15]
        text = record(np.eye(4), refined=False) + record(imu_lidar)
        out = compose_candidate(text, self.config)
        np.testing.assert_allclose(out['translation'], [.27443, -.02, .19232])
        np.testing.assert_allclose(np.array(out['matrix'])[:3, :3], imu_lidar[:3, :3])
        self.assertEqual(out['status'], 'unvalidated_candidate')
        self.assertEqual(out['calibration_imu_topic'], '/go2/imu')

    def test_nonidentity_body_rotation_uses_correct_order(self):
        config = copy.deepcopy(self.config)
        config['rotation_xyzw'] = Rotation.from_euler('z', 90, degrees=True).as_quat().tolist()
        imu_lidar = np.eye(4)
        imu_lidar[:3, :3] = Rotation.from_euler('x', 30, degrees=True).as_matrix()
        imu_lidar[:3, 3] = [.3, .1, .2]
        out = compose_candidate(record(imu_lidar), config)
        point = np.array([.4, -.2, .7, 1.])
        imu_point = (imu_lidar @ point)[:3]
        expected = Rotation.from_quat(config['rotation_xyzw']).apply(imu_point) + config['translation']
        np.testing.assert_allclose((np.array(out['matrix']) @ point)[:3], expected)

    def test_missing_or_invalid_refinement_is_rejected(self):
        reflected, nonfinite = np.eye(4), np.eye(4)
        reflected[0, 0] = -1
        nonfinite[0, 3] = np.nan
        for text in [record(np.eye(4), refined=False), 'Refinement result:\n',
                     record(reflected), record(nonfinite), record(np.eye(4))[:-12]]:
            with self.subTest(text=text), self.assertRaises(ValueError):
                compose_candidate(text, self.config)


if __name__ == '__main__':
    unittest.main()
