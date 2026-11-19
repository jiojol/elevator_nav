#!/usr/bin/env python3
"""Regression checks for elevator-aware TUM segmentation."""
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "nav/src/map_for_plan/scripts"))

from plan_io import read_pcd_xyz
from segmented_tum_to_plan_map import (
    build_segmented_map,
    elevator_intervals,
    elevator_state_at,
    read_mode_log,
)


class SegmentedTumToPlanMapTest(unittest.TestCase):
    def test_mode_log_accepts_elevator_debug_columns(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "elevator_debug.csv"
            path.write_text(
                "timestamp,in_ele\n"
                "10.0,0\n"
                "11.0,1\n"
                "12.0,1\n"
                "13.0,0\n",
                encoding="utf-8",
            )
            times, states = read_mode_log(path)
            np.testing.assert_allclose(times, [10, 11, 12, 13])
            self.assertFalse(elevator_state_at(times, states, 10.5))
            self.assertTrue(elevator_state_at(times, states, 11.5))
            self.assertFalse(elevator_state_at(times, states, 13.0))
            self.assertEqual(elevator_intervals(times, states), [[11.0, 13.0]])

    def test_elevator_section_does_not_create_vertical_free_space(self):
        trajectory = np.array([
            [0.0, 0.0, 0.0, 0.0],
            [1.0, 1.0, 0.0, 0.0],
            [2.0, 2.0, 0.0, 0.0],
            [3.0, 2.0, 0.0, 1.0],
            [4.0, 2.0, 0.0, 2.0],
            [5.0, 2.0, 0.0, 3.0],
            [6.0, 2.0, 0.0, 4.0],
            [7.0, 3.0, 0.0, 4.0],
            [8.0, 4.0, 0.0, 4.0],
        ])
        mode_times = np.array([0.0, 3.0, 6.0, 8.0])
        mode_states = np.array([False, True, False, False])
        points, segments, metadata = build_segmented_map(
            trajectory,
            mode_times,
            mode_states,
            resolution=0.2,
            half_width=0.5,
            pose_stride=0.1,
            z_offset=0.0,
            min_segment_poses=2,
        )
        self.assertEqual(metadata["normal_segments"], 2)
        self.assertEqual(metadata["elevator_intervals"], [[3.0, 6.0]])
        z = points[:, 2]
        self.assertGreaterEqual(z.min(), -0.1)
        self.assertLessEqual(z.max(), 4.11)
        self.assertFalse(np.any((z > 0.4) & (z < 3.6)))
        self.assertGreater(np.count_nonzero(z < 0.5), 1)
        self.assertGreater(np.count_nonzero(z > 3.5), 1)
        self.assertEqual(len(segments), 2)

    def test_cli_output_round_trips_pcd_and_metadata(self):
        with tempfile.TemporaryDirectory() as temp:
            temp = Path(temp)
            tum = temp / "trajectory.tum"
            tum.write_text(
                "# t x y z qx qy qz qw\n"
                "0 0 0 0 0 0 0 1\n"
                "1 1 0 0 0 0 0 1\n"
                "2 1 0 2 0 0 0 1\n"
                "3 2 0 4 0 0 0 1\n"
                "4 3 0 4 0 0 0 1\n",
                encoding="utf-8",
            )
            mode = temp / "mode.csv"
            mode.write_text(
                "timestamp,in_elevator\n0,false\n1.5,true\n3.0,false\n4,false\n",
                encoding="utf-8",
            )
            prefix = temp / "floor_map"
            import subprocess
            subprocess.run([
                sys.executable,
                str(ROOT / "nav/src/map_for_plan/scripts/segmented_tum_to_plan_map.py"),
                "--traj", str(tum),
                "--mode-log", str(mode),
                "--output-prefix", str(prefix),
                "--min-segment-poses", "2",
            ], check=True, capture_output=True, text=True)
            meta = json.loads((temp / "floor_map_plan.json").read_text(encoding="utf-8"))
            points = read_pcd_xyz(temp / "floor_map_free.pcd")
            self.assertEqual(meta["normal_segments"], 2)
            self.assertGreater(len(points), 0)
            self.assertFalse(np.any((points[:, 2] > 0.4) & (points[:, 2] < 3.6)))


if __name__ == "__main__":
    unittest.main()
