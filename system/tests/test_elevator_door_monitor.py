#!/usr/bin/env python3
"""Pure geometry tests for the elevator door ROI detector."""
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "nav/src/dog_nav_bringup/scripts"))
from elevator_door_monitor import evaluate_door_points


class ElevatorDoorMonitorGeometryTest(unittest.TestCase):
    def setUp(self):
        self.lobby = (2.0, 0.0, 0.0)
        self.inside = (1.0, 0.0, 0.0)

    def metrics(self, points, from_cabin=False):
        return evaluate_door_points(
            np.asarray(points, dtype=float), self.lobby, self.inside,
            floor_z=0.0, door_width=1.0, door_height=2.2,
            plane_thickness=0.16, cell_size=0.1, from_cabin=from_cabin,
        )

    def test_dense_door_plane_is_closed_candidate(self):
        points = [
            [1.5, y, z]
            for y in np.arange(-0.45, 0.46, 0.10)
            for z in np.arange(0.10, 2.11, 0.10)
        ]
        result = self.metrics(points)
        self.assertGreater(result["occupancy_ratio"], 0.18)
        self.assertGreater(result["plane_points"], 150)

    def test_clear_plane_with_cabin_returns_is_open_candidate(self):
        points = [
            [0.55 + 0.15 * (index % 4), -0.45 + 0.10 * (index % 10), 0.20 + 0.12 * (index // 10)]
            for index in range(120)
        ]
        result = self.metrics(points)
        self.assertLess(result["occupancy_ratio"], 0.06)
        self.assertGreaterEqual(result["inside_points"], 15)

    def test_cabin_view_requires_returns_on_lobby_side(self):
        points = [[2.1, y, z] for y in np.arange(-0.4, 0.4, 0.1)
                  for z in np.arange(0.2, 1.2, 0.1)]
        result = self.metrics(points, from_cabin=True)
        self.assertEqual(result["plane_points"], 0)
        self.assertGreaterEqual(result["inside_points"], 15)
        self.assertEqual(self.metrics(points)["inside_points"], 0)

    def test_explicit_door_plane_stays_fixed_when_lobby_moves_back(self):
        points = np.array([[1.5, y, z] for y in np.arange(-.45, .46, .1)
                           for z in np.arange(.10, 2.11, .1)])
        fixed = evaluate_door_points(points, (3.0, 0.0, 0.0), self.inside,
                                     0.0, 1.0, 2.2, .16, .1,
                                     door_center_xy=(1.5, 0.0))
        self.assertEqual(fixed, self.metrics(points))

    def test_points_outside_door_roi_do_not_count(self):
        points = np.array([[1.5, 2.0, 1.0], [1.5, -2.0, 1.0], [0.0, 0.0, 3.5]])
        result = self.metrics(points)
        self.assertEqual(result["plane_points"], 0)
        self.assertEqual(result["inside_points"], 0)


if __name__ == "__main__":
    unittest.main()
