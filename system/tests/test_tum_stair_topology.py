#!/usr/bin/env python3
"""Offline regressions for TUM stair connectivity; no ROS or robot required."""

from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "nav/src/map_for_plan/scripts"))

from plan_graph import FreeGraph
from plan_io import load_plan_meta, read_pcd_xyz, read_tum_xyz
from tum_to_plan_map import inflate_continuous_layers


def corridor(line):
    points = (np.asarray(sorted(inflate_continuous_layers(line, 0.2, 1.0))) + 0.5) * 0.2
    return FreeGraph(points, 0.2, centerline=line)


class StairTopologyTest(unittest.TestCase):
    def test_reference_height_recovers_subvoxel_demonstrated_surface(self):
        points = np.array([[.1, .1, .1], [.3, .1, .3], [.5, .1, .3]])
        line = np.array([[.1, .1, .18], [.5, .1, .38]])
        graph = FreeGraph(points, .2, centerline=line)
        original = graph.points.copy()
        reference = graph.reference_heights(points)
        np.testing.assert_array_equal(reference[:, :2], points[:, :2])
        np.testing.assert_array_equal(graph.points, original)
        self.assertTrue(np.any(np.abs(reference[:, 2] - points[:, 2]) > .001))
        self.assertLessEqual(np.max(np.abs(reference[:, 2] - points[:, 2])), .1 + 1.e-6)
        self.assertTrue(np.all(np.diff(reference[:, 2]) > 0))

    def test_reference_height_preserves_large_disagreement_and_unmodeled_graph(self):
        points = np.array([[.1, .1, .1], [.3, .1, .1]])
        line = points + [0, 0, .4]
        np.testing.assert_array_equal(FreeGraph(points, .2, centerline=line).reference_heights(points), points)
        np.testing.assert_array_equal(FreeGraph(points, .2).reference_heights(points), points)

    def test_hairpin_uses_landing_in_both_directions(self):
        for height in (0.1, 0.5):
            with self.subTest(return_height=height):
                line = np.array([[0.1, 0.1, 0.1], [6.1, 0.1, 0.1],
                                 [6.1, 1.3, height], [0.1, 1.3, height]])
                graph = corridor(line)
                start = graph.nearest(line[0], 1.0)
                goal = graph.nearest(line[-1], 1.0)
                for a, b in ((start, goal), (goal, start)):
                    path = graph.astar(a, b)
                    self.assertIsNotNone(path)
                    self.assertGreaterEqual(path[:, 0].max(), 5.7)
                    self.assertLess(np.linalg.norm(path - line[2], axis=1).min(), 0.6)

    def test_overlapping_floors_do_not_connect_vertically(self):
        line = np.array([[0.1, 0.1, 0.1], [6.1, 0.1, 0.1],
                         [6.1, 1.3, 0.3], [0.1, 1.3, 0.3], [0.1, 0.1, 0.3]])
        graph = corridor(line)
        path = graph.astar(graph.nearest(line[0], 1.0), graph.nearest(line[-1], 1.0))
        self.assertIsNotNone(path)
        self.assertGreaterEqual(path[:, 0].max(), 5.7)

    def test_retraced_ground_route_stays_connected(self):
        line = np.array([[0.1, 0.1, 0.1], [6.1, 0.1, 0.1],
                         [0.1, 0.1, 0.1], [0.1, 6.1, 0.1]])
        graph = corridor(line)
        start = graph.nearest([5.1, 0.1, 0.1], 1.0)
        goal = graph.nearest([0.1, 5.1, 0.1], 1.0)
        self.assertIsNotNone(graph.astar(start, goal))
        self.assertIsNotNone(graph.astar(goal, start))

    def test_offset_return_visit_does_not_force_backtracking(self):
        line = np.array([[0.1, 0.1, 0.1], [10.1, 0.1, 0.1],
                         [10.1, -6.1, 0.1], [10.5, -6.1, 0.1],
                         [10.5, 6.1, 0.1]])
        graph = corridor(line)
        start = graph.nearest([6.1, 0.1, 0.1], 1.0)
        goal = graph.nearest(line[-1], 1.0)
        for a, b in ((start, goal), (goal, start)):
            path = graph.astar(a, b)
            self.assertIsNotNone(path)
            self.assertGreaterEqual(path[:, 1].min(), -0.1)
            self.assertLess(np.linalg.norm(np.diff(path, axis=0), axis=1).sum(), 11.0)

    def test_diagonal_requires_support_at_matching_height(self):
        for support in ([], [[0.3, 0.1, 2.1], [0.1, 0.3, 2.1]]):
            with self.subTest(support=support):
                points = np.array([[0.1, 0.1, 0.1], [0.3, 0.3, 0.1]] + support)
                self.assertIsNone(FreeGraph(points, 0.2).astar(0, 1))
        points = np.array([[0.1, 0.1, 0.1], [0.3, 0.3, 0.1],
                           [0.3, 0.1, 0.1], [0.1, 0.3, 0.1]])
        self.assertEqual(len(FreeGraph(points, 0.2).astar(0, 1)), 2)

    def test_vertical_edges_obey_max_step(self):
        points = np.array([[0.1, 0.1, 0.1], [0.1, 0.1, 0.3]])
        self.assertIsNone(FreeGraph(points, 0.2, max_step=0.1).astar(0, 1))

    def test_projection_handles_duplicate_and_sparse_samples(self):
        sparse = np.array([[0.1, 0.1, 0.1], [6.1, 0.1, 0.1],
                           [6.1, 1.3, 0.5], [0.1, 1.3, 0.5]])
        dense = np.concatenate([a + np.linspace(0, 1, 41)[:, None] * (b - a)
                                for a, b in zip(sparse[:-1], sparse[1:])])
        graph = corridor(sparse)
        resampled = FreeGraph(graph.points, 0.2, centerline=dense)
        np.testing.assert_allclose(graph.center_arc, resampled.center_arc, atol=1e-10)
        np.testing.assert_allclose(graph.center_projection, resampled.center_projection, atol=1e-10)

    def test_recorded_map_route_remains_connected(self):
        meta_path = ROOT / "system/map/dog_plan.json"
        if not meta_path.is_file():
            self.skipTest("recorded map is not installed")
        meta = load_plan_meta(meta_path)
        line = read_tum_xyz(Path(meta["path_tum"]))
        graph = FreeGraph(read_pcd_xyz(Path(meta["free_pcd"])),
                          meta["resolution"], centerline=line)
        snaps = {graph.nearest(point, 2.0) for point in line}
        start = graph.nearest(line[0], 2.0)
        reverse = [[] for _ in graph.points]
        for i, neighbors in enumerate(graph.adj):
            for j, _ in neighbors:
                reverse[j].append((i, 0.0))
        for adjacency in (graph.adj, reverse):
            reached, pending = {start}, [start]
            while pending:
                for neighbor, _ in adjacency[pending.pop()]:
                    if neighbor not in reached:
                        reached.add(neighbor)
                        pending.append(neighbor)
            self.assertTrue(snaps <= reached, "recorded route disconnected")


if __name__ == "__main__":
    unittest.main()
