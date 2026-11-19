#!/usr/bin/env python3
"""单层 FREE 体素上的快速 A*（偏好走廊中央）。"""

from __future__ import annotations

import heapq
import math
from collections import deque
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

Key = Tuple[int, int, int]
Neighbor = Tuple[int, float]

XY_DIRS = (
    (1, 0, 1.0),
    (-1, 0, 1.0),
    (0, 1, 1.0),
    (0, -1, 1.0),
    (1, 1, math.sqrt(2.0)),
    (1, -1, math.sqrt(2.0)),
    (-1, 1, math.sqrt(2.0)),
    (-1, -1, math.sqrt(2.0)),
)
CARDINALS = ((1, 0), (-1, 0), (0, 1), (0, -1))


def trim_vertical_goal_tail(path: np.ndarray, resolution: float) -> np.ndarray:
    """Use the connected approach surface for a one-voxel vertical goal tail."""
    if len(path) < 3:
        return path
    first = len(path) - 1
    while first > 0 and np.linalg.norm(path[first - 1, :2] - path[-1, :2]) < 1.e-6:
        first -= 1
    if first == 0 or first == len(path) - 1:
        return path
    tail_z = path[first:, 2]
    approach = path[first] - path[first - 1]
    # Do not alter stairs, long vertical chains, or a path consisting only of
    # height changes. The replacement is an existing FREE voxel, not a new point.
    if (float(np.ptp(tail_z)) > resolution + 1.e-6
            or abs(float(approach[2])) > 1.e-6
            or np.linalg.norm(approach[:2]) < .5 * resolution):
        return path
    return path[:first + 1].copy()


class FreeGraph:
    def __init__(
        self,
        points: np.ndarray,
        resolution: float,
        max_step: float = 0.80,
        center_weight: float = 6.0,
        centerline: Optional[np.ndarray] = None,
        half_width: float = 1.0,
    ) -> None:
        if resolution <= 0.0:
            raise ValueError("resolution 必须为正")
        self.points = np.asarray(points, dtype=np.float64)
        self.resolution = float(resolution)
        self.max_step = float(max_step)
        self.center_weight = max(0.0, float(center_weight))
        self.revisit_distance = max(1.5 * self.resolution, float(half_width))
        self.keys: List[Key] = []
        self.index: Dict[Key, int] = {}
        self.columns: Dict[Tuple[int, int], List[int]] = {}
        for i, (x, y, z) in enumerate(self.points):
            key = (
                int(math.floor(x / self.resolution)),
                int(math.floor(y / self.resolution)),
                int(math.floor(z / self.resolution)),
            )
            self.keys.append(key)
            self.index[key] = i
            self.columns.setdefault(key[:2], []).append(i)
        self.clearance = self._clearance_cells()
        self.center_projection, self.center_arc, self.center_tangent = self._project_centerline(centerline)
        self.center_pen = self._center_penalty(centerline)
        self.adj: List[List[Neighbor]] = [[] for _ in range(len(self.points))]
        self._build_edges()

    def _clearance_cells(self) -> np.ndarray:
        """到走廊边界的格子距离：0 在边，越大越居中。"""
        xy_keys = list(self.columns)
        xy_index = {key: i for i, key in enumerate(xy_keys)}
        dist = np.full(len(xy_keys), -1, dtype=np.int32)
        queue: deque = deque()
        for i, (ix, iy) in enumerate(xy_keys):
            if any((ix + dx, iy + dy) not in xy_index for dx, dy in CARDINALS):
                dist[i] = 0
                queue.append(i)
        while queue:
            i = queue.popleft()
            ix, iy = xy_keys[i]
            for dx, dy in CARDINALS:
                j = xy_index.get((ix + dx, iy + dy))
                if j is None or dist[j] >= 0:
                    continue
                dist[j] = dist[i] + 1
                queue.append(j)
        dist[dist < 0] = 0
        by_xy = {key: float(dist[i]) for i, key in enumerate(xy_keys)}
        return np.asarray([by_xy[key[:2]] for key in self.keys], dtype=np.float64)

    def _project_centerline(self, centerline: Optional[np.ndarray]):
        """Project in 3D and retain route distance to distinguish nearby flights."""
        if centerline is None or len(centerline) < 2:
            return None, None, None
        line = np.asarray(centerline, dtype=np.float64)
        vectors = np.diff(line, axis=0)
        lengths = np.linalg.norm(vectors, axis=1)
        arc = np.r_[0.0, np.cumsum(lengths)]
        projected = np.empty_like(self.points)
        progress = np.empty(len(self.points), dtype=np.float64)
        tangents = np.empty_like(self.points)
        for begin in range(0, len(self.points), 256):
            query = self.points[begin : begin + 256]
            delta = query[:, None, :] - line[None, :-1, :]
            fraction = np.clip(
                np.einsum("ijk,jk->ij", delta, vectors)
                / np.maximum(lengths * lengths, 1.0e-20), 0.0, 1.0
            )
            feet = line[None, :-1, :] + fraction[:, :, None] * vectors
            distance = np.sum((query[:, None, :] - feet) ** 2, axis=2)
            segment = np.argmin(distance, axis=1)
            rows = np.arange(len(query))
            projected[begin : begin + len(query)] = feet[rows, segment]
            progress[begin : begin + len(query)] = (
                arc[segment] + fraction[rows, segment] * lengths[segment]
            )
            tangents[begin : begin + len(query)] = (
                vectors[segment] / np.maximum(lengths[segment, None], 1.0e-20)
            )
        return projected, progress, tangents

    def _center_penalty(self, centerline: Optional[np.ndarray]) -> np.ndarray:
        """0=最居中，1=贴边。净空 + 到中线距离。"""
        cmax = float(self.clearance.max()) if len(self.clearance) else 0.0
        if cmax <= 0.0:
            edge = np.zeros(len(self.points), dtype=np.float64)
        else:
            edge = 1.0 - self.clearance / cmax

        if centerline is None or len(centerline) == 0:
            return edge

        if self.center_projection is None:
            lateral = np.linalg.norm(self.points - np.asarray(centerline)[0], axis=1)
        else:
            lateral = np.linalg.norm(self.points - self.center_projection, axis=1)
        lmax = max(float(lateral.max()), self.resolution)
        return np.clip(0.35 * edge + 0.65 * (lateral / lmax), 0.0, 1.0)

    def _can_connect(self, i: int, j: int) -> bool:
        if abs(float(self.points[i, 2] - self.points[j, 2])) > self.max_step:
            return False
        if self.center_arc is None:
            return True
        length = float(np.linalg.norm(self.points[i] - self.points[j]))
        if abs(self.center_arc[i] - self.center_arc[j]) <= max(
            3.0 * self.resolution, 2.0 * length
        ):
            return True
        # Return visits can be offset within the recorded corridor. A voxel-sized
        # merge radius creates a seam that forces a detour to a distant junction.
        # Merge aligned visits within the corridor width. Keep height tolerance
        # at half a voxel so different stair flights remain separate; unrelated
        # corner segments still require the original route continuity check.
        delta = self.center_projection[i] - self.center_projection[j]
        if abs(float(delta[2])) > 0.5 * self.resolution:
            return False
        distance = float(np.linalg.norm(delta[:2]))
        if distance <= 1.5 * self.resolution:
            return True
        alignment = abs(float(np.dot(self.center_tangent[i], self.center_tangent[j])))
        return distance <= self.revisit_distance and alignment >= 0.5

    def _has_corner_support(self, i: int, j: int, xy: Tuple[int, int]) -> bool:
        return any(self._can_connect(i, k) and self._can_connect(k, j)
                   for k in self.columns.get(xy, ()))

    def _build_edges(self) -> None:
        for i, (ix, iy, iz) in enumerate(self.keys):
            z0 = self.points[i, 2]
            for dx, dy, horiz in XY_DIRS:
                candidates = self.columns.get((ix + dx, iy + dy), ())
                if not candidates:
                    continue
                candidates = [j for j in candidates if self._can_connect(i, j)]
                if not candidates:
                    continue
                j = min(candidates, key=lambda candidate: abs(float(self.points[candidate, 2] - z0)))
                if dx and dy and not (
                    self._has_corner_support(i, j, (ix + dx, iy))
                    and self._has_corner_support(i, j, (ix, iy + dy))
                ):
                    continue
                dz = abs(float(self.points[j, 2] - z0))
                if dz > self.max_step:
                    continue
                geom = math.hypot(horiz * self.resolution, dz)
                mid = 0.5 * (self.center_pen[i] + self.center_pen[j])
                cost = geom * (1.0 + self.center_weight * mid * mid)
                self.adj[i].append((j, cost))
            for dz_key in (-1, 1):
                j = self.index.get((ix, iy, iz + dz_key))
                if j is not None and self._can_connect(i, j):
                    self.adj[i].append((j, self.resolution))

    def nearest(self, xyz: Sequence[float], max_dist: float) -> Optional[int]:
        """吸附到距离最近的 FREE 体素，包含轨迹两侧扩展出来的格子。"""
        query = np.asarray(xyz, dtype=np.float64)
        delta = self.points - query
        dxy2 = delta[:, 0] ** 2 + delta[:, 1] ** 2
        nearby = np.flatnonzero(dxy2 <= max_dist * max_dist)
        if nearby.size == 0:
            return None
        score = dxy2[nearby] + delta[nearby, 2] ** 2
        return int(nearby[int(np.argmin(score))])

    def reference_heights(self, path: np.ndarray) -> np.ndarray:
        """Recover demonstrated Z without changing graph connectivity or XY."""
        reference = path.copy()
        if self.center_projection is None:
            return reference
        for row, point in enumerate(path):
            key = tuple(np.floor(point / self.resolution).astype(int))
            index = self.index.get(key)
            if index is None:
                continue
            height = float(self.center_projection[index, 2])
            # One surface voxel has half a cell of quantization error. Larger
            # disagreements are not quantization and must retain the graph Z.
            if abs(height - point[2]) <= .5 * self.resolution + 1.e-6:
                reference[row, 2] = height
        return reference

    def astar(self, start: int, goal: int) -> Optional[np.ndarray]:
        if start == goal:
            return self.points[[start]]
        goal_xyz = self.points[goal]
        open_heap = [(0.0, 0.0, start)]
        came: Dict[int, int] = {}
        best = {start: 0.0}
        closed = set()
        while open_heap:
            _f, g, node = heapq.heappop(open_heap)
            if node in closed:
                continue
            if node == goal:
                return self._rebuild(came, node)
            closed.add(node)
            for nxt, cost in self.adj[node]:
                if nxt in closed:
                    continue
                tentative = g + cost
                if tentative >= best.get(nxt, 1.0e300):
                    continue
                came[nxt] = node
                best[nxt] = tentative
                remain = float(np.linalg.norm(self.points[nxt] - goal_xyz))
                heapq.heappush(open_heap, (tentative + remain, tentative, nxt))
        return None

    def _rebuild(self, came: Dict[int, int], node: int) -> np.ndarray:
        chain = [node]
        while node in came:
            node = came[node]
            chain.append(node)
        chain.reverse()
        return self.points[np.asarray(chain, dtype=np.int64)]
