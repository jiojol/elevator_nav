"""Associate live returns with measured treads in a supplied navigation map."""
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'nav/src/map_for_plan/scripts'))
from plan_io import read_pcd_xyz


class StairSurface:
    def __init__(self, points, resolution):
        points = points[np.isfinite(points).all(axis=1)]
        cells, memberships, counts = np.unique(
            np.floor(points / resolution).astype(np.int64), axis=0,
            return_inverse=True, return_counts=True)
        heights = np.bincount(memberships, weights=points[:, 2]) / counts
        columns = {}
        for (ix, iy, _), height in zip(cells, heights):
            columns.setdefault((ix, iy), []).append(height)
        # Restrict suppression to traversable surfaces with a neighboring
        # step-height change. Flat floors and unobserved space keep all hits.
        treads = []
        for (ix, iy, _), height in zip(cells, heights):
            neighbors = ((ix, iy), (ix-1, iy), (ix+1, iy), (ix, iy-1), (ix, iy+1))
            treads.append(any(.09 <= abs(height-z) <= .28
                              for column in neighbors for z in columns.get(column, ())))
        self.points = points[np.asarray(treads, dtype=bool)[memberships]]
        self.tree = cKDTree(self.points)
        self.resolution = resolution

    @classmethod
    def from_meta(cls, meta_path):
        if not meta_path:
            return None
        metadata = Path(meta_path)
        meta = json.loads(metadata.read_text())
        source_name, source_hash = meta.get('source_pcd'), meta.get('source_sha256')
        if not source_name or not source_hash:
            return None
        source = metadata.parent / source_name
        if hashlib.sha256(source.read_bytes()).hexdigest() != source_hash:
            raise ValueError('traversable surface checksum does not match the planning map')
        resolution = float(meta['resolution'])
        if not np.isfinite(resolution) or resolution <= 0.0:
            raise ValueError('invalid planning voxel resolution')
        return cls(read_pcd_xyz(source), resolution)

    def tread_mask(self, points):
        finite = np.isfinite(points).all(axis=1)
        mask = np.zeros(len(points), dtype=bool)
        if not len(self.points) or not finite.any():
            return mask
        query = points[finite]
        distance, indices = self.tree.query(query, k=8, distance_upper_bound=.12)
        neighbors = self.points[np.minimum(indices, len(self.points)-1)]
        above = query[:, None, 2] - neighbors[:, :, 2]
        # Below a measured stair tread is terrain, including its riser/top
        # transition. The 12 cm 3D association radius already bounds that
        # downward range; the old extra 4.5 cm cut left riser hits occupied.
        # Retain points above the supported tread, allowing the observed
        # source/live sampling difference of at most 4.5 cm.
        matching = np.isfinite(distance) & (above <= .045)
        # Several source points at the measured tread height must support the
        # association. A point above the tread or outside its coverage stays.
        supported = np.count_nonzero(matching, axis=1) >= 2
        pending = np.flatnonzero(~supported)
        if len(pending):
            # A source cell can be sparsely sampled near its corner. Associate
            # within that same original XY cell and one vertical cell only.
            # The cell diagonal bounds the search; absent neighboring columns
            # are never filled and a different floor cannot match.
            d, ids = self.tree.query(query[pending], k=32,
                                     distance_upper_bound=np.sqrt(3.) * self.resolution)
            source = self.points[np.minimum(ids, len(self.points)-1)]
            cell = np.floor(query[pending, :2] / self.resolution).astype(np.int64)
            same_column = np.all(np.floor(source[:, :, :2] / self.resolution).astype(np.int64)
                                 == cell[:, None, :], axis=2)
            residual = query[pending, None, 2] - source[:, :, 2]
            in_layer = (residual >= -self.resolution) & (residual <= .045)
            supported[pending] = np.count_nonzero(np.isfinite(d) & same_column & in_layer, axis=1) >= 2
        mask[finite] = supported
        return mask

    def filter_records(self, data, height, width, row_step, point_step, fields, bigendian):
        byteorder = '>' if bigendian else '<'
        layout = np.dtype({
            'names': ['x', 'y', 'z'], 'formats': [byteorder+'f4']*3,
            'offsets': fields, 'itemsize': point_step,
        })
        shape, strides = (height, width), (row_step, point_step)
        points = np.ndarray(shape, dtype=layout, buffer=data, strides=strides)
        xyz = np.column_stack([points[axis].ravel() for axis in ('x', 'y', 'z')])
        keep = ~self.tread_mask(xyz)
        removed = len(keep) - np.count_nonzero(keep)
        if not removed:
            return data, 0
        records = np.ndarray(shape, dtype='V%d' % point_step, buffer=data, strides=strides)
        return records.ravel()[keep].tobytes(), int(removed)
