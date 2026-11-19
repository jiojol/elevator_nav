#!/usr/bin/env python3
"""Build a FREE planning map while removing Elevator-LIO transit intervals.

The input mode log must contain a timestamp column and an elevator flag column
(for example Elevator-LIO's elevator_debug.csv: timestamp,in_ele).

Normal-mode trajectory is split into independent segments before resampling and
inflation. This is intentional: removing only the elevator samples and then
resampling the remaining TUM would interpolate a new vertical corridor across
the shaft and reintroduce the exact connectivity this tool must prevent.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Iterable, List, Tuple

import numpy as np

_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

from tum_to_plan_map import inflate_continuous_layers, read_tum, resample, write_pcd


TRUE_VALUES = {"1", "true", "yes", "on", "elevator", "in_elevator"}
FALSE_VALUES = {"0", "false", "no", "off", "normal", "ordinary"}


def _parse_bool(value: str, path: Path, row_number: int) -> bool:
    normalized = value.strip().lower()
    if normalized in TRUE_VALUES:
        return True
    if normalized in FALSE_VALUES:
        return False
    raise SystemExit(
        "{}:{}: unsupported elevator flag value {!r}".format(path, row_number, value)
    )


def read_mode_log(path: Path,
                  timestamp_column: str = "",
                  mode_column: str = "") -> Tuple[np.ndarray, np.ndarray]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames:
            raise SystemExit("mode log has no CSV header: {}".format(path))
        fields = [name.strip() for name in reader.fieldnames]
        reader.fieldnames = fields
        if not timestamp_column:
            timestamp_column = next(
                (name for name in ("timestamp", "time", "stamp", "header.stamp") if name in fields),
                "",
            )
        if not mode_column:
            mode_column = next(
                (name for name in ("in_elevator", "in_ele", "elevator", "flag") if name in fields),
                "",
            )
        if not timestamp_column or timestamp_column not in fields:
            raise SystemExit("mode log needs a timestamp column: {}".format(path))
        if not mode_column or mode_column not in fields:
            raise SystemExit("mode log needs an elevator flag column: {}".format(path))

        times: List[float] = []
        states: List[bool] = []
        for row_number, row in enumerate(reader, 2):
            try:
                timestamp = float(row[timestamp_column])
                state = _parse_bool(row[mode_column], path, row_number)
            except (TypeError, ValueError):
                continue
            if math.isfinite(timestamp):
                times.append(timestamp)
                states.append(state)

    if not times:
        raise SystemExit("mode log has no valid samples: {}".format(path))
    order = np.argsort(np.asarray(times, dtype=np.float64), kind="mergesort")
    return np.asarray(times, dtype=np.float64)[order], np.asarray(states, dtype=bool)[order]


def elevator_state_at(times: np.ndarray, states: np.ndarray, query: float) -> bool:
    """Zero-order hold; queries before the first sample use the first state."""
    index = int(np.searchsorted(times, query, side="right")) - 1
    if index < 0:
        index = 0
    return bool(states[index])


def normal_segments(traj: np.ndarray, mode_times: np.ndarray,
                    mode_states: np.ndarray) -> List[np.ndarray]:
    segments: List[np.ndarray] = []
    current: List[np.ndarray] = []
    for row in traj:
        if elevator_state_at(mode_times, mode_states, float(row[0])):
            if current:
                segments.append(np.asarray(current, dtype=np.float64))
                current = []
        else:
            current.append(row)
    if current:
        segments.append(np.asarray(current, dtype=np.float64))
    return segments


def elevator_intervals(mode_times: np.ndarray,
                       mode_states: np.ndarray) -> List[List[float]]:
    intervals: List[List[float]] = []
    start = None
    for index, state in enumerate(mode_states):
        if state and start is None:
            start = float(mode_times[index])
        elif not state and start is not None:
            intervals.append([start, float(mode_times[index])])
            start = None
    if start is not None:
        intervals.append([start, float(mode_times[-1])])
    return intervals


def resample_segment(segment: np.ndarray, stride: float) -> np.ndarray:
    if len(segment) <= 1 or stride <= 0.0:
        return segment.copy()
    return resample(segment, stride)


def build_segmented_map(traj: np.ndarray,
                        mode_times: np.ndarray,
                        mode_states: np.ndarray,
                        resolution: float,
                        half_width: float,
                        pose_stride: float,
                        z_offset: float,
                        min_segment_poses: int) -> Tuple[np.ndarray, np.ndarray, dict]:
    raw = np.asarray(traj, dtype=np.float64).copy()
    raw[:, 3] += z_offset
    segments = normal_segments(raw, mode_times, mode_states)

    kept: List[np.ndarray] = []
    voxels = set()
    for segment in segments:
        if len(segment) < min_segment_poses:
            continue
        sampled = resample_segment(segment, pose_stride)
        if len(sampled) == 0:
            continue
        kept.append(sampled)
        voxels.update(inflate_continuous_layers(sampled[:, 1:4], resolution, half_width))

    if not kept:
        raise SystemExit("no normal-mode trajectory segment survived segmentation")
    if not voxels:
        raise SystemExit("segmented trajectory generated no FREE voxels")

    points = (np.asarray(sorted(voxels), dtype=np.float64) + 0.5) * resolution
    xyz = np.vstack(kept)[:, 1:4]
    path_length = sum(
        float(np.linalg.norm(np.diff(segment[:, 1:4], axis=0), axis=1).sum())
        for segment in kept
    )
    metadata = {
        "normal_segments": len(kept),
        "min_segment_poses": int(min_segment_poses),
        "raw_poses": int(len(raw)),
        "normal_poses": int(sum(len(segment) for segment in kept)),
        "path_length_m": path_length,
        "elevator_intervals": elevator_intervals(mode_times, mode_states),
        "free_voxels": int(points.shape[0]),
        "z_min": float(xyz[:, 2].min()),
        "z_max": float(xyz[:, 2].max()),
    }
    return points, kept, metadata


def write_tum_segments(path: Path, segments: Iterable[np.ndarray]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        stream.write("# timestamp tx ty tz qx qy qz qw\n")
        for index, segment in enumerate(segments):
            stream.write("# normal_segment {}\n".format(index))
            for t, x, y, z in segment:
                stream.write("{:.6f} {:.6f} {:.6f} {:.6f} 0 0 0 1\n".format(t, x, y, z))


def main() -> int:
    parser = argparse.ArgumentParser(description="从带电梯标签的轨迹生成分层 FREE 地图")
    parser.add_argument("--traj", type=Path, required=True, help="TUM: t x y z [qx qy qz qw]")
    parser.add_argument("--mode-log", type=Path, required=True, help="含时间戳和 in_elevator 的 CSV")
    parser.add_argument("--timestamp-column", default="")
    parser.add_argument("--mode-column", default="")
    parser.add_argument("--output-prefix", type=Path, default=None)
    parser.add_argument("--resolution", type=float, default=0.20)
    parser.add_argument("--half-width", type=float, default=0.5)
    parser.add_argument("--pose-stride", type=float, default=0.10)
    parser.add_argument("--z-offset", type=float, default=0.0)
    parser.add_argument("--min-segment-poses", type=int, default=3)
    args = parser.parse_args()

    if args.resolution <= 0.0 or args.half_width <= 0.0:
        raise SystemExit("resolution / half-width must be positive")
    if args.min_segment_poses < 1:
        raise SystemExit("min-segment-poses must be at least 1")

    traj = read_tum(args.traj)
    mode_times, mode_states = read_mode_log(
        args.mode_log, args.timestamp_column, args.mode_column
    )
    points, segments, summary = build_segmented_map(
        traj, mode_times, mode_states, args.resolution, args.half_width,
        args.pose_stride, args.z_offset, args.min_segment_poses
    )

    prefix = args.output_prefix or args.traj.expanduser().resolve().with_suffix("")
    prefix = prefix.expanduser().resolve()
    if prefix.suffix:
        prefix = prefix.with_suffix("")
    free_pcd = Path(str(prefix) + "_free.pcd")
    path_tum = Path(str(prefix) + "_path.tum")
    meta_path = Path(str(prefix) + "_plan.json")

    write_pcd(free_pcd, points)
    write_tum_segments(path_tum, segments)
    metadata = {
        "traj": str(args.traj.expanduser().resolve()),
        "mode_log": str(args.mode_log.expanduser().resolve()),
        "free_pcd": free_pcd.name,
        "path_tum": path_tum.name,
        "resolution": args.resolution,
        "half_width": args.half_width,
        "pose_stride": args.pose_stride,
        "z_offset": args.z_offset,
    }
    metadata.update(summary)
    meta_path.write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(
        "segmented plan map: {} voxels, {} normal segments, z {:.2f}..{:.2f}\n  {}\n  {}".format(
            points.shape[0], summary["normal_segments"], summary["z_min"],
            summary["z_max"], free_pcd, meta_path
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
