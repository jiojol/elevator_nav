#!/usr/bin/env python3

"""Evaluate start-to-end closure from a TUM trajectory.

The physical start and end poses must be known to coincide. Endpoint poses are
coordinate medians over fixed-duration endpoint windows or explicit time ranges
relative to the first trajectory sample. Path length is computed from one
coordinate-median sample per fixed-duration bin so high-rate stationary jitter
does not inflate the denominator.
"""

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import statistics
import sys
from typing import Dict, List, Optional, Sequence, Tuple


class EvaluationError(RuntimeError):
    """Invalid input or an unusable evaluation window."""


@dataclass(frozen=True)
class PoseSample:
    stamp: float
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class WindowMetrics:
    window_seconds: Optional[float]
    start_samples: int
    end_samples: int
    start_time: float
    end_time: float
    start_x: float
    start_y: float
    start_z: float
    start_yaw_deg: float
    end_x: float
    end_y: float
    end_z: float
    end_yaw_deg: float
    dx_m: float
    dy_m: float
    dz_m: float
    horizontal_error_m: float
    error_3d_m: float
    yaw_error_deg: float
    path_horizontal_m: float
    path_3d_m: float
    closure_horizontal_percent: float
    closure_3d_percent: float
    path_bin_seconds: float
    path_points: int


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Compute start/end closure and robust path length from a TUM "
            "trajectory, using fixed endpoint windows or explicit ranges."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("trajectory", type=Path, help="TUM file: t x y z qx qy qz qw")
    parser.add_argument(
        "--window-seconds",
        type=float,
        default=1.0,
        help=(
            "primary start and end median-window duration when explicit "
            "ranges are omitted"
        ),
    )
    parser.add_argument(
        "--start-range",
        type=float,
        nargs=2,
        metavar=("START", "END"),
        help=(
            "primary start-pose interval in seconds relative to the first TUM "
            "timestamp; requires --end-range"
        ),
    )
    parser.add_argument(
        "--end-range",
        type=float,
        nargs=2,
        metavar=("START", "END"),
        help=(
            "primary end-pose interval in seconds relative to the first TUM "
            "timestamp; requires --start-range"
        ),
    )
    parser.add_argument(
        "--verify-windows",
        type=float,
        nargs="*",
        default=[0.5, 1.0, 2.0, 5.0],
        metavar="SECONDS",
        help="additional endpoint windows to evaluate; pass with no values to disable",
    )
    parser.add_argument(
        "--path-bin-seconds",
        type=float,
        default=1.0,
        help="coordinate-median bin width for traveled path length",
    )
    parser.add_argument(
        "--minimum-window-samples",
        type=int,
        default=2,
        help="minimum samples required in each endpoint window",
    )
    parser.add_argument("--json", action="store_true", help="emit JSON")
    parser.add_argument(
        "--output",
        type=Path,
        help="also save the complete text or JSON report to this path",
    )
    return parser


def require_positive(name: str, value: float) -> None:
    if not math.isfinite(value) or value <= 0.0:
        raise EvaluationError("{} must be finite and > 0".format(name))


def validate_args(args: argparse.Namespace) -> None:
    require_positive("--window-seconds", args.window_seconds)
    require_positive("--path-bin-seconds", args.path_bin_seconds)
    if args.minimum_window_samples < 1:
        raise EvaluationError("--minimum-window-samples must be >= 1")
    for value in args.verify_windows:
        require_positive("--verify-windows", value)
    if (args.start_range is None) != (args.end_range is None):
        raise EvaluationError("--start-range and --end-range must be provided together")
    for name, interval in (
        ("--start-range", args.start_range),
        ("--end-range", args.end_range),
    ):
        if interval is None:
            continue
        begin, end = interval
        if not math.isfinite(begin) or not math.isfinite(end):
            raise EvaluationError("{} bounds must be finite".format(name))
        if begin < 0.0:
            raise EvaluationError("{} START must be >= 0".format(name))
        if end <= begin:
            raise EvaluationError("{} END must be greater than START".format(name))


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise EvaluationError("trajectory contains a zero or invalid quaternion")
    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def read_tum(path: Path) -> Tuple[List[PoseSample], int]:
    path = path.expanduser().resolve()
    if not path.is_file():
        raise EvaluationError("trajectory does not exist: {}".format(path))

    samples_by_stamp: Dict[float, PoseSample] = {}
    skipped = 0
    try:
        with path.open("r", encoding="utf-8") as stream:
            for line_number, raw_line in enumerate(stream, 1):
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                fields = line.split()
                if len(fields) < 8:
                    raise EvaluationError(
                        "{}:{} has {} fields; need at least 8".format(
                            path, line_number, len(fields)
                        )
                    )
                try:
                    values = [float(value) for value in fields[:8]]
                except ValueError as error:
                    raise EvaluationError(
                        "{}:{} contains a non-numeric value".format(path, line_number)
                    ) from error
                if not all(math.isfinite(value) for value in values):
                    skipped += 1
                    continue
                stamp, x, y, z, qx, qy, qz, qw = values
                yaw = quaternion_yaw(qx, qy, qz, qw)
                samples_by_stamp[stamp] = PoseSample(stamp, x, y, z, yaw)
    except OSError as error:
        raise EvaluationError("failed to read {}: {}".format(path, error)) from error

    samples = [samples_by_stamp[stamp] for stamp in sorted(samples_by_stamp)]
    if len(samples) < 2:
        raise EvaluationError("trajectory has fewer than two valid unique samples")
    if samples[-1].stamp <= samples[0].stamp:
        raise EvaluationError("trajectory has zero duration")
    return samples, skipped


def yaw_median(samples: Sequence[PoseSample]) -> float:
    reference = samples[0].yaw
    unwrapped = [reference + wrap_angle(sample.yaw - reference) for sample in samples]
    return wrap_angle(statistics.median(unwrapped))


def representative_pose(samples: Sequence[PoseSample]) -> PoseSample:
    if not samples:
        raise EvaluationError("cannot estimate a pose from an empty window")
    return PoseSample(
        stamp=statistics.median(sample.stamp for sample in samples),
        x=statistics.median(sample.x for sample in samples),
        y=statistics.median(sample.y for sample in samples),
        z=statistics.median(sample.z for sample in samples),
        yaw=yaw_median(samples),
    )


def endpoint_samples(
    samples: Sequence[PoseSample], window_seconds: float
) -> Tuple[List[PoseSample], List[PoseSample]]:
    first = samples[0].stamp
    last = samples[-1].stamp
    if 2.0 * window_seconds >= last - first:
        raise EvaluationError(
            "{:.3f}s endpoint windows overlap on a {:.3f}s trajectory".format(
                window_seconds, last - first
            )
        )
    start = [sample for sample in samples if sample.stamp <= first + window_seconds]
    end = [sample for sample in samples if sample.stamp >= last - window_seconds]
    return start, end


def relative_range_samples(
    samples: Sequence[PoseSample],
    start_range: Sequence[float],
    end_range: Sequence[float],
) -> Tuple[List[PoseSample], List[PoseSample], float, float]:
    origin = samples[0].stamp
    duration = samples[-1].stamp - origin
    start_begin, start_end = start_range
    end_begin, end_end = end_range
    if start_end >= end_begin:
        raise EvaluationError(
            "--start-range must end before --end-range begins"
        )
    if end_end > duration + 1.0e-9:
        raise EvaluationError(
            "--end-range ends at {:.3f}s but trajectory duration is {:.3f}s".format(
                end_end, duration
            )
        )

    start_begin_abs = origin + start_begin
    start_end_abs = origin + start_end
    end_begin_abs = origin + end_begin
    end_end_abs = origin + end_end
    start = [
        sample
        for sample in samples
        if start_begin_abs <= sample.stamp <= start_end_abs
    ]
    end = [
        sample
        for sample in samples
        if end_begin_abs <= sample.stamp <= end_end_abs
    ]
    return start, end, start_end_abs, end_begin_abs


def robust_path(
    samples: Sequence[PoseSample],
    start_pose: PoseSample,
    end_pose: PoseSample,
    start_cutoff: float,
    end_cutoff: float,
    bin_seconds: float,
) -> Tuple[float, float, int]:
    bins: Dict[int, List[PoseSample]] = {}
    for sample in samples:
        if not start_cutoff < sample.stamp < end_cutoff:
            continue
        index = int(math.floor((sample.stamp - start_cutoff) / bin_seconds))
        bins.setdefault(index, []).append(sample)

    points = [start_pose]
    points.extend(representative_pose(bins[index]) for index in sorted(bins))
    points.append(end_pose)
    path_horizontal = 0.0
    path_3d = 0.0
    for previous, current in zip(points, points[1:]):
        dx = current.x - previous.x
        dy = current.y - previous.y
        dz = current.z - previous.z
        path_horizontal += math.hypot(dx, dy)
        path_3d += math.sqrt(dx * dx + dy * dy + dz * dz)
    return path_horizontal, path_3d, len(points)


def calculate_metrics(
    samples: Sequence[PoseSample],
    start_samples: Sequence[PoseSample],
    end_samples: Sequence[PoseSample],
    start_path_cutoff: float,
    end_path_cutoff: float,
    path_bin_seconds: float,
    minimum_window_samples: int,
    selection_label: str,
    window_seconds: Optional[float],
) -> WindowMetrics:
    if len(start_samples) < minimum_window_samples:
        raise EvaluationError(
            "only {} samples in the {} start interval; need {}".format(
                len(start_samples), selection_label, minimum_window_samples
            )
        )
    if len(end_samples) < minimum_window_samples:
        raise EvaluationError(
            "only {} samples in the {} end interval; need {}".format(
                len(end_samples), selection_label, minimum_window_samples
            )
        )

    start_pose = representative_pose(start_samples)
    end_pose = representative_pose(end_samples)
    dx = end_pose.x - start_pose.x
    dy = end_pose.y - start_pose.y
    dz = end_pose.z - start_pose.z
    horizontal = math.hypot(dx, dy)
    error_3d = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw_error = wrap_angle(end_pose.yaw - start_pose.yaw)
    path_horizontal, path_3d, path_points = robust_path(
        samples,
        start_pose,
        end_pose,
        start_path_cutoff,
        end_path_cutoff,
        path_bin_seconds,
    )
    if path_horizontal <= 1.0e-12 or path_3d <= 1.0e-12:
        raise EvaluationError("trajectory has zero robust path length")

    return WindowMetrics(
        window_seconds=window_seconds,
        start_samples=len(start_samples),
        end_samples=len(end_samples),
        start_time=start_pose.stamp,
        end_time=end_pose.stamp,
        start_x=start_pose.x,
        start_y=start_pose.y,
        start_z=start_pose.z,
        start_yaw_deg=math.degrees(start_pose.yaw),
        end_x=end_pose.x,
        end_y=end_pose.y,
        end_z=end_pose.z,
        end_yaw_deg=math.degrees(end_pose.yaw),
        dx_m=dx,
        dy_m=dy,
        dz_m=dz,
        horizontal_error_m=horizontal,
        error_3d_m=error_3d,
        yaw_error_deg=math.degrees(yaw_error),
        path_horizontal_m=path_horizontal,
        path_3d_m=path_3d,
        closure_horizontal_percent=100.0 * horizontal / path_horizontal,
        closure_3d_percent=100.0 * error_3d / path_3d,
        path_bin_seconds=path_bin_seconds,
        path_points=path_points,
    )


def evaluate_window(
    samples: Sequence[PoseSample],
    window_seconds: float,
    path_bin_seconds: float,
    minimum_window_samples: int,
) -> WindowMetrics:
    start_samples, end_samples = endpoint_samples(samples, window_seconds)
    return calculate_metrics(
        samples,
        start_samples,
        end_samples,
        samples[0].stamp + window_seconds,
        samples[-1].stamp - window_seconds,
        path_bin_seconds,
        minimum_window_samples,
        "{:.3f}s endpoint-window".format(window_seconds),
        window_seconds,
    )


def evaluate_ranges(
    samples: Sequence[PoseSample],
    start_range: Sequence[float],
    end_range: Sequence[float],
    path_bin_seconds: float,
    minimum_window_samples: int,
) -> WindowMetrics:
    start_samples, end_samples, start_cutoff, end_cutoff = relative_range_samples(
        samples, start_range, end_range
    )
    return calculate_metrics(
        samples,
        start_samples,
        end_samples,
        start_cutoff,
        end_cutoff,
        path_bin_seconds,
        minimum_window_samples,
        "user-selected relative-range",
        None,
    )


def unique_windows(values: Sequence[float]) -> List[float]:
    result: List[float] = []
    for value in values:
        if not any(abs(value - existing) <= 1.0e-12 for existing in result):
            result.append(value)
    return result


def run(args: argparse.Namespace) -> Dict[str, object]:
    validate_args(args)
    samples, skipped = read_tum(args.trajectory)
    duration = samples[-1].stamp - samples[0].stamp
    if args.start_range is not None:
        primary = evaluate_ranges(
            samples,
            args.start_range,
            args.end_range,
            args.path_bin_seconds,
            args.minimum_window_samples,
        )
        primary_selection = {
            "mode": "relative_ranges",
            "time_origin": samples[0].stamp,
            "start_range_relative_seconds": list(args.start_range),
            "end_range_relative_seconds": list(args.end_range),
            "start_range_absolute_timestamps": [
                samples[0].stamp + value for value in args.start_range
            ],
            "end_range_absolute_timestamps": [
                samples[0].stamp + value for value in args.end_range
            ],
        }
    else:
        primary = evaluate_window(
            samples,
            args.window_seconds,
            args.path_bin_seconds,
            args.minimum_window_samples,
        )
        primary_selection = {
            "mode": "fixed_endpoint_windows",
            "time_origin": samples[0].stamp,
            "window_seconds": args.window_seconds,
            "start_range_relative_seconds": [0.0, args.window_seconds],
            "end_range_relative_seconds": [
                duration - args.window_seconds,
                duration,
            ],
        }

    verification: List[Dict[str, object]] = []
    for window in unique_windows(args.verify_windows):
        try:
            metrics = evaluate_window(
                samples,
                window,
                args.path_bin_seconds,
                args.minimum_window_samples,
            )
            verification.append(asdict(metrics))
        except EvaluationError as error:
            verification.append({"window_seconds": window, "error": str(error)})

    trajectory = args.trajectory.expanduser().resolve()
    return {
        "trajectory": str(trajectory),
        "assumption": "physical start and end poses coincide",
        "samples": len(samples),
        "skipped_nonfinite_samples": skipped,
        "start_timestamp": samples[0].stamp,
        "end_timestamp": samples[-1].stamp,
        "duration_seconds": duration,
        "primary_selection": primary_selection,
        "primary": asdict(primary),
        "verification_selection": {
            "mode": "fixed_endpoint_windows",
            "windows_seconds": unique_windows(args.verify_windows),
        },
        "verification": verification,
        "note": (
            "Closure is start/end self-consistency, not absolute accuracy "
            "without an external ground-truth trajectory."
        ),
    }


def format_metrics(
    metrics: Dict[str, object],
    selection: Dict[str, object],
    indent: str = "  ",
) -> List[str]:
    if selection["mode"] == "relative_ranges":
        first_line = (
            "{}ranges / samples : start={:+.3f}..{:+.3f} s  "
            "end={:+.3f}..{:+.3f} s / {} start, {} end"
        ).format(
            indent,
            selection["start_range_relative_seconds"][0],
            selection["start_range_relative_seconds"][1],
            selection["end_range_relative_seconds"][0],
            selection["end_range_relative_seconds"][1],
            metrics["start_samples"],
            metrics["end_samples"],
        )
    else:
        first_line = "{}window / samples : {:.3f} s / {} start, {} end".format(
            indent,
            selection["window_seconds"],
            metrics["start_samples"],
            metrics["end_samples"],
        )
    return [
        first_line,
        "{}start pose       : xyz=({:+.4f}, {:+.4f}, {:+.4f}) m  yaw={:+.3f} deg".format(
            indent,
            metrics["start_x"],
            metrics["start_y"],
            metrics["start_z"],
            metrics["start_yaw_deg"],
        ),
        "{}end pose         : xyz=({:+.4f}, {:+.4f}, {:+.4f}) m  yaw={:+.3f} deg".format(
            indent,
            metrics["end_x"],
            metrics["end_y"],
            metrics["end_z"],
            metrics["end_yaw_deg"],
        ),
        "{}delta xyz / z    : ({:+.4f}, {:+.4f}, {:+.4f}) m / {:+.4f} m".format(
            indent,
            metrics["dx_m"],
            metrics["dy_m"],
            metrics["dz_m"],
            metrics["dz_m"],
        ),
        "{}closure          : horizontal={:.4f} m  3D={:.4f} m  yaw={:+.3f} deg".format(
            indent,
            metrics["horizontal_error_m"],
            metrics["error_3d_m"],
            metrics["yaw_error_deg"],
        ),
        "{}robust path      : horizontal={:.3f} m  3D={:.3f} m  bins={:.3f} s".format(
            indent,
            metrics["path_horizontal_m"],
            metrics["path_3d_m"],
            metrics["path_bin_seconds"],
        ),
        "{}closure / path   : horizontal={:.4f}%  3D={:.4f}%".format(
            indent,
            metrics["closure_horizontal_percent"],
            metrics["closure_3d_percent"],
        ),
    ]


def text_report(result: Dict[str, object]) -> str:
    primary_selection = result["primary_selection"]
    endpoint_method = (
        "user-selected relative coordinate-median ranges"
        if primary_selection["mode"] == "relative_ranges"
        else "fixed start/end coordinate-median windows"
    )
    lines = [
        "TUM trajectory closure evaluation",
        "  trajectory        : {}".format(result["trajectory"]),
        "  valid samples     : {}".format(result["samples"]),
        "  duration          : {:.3f} s".format(result["duration_seconds"]),
        "  endpoint method   : {}".format(endpoint_method),
        "  path method       : fixed-time coordinate-median bins",
        "",
        "[Primary]",
    ]
    lines.extend(format_metrics(result["primary"], primary_selection))
    if result["verification"]:
        lines.extend(["", "[Window verification]"])
        for item in result["verification"]:
            if "error" in item:
                lines.append(
                    "  {:.3f} s: unavailable ({})".format(
                        item["window_seconds"], item["error"]
                    )
                )
            else:
                lines.append(
                    "  {:.3f} s: XY={:.4f} m  3D={:.4f} m  "
                    "yaw={:+.3f} deg  dz={:+.4f} m  n={}/{}".format(
                        item["window_seconds"],
                        item["horizontal_error_m"],
                        item["error_3d_m"],
                        item["yaw_error_deg"],
                        item["dz_m"],
                        item["start_samples"],
                        item["end_samples"],
                    )
                )
    lines.extend(["", "Note: {}".format(result["note"])])
    return "\n".join(lines)


def emit(report: str, output: Optional[Path]) -> None:
    print(report)
    if output is None:
        return
    output = output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        output.write_text(report + "\n", encoding="utf-8")
    except OSError as error:
        raise EvaluationError("failed to write {}: {}".format(output, error)) from error


def main() -> int:
    args = build_parser().parse_args()
    try:
        result = run(args)
        report = (
            json.dumps(result, indent=2, sort_keys=True, allow_nan=False)
            if args.json
            else text_report(result)
        )
        emit(report, args.output)
    except EvaluationError as error:
        print("error: {}".format(error), file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
