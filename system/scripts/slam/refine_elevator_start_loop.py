#!/usr/bin/env python3
"""Refine a surveyed start-area revisit and rebuild the Elevator-LIO map offline."""
import argparse
import json
import shutil
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import rosbag
import rospy
import yaml
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp


LIVOX_MD5 = "e4d6829bdfe657cb6c21a746c86b21a6"
LIVOX_POINT = np.dtype([
    ("dt", "<u4"), ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
    ("reflectivity", "u1"), ("tag", "u1"), ("line", "u1"),
])


def voxel_cloud(cloud, resolution):
    keys = np.floor(cloud[:, :3] / resolution).astype(np.int32)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return cloud[indices]


def livox_scan(raw):
    if raw[2] != LIVOX_MD5:
        raise ValueError("Unsupported ROS 1 Livox CustomMsg layout")
    data = raw[1]
    _, seconds, nanoseconds, frame_len = struct.unpack_from("<IIII", data)
    point_count = struct.unpack_from("<I", data, 32 + frame_len)[0]
    fields = np.frombuffer(data, dtype=LIVOX_POINT, count=point_count, offset=36 + frame_len)
    stamp = seconds + nanoseconds * 1e-9
    duration = float(fields["dt"][-1]) * 1e-9 if point_count else 0.
    return stamp, duration, fields


class Trajectory:
    def __init__(self, tum, sensor_yaml):
        self.rows = np.loadtxt(tum, ndmin=2)
        if (self.rows.shape[1] != 8 or len(self.rows) < 3 or not np.isfinite(self.rows).all()
                or np.any(np.diff(self.rows[:, 0]) <= 0.)):
            raise ValueError("Expected finite, ordered TUM poses with quaternions")
        self.start = self.rows[0, 0]
        self.times = self.rows[:, 0] - self.start
        self.positions = self.rows[:, 1:4]
        self.rotations = Rotation.from_quat(self.rows[:, 4:8])
        self.interpolate_rotation = Slerp(self.times, self.rotations)
        offset = yaml.safe_load(sensor_yaml.read_text())["offset"]
        from_matrix = getattr(Rotation, "from_matrix", None) or Rotation.from_dcm
        self.imu_from_lidar = from_matrix(offset["imu_R_lidar"])
        self.lidar_position = np.asarray(offset["imu_t_lidar"], dtype=float)
        self.arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(self.positions[:, :2], axis=0), axis=1))]

    def project(self, fields, timestamp, max_range):
        points = np.column_stack([fields[name] for name in ("x", "y", "z")])
        range2 = np.einsum("ij,ij->i", points, points)
        valid = ((fields["line"] < 6) & ((fields["tag"] & 0x20) == 0)
                 & (range2 > .8**2) & (range2 < max_range**2))
        times = np.clip(timestamp + fields["dt"][valid] * 1e-9, self.times[0], self.times[-1])
        if not len(times):
            return np.empty((0, 3)), times, fields["reflectivity"][valid]
        position = np.column_stack([np.interp(times, self.times, axis) for axis in self.positions.T])
        imu_points = self.imu_from_lidar.apply(points[valid]) + self.lidar_position
        world = self.interpolate_rotation(times).apply(imu_points) + position
        return world, times, fields["reflectivity"][valid]


def find_revisit(trajectory, ride_intervals):
    xy_distance = np.linalg.norm(trajectory.positions[:, :2] - trajectory.positions[0, :2], axis=1)
    first_ride = ride_intervals[0][0] - trajectory.start if ride_intervals else trajectory.times[-1]
    departure = np.flatnonzero(xy_distance >= 1.5)
    if not len(departure):
        raise ValueError("No departure from the initial area")
    left = departure[0]
    returned = np.flatnonzero((trajectory.arc - trajectory.arc[left] > 30.)
                              & (xy_distance < 5.) & (trajectory.times < first_ride - 3.5))
    if not len(returned):
        raise ValueError("No start-area revisit found before the first elevator ride")
    discontinuities = np.flatnonzero(np.diff(returned) > 1)
    first_pass = returned[:discontinuities[0] + 1] if len(discontinuities) else returned
    right = first_pass[np.argmin(xy_distance[first_pass])]
    return left, right


def window_cloud(bag, trajectory, start, stop, step):
    clouds = []
    previous = -np.inf
    for _, raw, _ in bag.read_messages(
        topics=["/livox/lidar"], raw=True,
        start_time=rospy.Time.from_sec(trajectory.start + start - .4),
        end_time=rospy.Time.from_sec(trajectory.start + stop + .4),
    ):
        stamp, _, fields = livox_scan(raw)
        t = stamp - trajectory.start
        if start <= t < stop and t - previous >= step:
            cloud, _, _ = trajectory.project(fields, t, 36.)
            clouds.append(voxel_cloud(cloud, .20))
            previous = t
    if not clouds:
        raise ValueError("No LiDAR scans in a loop registration window")
    cloud = voxel_cloud(np.concatenate(clouds), .20)
    return cloud[np.linalg.norm(cloud[:, :2] - trajectory.positions[0, :2], axis=1) < 19.]


class LoopRegistration:
    def __init__(self, target):
        if len(target) < 1000:
            raise ValueError("Insufficient reference geometry")
        self.target = target
        self.tree = cKDTree(target)
        nearby = target[self.tree.query(target, k=16)[1]]
        centered = nearby - nearby.mean(axis=1, keepdims=True)
        eigenvalues, basis = np.linalg.eigh(np.einsum("nji,njk->nik", centered, centered))
        self.normals = basis[:, :, 0]
        self.flat = ((eigenvalues[:, 0] < .07 * eigenvalues[:, 1])
                     & (eigenvalues[:, 1] > .07 * eigenvalues[:, 2]))

    def estimate(self, source):
        if len(source) < 1000:
            raise ValueError("Insufficient revisit geometry")
        anchor = np.median(source, axis=0)
        yaw, translation = 0., np.zeros(3)
        for maximum in (1.8, 1.1, .65, .45, .34):
            for _ in range(16):
                rotated = Rotation.from_rotvec((0., 0., yaw)).apply(source - anchor)
                proposed = rotated + anchor + translation
                distance, closest = self.tree.query(proposed)
                usable = (distance < maximum) & self.flat[closest]
                if usable.sum() < 500:
                    raise ValueError("Too few matched planes")
                normals = self.normals[closest[usable]]
                residual = np.einsum("ij,ij->i", proposed[usable] - self.target[closest[usable]],
                                     normals)
                jacobian = np.column_stack((np.cross(rotated[usable], normals)[:, 2], normals))
                weight = np.sqrt(np.minimum(1., .13 / np.maximum(np.abs(residual), 1e-6)))
                step, _, rank, singular_values = np.linalg.lstsq(
                    jacobian * weight[:, None], -residual * weight, rcond=None)
                if rank < 4 or singular_values[-1] < .005 * singular_values[0]:
                    raise ValueError("Plane geometry does not constrain position and heading")
                yaw += step[0]
                translation += step[1:]
                if np.linalg.norm(step) < 1e-4:
                    break
        offset = translation + anchor - Rotation.from_rotvec((0., 0., yaw)).apply(anchor)
        if np.linalg.norm(offset) > 2. or abs(yaw) > np.deg2rad(5.):
            raise ValueError("Loop proposal exceeds the bounded refinement range")
        return np.r_[yaw, offset]

    def measure(self, cloud):
        distance, closest = self.tree.query(cloud)
        near = distance < .35
        usable = near & self.flat[closest]
        if usable.sum() < 500:
            raise ValueError("Insufficient supported points to verify loop")
        residual = np.einsum("ij,ij->i", (cloud - self.target[closest])[usable],
                             self.normals[closest[usable]])
        return dict(overlap=float(near.mean()), plane_rmse=float(np.sqrt(np.mean(residual**2))))


def fit_loop(bag, trajectory, departure, revisit):
    target = window_cloud(bag, trajectory, max(0., departure - 6.7), departure + 5.3, .40)
    alignment = LoopRegistration(target)
    spans = [(-3.9, -1.4), (-1.4, .8), (.8, 3.0)]
    sources = [window_cloud(bag, trajectory, revisit + lo, revisit + hi, .26) for lo, hi in spans]
    candidates = np.vstack([alignment.estimate(source) for source in sources])
    correction = np.median(candidates, axis=0)
    if (np.max(np.abs(candidates[:, 0] - correction[0])) > np.deg2rad(.7)
            or np.max(np.linalg.norm(candidates[:, 1:] - correction[1:], axis=1)) > .15):
        raise ValueError(f"Independent revisit windows disagree: {candidates.tolist()}")
    response = Rotation.from_rotvec((0., 0., correction[0]))
    verification = []
    for source in sources:
        original = alignment.measure(source)
        matched = alignment.measure(response.apply(source) + correction[1:])
        if (matched["overlap"] < .65 or matched["overlap"] - original["overlap"] < .10
                or matched["plane_rmse"] > .12 or matched["plane_rmse"] > .7 * original["plane_rmse"]):
            raise ValueError(f"Insufficient geometric improvement: {original} -> {matched}")
        verification.append({"before": original, "after": matched})
    return correction, verification


def apply_correction(points, times, trajectory, correction, first_arc, last_arc):
    progress = np.clip((np.interp(times, trajectory.times, trajectory.arc) - first_arc)
                       / (last_arc - first_arc), 0., 1.)
    progress = progress**2 * (3. - 2. * progress)
    vectors = np.zeros((len(progress), 3))
    vectors[:, 2] = progress * correction[0]
    rotation = Rotation.from_rotvec(vectors)
    return rotation.apply(points) + progress[:, None] * correction[1:], rotation


def rebuild_cloud(bag, trajectory, correction, first_arc, last_arc, ride_intervals):
    quaternions = trajectory.rotations.as_quat()
    chunks, clouds = [], []
    previous = None
    count = 0
    for _, raw, _ in bag.read_messages(topics=["/livox/lidar"], raw=True):
        stamp, duration, fields = livox_scan(raw)
        end = stamp + duration
        if (stamp < trajectory.start or end > trajectory.start + trajectory.times[-1] + .05
                or any(stamp <= b + .25 and end >= a - .25 for a, b in ride_intervals)):
            continue
        timestamp = end - trajectory.start
        index = min(int(np.searchsorted(trajectory.times, timestamp)), len(trajectory.times)-1)
        if previous is not None:
            moved = np.linalg.norm(trajectory.positions[index] - trajectory.positions[previous])
            same_heading = abs(quaternions[previous] @ quaternions[index]) > np.cos(.06 / 2.)
            if moved < .23 and same_heading and timestamp - trajectory.times[previous] < 2.:
                continue
        points, times, intensity = trajectory.project(fields, stamp - trajectory.start, 55.)
        if not len(points):
            continue
        mapped, _ = apply_correction(points, times, trajectory, correction, first_arc, last_arc)
        clouds.append(voxel_cloud(np.column_stack((mapped, intensity)).astype(np.float32), .12))
        count += 1
        previous = index
        if len(clouds) == 60:
            chunks.append(voxel_cloud(np.concatenate(clouds), .12))
            clouds.clear()
        if count % 300 == 0:
            print(f"Reprojected {count} scans", flush=True)
    if clouds:
        chunks.append(voxel_cloud(np.concatenate(clouds), .12))
    if count < 100:
        raise ValueError("Too few scans to rebuild the full map")
    return voxel_cloud(np.concatenate(chunks), .12), count


def save_map_cloud(path, points):
    header = (
        "VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n"
        "COUNT 1 1 1 1\nWIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS {n}\nDATA binary\n"
    ).format(n=len(points))
    with path.open("wb") as file:
        file.write(header.encode("ascii"))
        points.astype("<f4", copy=False).tofile(file)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--map-prefix", type=Path, required=True)
    parser.add_argument("--sensor-config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True,
                        help="New sibling map directory. Existing outputs are protected.")
    args = parser.parse_args()
    prefix = args.map_prefix.resolve()
    output = args.output_dir.resolve()
    if output.exists():
        parser.error("Output directory already exists")
    metadata = json.loads(Path(f"{prefix}_plan.json").read_text())
    mode_log = Path(f"{prefix}_elevator_debug.csv")
    ride_intervals = metadata["elevator_intervals"]
    trajectory = Trajectory(Path(f"{prefix}.tum"), args.sensor_config)
    left, right = find_revisit(trajectory, ride_intervals)
    print(f"Start-area revisit at {trajectory.times[right]:.2f} s, "
          f"raw height difference {trajectory.positions[right, 2] - trajectory.positions[0, 2]:+.3f} m",
          flush=True)
    first_arc, last_arc = trajectory.arc[left], trajectory.arc[right]
    with rosbag.Bag(str(args.bag)) as bag:
        correction, checks = fit_loop(bag, trajectory, trajectory.times[left], trajectory.times[right])
        print(f"Verified loop correction: yaw {np.rad2deg(correction[0]):+.3f} deg, "
              f"translation {correction[1:].round(3).tolist()} m", flush=True)
        pcd, count = rebuild_cloud(bag, trajectory, correction, first_arc, last_arc, ride_intervals)
    xyz, rotations = apply_correction(trajectory.positions, trajectory.times, trajectory,
                                     correction, first_arc, last_arc)
    tum = np.column_stack((trajectory.rows[:, 0], xyz, (rotations * trajectory.rotations).as_quat()))
    planner = Path(__file__).resolve().parents[3] / "nav/src/map_for_plan/scripts/segmented_tum_to_plan_map.py"
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=output.parent, prefix=".loop-") as directory:
        staged = Path(directory) / output.name
        staged.mkdir()
        target = staged / prefix.name
        save_map_cloud(target.with_suffix(".pcd"), pcd)
        np.savetxt(target.with_suffix(".tum"), tum, fmt="%.6f")
        shutil.copy2(mode_log, Path(f"{target}_elevator_debug.csv"))
        subprocess.run([
            sys.executable, str(planner), "--traj", f"{target}.tum", "--mode-log",
            f"{target}_elevator_debug.csv", "--output-prefix", str(target),
            "--resolution", str(metadata["resolution"]), "--half-width", str(metadata["half_width"]),
            "--pose-stride", str(metadata["pose_stride"]), "--z-offset", str(metadata["z_offset"]),
        ], check=True)
        plan_path = Path(f"{target}_plan.json")
        plan = json.loads(plan_path.read_text())
        plan["traj"] = str(output / f"{prefix.name}.tum")
        plan["mode_log"] = str(output / f"{prefix.name}_elevator_debug.csv")
        plan_path.write_text(json.dumps(plan, indent=2) + "\n")
        report = {
            "input_map_prefix": str(prefix), "input_bag": str(args.bag.resolve()),
            "reference_timestamp": float(trajectory.rows[left, 0]),
            "revisit_timestamp": float(trajectory.rows[right, 0]),
            "raw_revisit_height_difference_m": float(trajectory.positions[right, 2] - trajectory.positions[0, 2]),
            "corrected_revisit_height_difference_m": float(xyz[right, 2] - xyz[0, 2]),
            "correction_yaw_deg": float(np.rad2deg(correction[0])),
            "correction_translation_m": correction[1:].tolist(),
            "independent_registration_windows": checks,
            "reprojected_scans": count, "global_map_points": int(len(pcd)),
        }
        (staged / "loop_report.json").write_text(json.dumps(report, indent=2) + "\n")
        staged.rename(output)
    print(f"Corrected map: {output}", flush=True)


if __name__ == "__main__":
    try:
        main()
    except ValueError as error:
        sys.exit(str(error))
