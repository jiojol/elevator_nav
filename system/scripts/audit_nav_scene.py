#!/usr/bin/env python3
"""Read-only audit of recorded body pose, ground reference and occupancy.

No ROS master, publishers, parameter changes or map clearing are performed.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
import rosbag
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from sensor_msgs import point_cloud2
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'nav/src/map_for_plan/scripts'))
from plan_io import load_plan_meta, read_pcd_xyz


def position(pose):
    p = pose.position
    return np.array([p.x, p.y, p.z])


def points(msg):
    return np.asarray(list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))).reshape(-1, 3)


def body_hits(occupied, center, rotation, half, resolution):
    extent = np.abs(rotation) @ half
    lower = (np.floor((center - extent) / resolution) + .5) * resolution
    upper = (np.floor((center + extent) / resolution) + .5) * resolution
    bounded = occupied[np.all((occupied >= lower - 1e-6) & (occupied <= upper + 1e-6), axis=1)]
    allowance = half + .5 * resolution * np.abs(rotation).sum(axis=0)
    return bounded[np.all(np.abs((bounded - center) @ rotation) <= allowance + 1e-9, axis=1)]


def audit(scene, global_params, meta_path, incident_time=None):
    params = yaml.safe_load((scene / 'planner_params.yaml').read_text())
    global_config = yaml.safe_load(global_params.read_text())
    grid = params['grid_map']
    body_height = float(global_config['body_height'])
    path_offset = float(global_config['path_z_offset'])
    latest, scans, sensor_poses, states, body_positions = {}, [], {}, [], []
    with rosbag.Bag(str(scene / 'scene.bag')) as bag:
        start, end = bag.get_start_time(), bag.get_end_time()
        for topic, msg, _ in bag.read_messages():
            latest[topic] = msg
            if topic == '/navigation/obstacle_points':
                scans.append(msg)
            elif topic == '/dog_nav/sensor_pose':
                sensor_poses[msg.header.stamp.to_nsec()] = position(msg.pose.pose)
            elif topic == '/dog_nav/odometry':
                body_positions.append(position(msg.pose.pose))
            elif topic == '/go2/sportmodestate':
                states.append(json.loads(msg.data))
    pose = latest['/dog_nav/odometry'].pose.pose
    body = position(pose)
    q = pose.orientation
    measured_rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
    yaw = measured_rotation.as_euler('xyz')[2]
    resolution = float(grid['resolution'])
    half = np.array([grid['body_length'] / 2 + grid['body_clearance_xy'],
                     grid['body_width'] / 2 + grid['body_clearance_xy'],
                     (grid['obstacles_inflation_z_up'] + grid['obstacles_inflation_z_down']) / 2 + grid['body_clearance']])
    flat_rotation = Rotation.from_euler('z', yaw).as_matrix()
    occupied = points(latest['/grid_map/occupancy'])
    lower, upper = grid['obstacles_inflation_z_up'], grid['obstacles_inflation_z_down']
    rear = grid.get('body_rear_lower_extent', lower)
    parts = [(np.array([0., 0., .5 * (upper - lower)]), half)]
    if rear < lower - 1e-8:
        quarter = .25 * grid['body_length']
        parts = [(np.array([quarter, 0., .5 * (upper - lower)]),
                  np.array([quarter + grid['body_clearance_xy'], half[1], half[2]])),
                 (np.array([-quarter, 0., .5 * (upper - rear)]),
                  np.array([quarter + grid['body_clearance_xy'], half[1],
                            .5 * (upper + rear) + grid['body_clearance']]))]
    def contacts_for(rotation):
        return np.unique(np.concatenate([body_hits(occupied, body + rotation @ offset, rotation, size, resolution)
                                         for offset, size in parts]), axis=0)
    contacts = contacts_for(flat_rotation)
    actual_rotation = measured_rotation.as_matrix()
    actual_contacts = contacts_for(actual_rotation)
    raw_arrays = [(scan, points(scan)) for scan in scans]
    raw_trees = [cKDTree(array) for _, array in raw_arrays if len(array)]
    contact_details = []
    for voxel in contacts[:16]:
        hit_frames, traversed_frames = 0, 0
        for scan, array in raw_arrays:
            if not len(array):
                continue
            hit_frames += int(np.any(np.all(np.abs(array - voxel) <= resolution / 2 + 1e-7, axis=1)))
            origin = sensor_poses.get(scan.header.stamp.to_nsec())
            if origin is None:
                continue
            direction = array - origin
            lengths = np.linalg.norm(direction, axis=1)
            # The mapper truncates rays at max_ray_length; do the same here.
            direction *= np.minimum(1., grid['max_ray_length'] / np.maximum(lengths, 1e-12))[:, None]
            with np.errstate(divide='ignore', invalid='ignore'):
                first = (voxel - resolution / 2 - origin) / direction
                last = (voxel + resolution / 2 - origin) / direction
            enter = np.minimum(first, last).max(axis=1)
            leave = np.maximum(first, last).min(axis=1)
            traversed_frames += int(np.any((enter <= leave) & (leave > 0) & (enter < 1)))
        contact_details.append({
            'voxel': voxel.tolist(), 'relative_to_measured_body': ((voxel - body) @ measured_rotation.as_matrix()).tolist(),
            'observed_hit_frames': hit_frames, 'ray_traversed_frames': traversed_frames,
            'nearest_observed_return_m': min(float(tree.query(voxel)[0]) for tree in raw_trees) if raw_trees else None,
        })
    meta = load_plan_meta(meta_path)
    ground_points = read_pcd_xyz(Path(meta['free_pcd']))
    radius = float(global_config.get('snap_radius', 2.))
    def snap(query):
        delta = ground_points - query
        nearby = np.flatnonzero(np.sum(delta[:, :2] ** 2, axis=1) <= radius ** 2)
        if not len(nearby):
            return None
        index = nearby[np.argmin(np.sum(delta[nearby] ** 2, axis=1))]
        ground = ground_points[index]
        reference_z = float(ground[2] + body_height + path_offset)
        return {'ground_vertex': ground.tolist(), 'reference_body_z': reference_z,
                'reference_minus_measured_body_z': reference_z - float(body[2])}
    report = {
        'scene': str(scene), 'bag_start_utc_epoch': start, 'bag_end_utc_epoch': end,
        'seconds_after_incident': start - incident_time if incident_time is not None else None,
        'measured_body_position': body.tolist(), 'measured_body_rpy_deg': measured_rotation.as_euler('xyz', degrees=True).tolist(),
        'pose_min': np.min(body_positions, axis=0).tolist(), 'pose_max': np.max(body_positions, axis=0).tolist(),
        'nominal_body_height': body_height, 'additional_reference_offset': path_offset,
        'ground_snap_using_body_xyz_legacy': snap(body),
        'ground_snap_using_projected_surface_fixed': snap(body - [0, 0, body_height]),
        'body_collision_half_size': half.tolist(), 'occupancy_voxels': len(occupied),
        'body_collision_parts': [{'offset': offset.tolist(), 'half_size': size.tolist()} for offset, size in parts],
        'diagnostic_flat_pitch_body_contacts': len(contacts), 'contact_details': contact_details,
        'diagnostic_measured_attitude_body_contacts': len(actual_contacts),
        'raw_scan_frames': len(scans), 'reference_poses_in_snapshot': len(latest['/initial_path'].poses),
        'limits': ['Flat-pitch diagnostic only; the original reference-predicted pitch may differ.',
                   'No return and no traversing ray is unobserved space, NOT evidence that a voxel is free.',
                   'A later snapshot cannot certify what was occupied at the incident time.'],
    }
    if states:
        heights = [state['body_height'] for state in states]
        report['sdk_body_height_range'] = [min(heights), max(heights)]
        report['sdk_last_state'] = states[-1]
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene', type=Path)
    parser.add_argument('--global-params', type=Path, required=True)
    parser.add_argument('--meta', type=Path, default=ROOT / 'system/map/dog_plan.json')
    parser.add_argument('--incident-time', type=float)
    args = parser.parse_args()
    print(json.dumps(audit(args.scene, args.global_params, args.meta, args.incident_time), indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
