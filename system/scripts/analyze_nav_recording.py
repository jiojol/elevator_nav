#!/usr/bin/env python3
"""Audit recorded trajectories and export timestamped offline replay scenes."""
import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
from zoneinfo import ZoneInfo

import numpy as np
import rosbag
from scipy.interpolate import BSpline
import yaml


def audit_curve(msg, path):
    cp = np.array([[p.x, p.y, p.z] for p in msg.pos_pts])
    curve = BSpline(msg.knots, cp, msg.order)
    begin, end = msg.knots[msg.order], msg.knots[-msg.order - 1]
    samples = curve(np.linspace(begin, end, 401))
    record = {'id': msg.traj_id, 'duration': end - begin,
              'start': samples[0].tolist(), 'end': samples[-1].tolist()}
    if path is not None and len(path.poses) > 1:
        route = np.array([[p.pose.position.x, p.pose.position.y] for p in path.poses])
        delta = np.diff(route, axis=0)
        lengths = np.linalg.norm(delta, axis=1)
        fractions = np.clip(np.einsum('ij,ij->i', samples[0, :2] - route[:-1], delta) /
                            np.maximum(lengths ** 2, 1e-12), 0, 1)
        feet = route[:-1] + fractions[:, None] * delta
        index = int(np.argmin(np.linalg.norm(feet - samples[0, :2], axis=1)))
        tangent = delta[index] / max(lengths[index], 1e-12)
        displacement = samples[:, :2] - samples[0, :2]
        travelled = np.r_[0., np.cumsum(np.linalg.norm(np.diff(samples[:, :2], axis=0), axis=1))]
        local = displacement[travelled <= .4]
        record['initial_reverse_m'] = float(max(0., -(local @ tangent).min()))
        first = np.flatnonzero(np.linalg.norm(displacement, axis=1) >= .02)
        if len(first):
            direction = displacement[first[0]]
            record['initial_heading_error_deg'] = float(np.degrees(np.arctan2(
                tangent[0] * direction[1] - tangent[1] * direction[0], tangent @ direction)))
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--snapshot', type=float, action='append', default=[], help='UTC epoch of scene to export')
    args = parser.parse_args()
    output = args.recording / 'analysis'
    output.mkdir(exist_ok=True)
    latest, trajectories, tracking = {}, [], []
    snapshots = {stamp: {} for stamp in args.snapshot}
    topics = ['/dog_nav/odometry', '/dog_nav/sensor_pose', '/grid_map/occupancy',
              '/initial_path', '/plan_map/global_path', '/planning/bspline',
              '/navigation/obstacle_points', '/go2/sportmodestate', '/rosout',
              '/cmd_vel', '/dog_nav/localization_status']
    for bag_path in sorted(args.recording.glob('navigation_*.bag')):
        with rosbag.Bag(str(bag_path)) as bag:
            for topic, msg, stamp in bag.read_messages(topics=topics):
                latest[topic] = (msg, stamp)
                for cutoff, selected in snapshots.items():
                    if stamp.to_sec() <= cutoff:
                        selected[topic] = (msg, stamp)
                if topic == '/planning/bspline':
                    item = audit_curve(msg, latest.get('/initial_path', (None,))[0])
                    item['epoch'] = stamp.to_sec()
                    item['time_local'] = datetime.fromtimestamp(stamp.to_sec(), ZoneInfo('Asia/Shanghai')).isoformat()
                    trajectories.append(item)
                elif topic == '/rosout' and '[closed_loop_controller] tracking' in msg.msg:
                    tracking.append((stamp.to_sec(), msg.msg))
    (output / 'trajectories.json').write_text(json.dumps(trajectories, indent=2) + '\n')
    with (output / 'tracking.csv').open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['epoch', 'message'])
        writer.writerows(tracking)
    params = yaml.safe_load((args.recording / 'params_end.yaml').read_text())
    for cutoff, selected in snapshots.items():
        scene = output / ('scene_%d' % cutoff)
        scene.mkdir(exist_ok=True)
        with rosbag.Bag(str(scene / 'scene.bag'), 'w') as bag:
            for topic, (msg, stamp) in selected.items():
                bag.write(topic, msg, stamp)
        (scene / 'planner_params.yaml').write_text(yaml.safe_dump(params['scan_planner_node']))
        (scene / 'global_params.yaml').write_text(yaml.safe_dump(params['global_planner']))
        print('Snapshot:', scene, flush=True)
    for item in trajectories:
        if item.get('initial_reverse_m', 0.) > .03 or abs(item.get('initial_heading_error_deg', 0.)) > 90:
            print('Unexpected initial direction:', json.dumps(item), flush=True)
    print('Audited %d trajectories. Outputs: %s' % (len(trajectories), output), flush=True)


if __name__ == '__main__':
    main()
