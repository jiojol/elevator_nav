#!/usr/bin/env python3
"""Summarize every retained trial, including failures and earlier sensor inputs."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np


def trial(folder):
    report = json.loads((folder/'result.json').read_text())
    events = json.loads((folder/'events.json').read_text())
    data = np.load(folder/'trajectory.npy')
    contact = [x['sim_time'] for x in events if x['event'] == 'contact_begin']
    delay = None
    if contact:
        idx = np.flatnonzero((data[:, 0] >= contact[0]) & (np.linalg.norm(data[:, 7:10], axis=1) < .005))
        if len(idx):
            delay = max(0., float(data[idx[0], 0]-contact[0]))
    goal = np.asarray(report['goal_reference_xyz'][:2])
    yaw = report['goal_yaw']
    offset = np.array([-.221454, -.018929])
    goal_body = goal + np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])@offset
    result = {
        'folder': folder.name, 'report': report,
        'input_revision': 'changing_livox' if 'lidar_sampling' in report else 'earlier_fixed_rays',
        'debug_logging': folder.name.startswith('debug_'),
        'body_path_length_m': float(np.linalg.norm(np.diff(data[:, 1:3], axis=0), axis=1).sum()),
        'final_body_xy_error_m': float(np.linalg.norm(data[-1, 1:3]-goal_body)),
        'final_body_yaw_error_rad': abs(math.atan2(math.sin(yaw-data[-1, 6]), math.cos(yaw-data[-1, 6]))),
        'first_contact_sim_s': contact[0] if contact else None,
        'first_sampled_zero_command_delay_s': delay,
        'raw_sha256': {name: hashlib.sha256((folder/name).read_bytes()).hexdigest()
                       for name in ['result.json', 'events.json', 'trajectory.npy', 'qpos.npy', 'mocap.npy', 'scene.xml', 'planner.log'] if (folder/name).exists()},
        'has_recorded_physics_replay': (folder/'qpos.npy').exists(),
    }
    metadata = folder/'run_metadata.json'
    if metadata.exists():
        result['run_metadata'] = json.loads(metadata.read_text())
    return result


def overview(base, output):
    cases = [
        ('livox_native_large_obstacle', 'Large static obstacle'),
        ('livox_native_door_065', '0.65 m doorway'),
        ('livox_native_door_055', '0.55 m doorway (first trial)'),
        ('livox_native_stairs', 'Stairs (first trial, side view)'),
        ('livox_native_dynamic_crossing', 'Crossing obstacle: native'),
        ('livox_native_straight', 'Straight path'),
    ]
    fig, axes = plt.subplots(2, 3, figsize=(14, 8), layout='constrained')
    for ax, (name, title) in zip(axes.flat, cases):
        folder = base/name
        if not folder.is_dir():
            folder = base/name.removeprefix('livox_')
        if not folder.is_dir():
            ax.set_title(title+'\nNo recorded trial')
            ax.axis('off')
            continue
        a = np.load(folder/'trajectory.npy')
        r = json.loads((folder/'result.json').read_text())
        is_stairs = r['case'] == 'stairs'
        for geom in ET.parse(folder/'scene.xml').getroot().find('worldbody').findall('geom'):
            if geom.get('type') != 'box':
                continue
            pos, size = np.fromstring(geom.get('pos'), sep=' '), np.fromstring(geom.get('size'), sep=' ')
            if is_stairs:
                if not geom.get('name').startswith(('stair_', 'landing')):
                    continue
                p, s = pos[[0, 2]], size[[0, 2]]
            else:
                p, s = pos[:2], size[:2]
            ax.add_patch(Rectangle(p-s, 2*s[0], 2*s[1], color='#d5dce5'))
        vertical = 3 if is_stairs else 2
        ax.plot(a[:, 1], a[:, vertical], color='#087f8c', lw=2, label='Measured body')
        ax.scatter(a[0, 1], a[0, vertical], color='#087f8c', marker='o', zorder=5)
        ax.scatter(a[-1, 1], a[-1, vertical], color='#bc3d32' if r['result'] != 'reached' else '#087f8c', marker='X', s=65, zorder=6)
        if 'dynamic_crossing' in name:
            mocap = np.load(folder/'mocap.npy')[:, 0, :]
            ax.plot(mocap[:, 0], mocap[:, 1], '--', color='#bc3d32', label='Obstacle centre')
            events = json.loads((folder/'events.json').read_text())
            hits = [x['sim_time'] for x in events if x['event'] == 'contact_begin']
            if hits:
                idx = np.argmin(abs(a[:, 0]-hits[0]))
                ax.scatter(a[idx, 1], a[idx, 2], marker='*', s=140, color='#bc3d32', zorder=7, label='First contact')
        contact_note = ', contact observed' if (r['counts']['body_contacts'] or r['counts']['obstacle_contacts']) else ''
        ax.set_title(f'{title}\n{r["result"]}, {r["duration_sim"]:.2f} s{contact_note}', fontsize=11)
        ax.set_xlabel('World x (m)')
        ax.set_ylabel('Body height z (m)' if is_stairs else 'World y (m)')
        ax.set_xlim(-1.2 if is_stairs else -3.1, 4.8 if is_stairs else 3.1)
        ax.set_ylim(-.1 if is_stairs else -2., 2.1 if is_stairs else 2.)
        ax.set_aspect('equal', adjustable='box')
        ax.grid(alpha=.2)
        ax.legend(fontsize=8, loc='upper left')
    fig.suptitle('Go2 MuJoCo physics — recorded trials, not a success-rate estimate', fontsize=15)
    fig.savefig(output/'trajectories.png', dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('artifacts', type=Path)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    trials = [trial(path.parent) for path in sorted(args.artifacts.glob('*/result.json'))]
    root = Path(__file__).resolve().parents[1]
    paths = ['simulation/physics_bridge.py', 'simulation/isolated.launch', 'nav/src/dog_nav_bringup/scripts/scan_command_gate.py']
    summary = {'trials': trials, 'tested_code_sha256': {p: hashlib.sha256((root/p).read_bytes()).hexdigest() for p in paths},
               'contact_counts_are_sampled_pairs_not_distinct_collisions': True,
               'zero_command_delay_is_not_physical_stop_latency': True}
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    overview(args.artifacts, args.output)
