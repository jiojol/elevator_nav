#!/usr/bin/env python3
"""Render recorded MuJoCo states for evidence; never drives a controller."""
import argparse
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image, ImageDraw


def render(folder, times, output):
    model = mujoco.MjModel.from_xml_path(str(folder/'scene.xml'))
    model.vis.global_.offwidth, model.vis.global_.offheight = 720, 480
    data = mujoco.MjData(model)
    poses = np.load(folder/'qpos.npy')
    trajectory = np.load(folder/'trajectory.npy')
    mocap = np.load(folder/'mocap.npy')
    # The sensor excludes all robot geoms. Show visual meshes in this offline
    # replay, and hide duplicate physical collision capsules.
    for i in range(model.ngeom):
        if model.geom_group[i] == 3 and model.geom_contype[i] == model.geom_conaffinity[i] == 0:
            model.geom_group[i] = 1
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
        if name in {'wall_n', 'wall_s', 'wall_e', 'wall_w', 'wall_stair_n', 'wall_stair_s'}:
            model.geom_rgba[i, 3] = 0.  # replay cutaway; physics is already recorded
    model.vis.headlight.ambient[:] = .4
    model.vis.headlight.diffuse[:] = .7
    option = mujoco.MjvOption()
    option.geomgroup[3] = 0
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [1.7, 0., .9] if 'stairs' in folder.name else [0., 0., .4]
    camera.distance, camera.azimuth, camera.elevation = 6., 100., -32.
    frames = []
    with mujoco.Renderer(model, height=480, width=720) as renderer:
        for t in times:
            index = int(np.argmin(abs(trajectory[:, 0]-t)))
            data.qpos[:] = poses[index]
            data.mocap_pos[:] = mocap[index]
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=camera, scene_option=option)
            frame = Image.fromarray(renderer.render())
            ImageDraw.Draw(frame).text((14, 12), f'{folder.name} | recorded physics t={trajectory[index, 0]:.2f}s', fill='black', stroke_width=1, stroke_fill='white')
            frames.append(frame)
    canvas = Image.new('RGB', (720*2, 480*((len(frames)+1)//2)), 'white')
    for i, frame in enumerate(frames):
        canvas.paste(frame, ((i%2)*720, (i//2)*480))
    output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    parser.add_argument('--times', type=float, nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    render(args.folder, args.times, args.output)
