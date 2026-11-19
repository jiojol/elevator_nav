#!/usr/bin/env python3
"""Hover motion follows terrain without a gait, but cannot cross solid boxes."""
from pathlib import Path
from types import SimpleNamespace
import sys
import tempfile
import unittest
import mujoco
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'simulation'))
from physics_bridge import scene, Simulation
from hover_motion import HoverMotion


class HoverSimulationTest(unittest.TestCase):
    def make(self,case):
        directory=tempfile.TemporaryDirectory();self.addCleanup(directory.cleanup)
        xml,_,start,_,_=scene(Path('/nonexistent-assets'),case,Path(directory.name),'hover')
        model=mujoco.MjModel.from_xml_path(str(xml));data=mujoco.MjData(model)
        terrain=SimpleNamespace(args=SimpleNamespace(case=case))
        terrain.floor=lambda x,y:Simulation.floor(terrain,x,y)
        support=lambda x,y:Simulation.hover_support(terrain,x,y)
        data.qpos[:3]=start;data.qpos[2]=support(*start[:2])+.4;data.qpos[3]=1.
        mujoco.mj_forward(model,data)
        return model,data,HoverMotion(model,data,support,.4),support

    def test_stairs_climb_and_descend_without_actuators_at_fixed_clearance(self):
        model,data,motion,support=self.make('stairs')
        self.assertEqual(model.nu,0)
        initial=data.qpos[0];max_z=data.qpos[2]
        for command in ([.8,0.,0.],[-.8,0.,0.]):
            for _ in range(1500):
                motion.step(command)
                self.assertFalse(motion.blocked)
                self.assertAlmostEqual(data.qpos[2]-support(*data.qpos[:2]),.4)
                np.testing.assert_allclose(data.qpos[3:7],[1.,0.,0.,0.])
                max_z=max(max_z,data.qpos[2])
        self.assertGreater(max_z,1.89)
        self.assertAlmostEqual(data.qpos[0],initial,places=6)

    def test_swept_body_cannot_tunnel_through_obstacle(self):
        model,data,motion,_=self.make('large_obstacle')
        # One deliberately enormous step crosses the entire box without an
        # intermediate sweep. The real check must stop at its near face.
        model.opt.timestep=1.
        motion.step([5.,0.,0.])
        self.assertTrue(motion.blocked)
        self.assertLessEqual(data.qpos[0],-.85+1e-8)
        self.assertGreater(data.qpos[0],-.88)
        self.assertTrue(all(data.contact[i].dist>=0. for i in range(data.ncon)))

    def test_yaw_and_lateral_commands_are_body_relative(self):
        _,data,motion,_=self.make('straight')
        data.qpos[3:7]=[np.sqrt(.5),0.,0.,np.sqrt(.5)]
        before=data.qpos[:2].copy()
        for _ in range(250):motion.step([.5,.2,0.])
        np.testing.assert_allclose(data.qpos[:2]-before,[-.2,.5],atol=1e-6)
        for _ in range(250):motion.step([0.,0.,.5])
        self.assertAlmostEqual(2.*np.arctan2(data.qpos[6],data.qpos[3]),np.pi/2+.5)

    def test_moving_obstacle_contacts_and_reset_clear_the_signal(self):
        model,data,motion,_=self.make('straight')
        data.mocap_pos[0]=[data.qpos[0],data.qpos[1],.4]
        motion.step([0.,0.,0.]);self.assertTrue(motion.blocked)
        data.mocap_pos[0]=[0.,8.,.4]
        for _ in range(60):motion.step([0.,0.,0.])
        self.assertFalse(motion.blocked)
        motion.reset();self.assertFalse(motion.blocked)


if __name__=='__main__':unittest.main()
