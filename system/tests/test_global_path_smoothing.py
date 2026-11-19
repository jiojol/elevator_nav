#!/usr/bin/env python3
"""Global smoothing must reduce grid turns without crossing FREE boundaries."""
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'nav/src/map_for_plan/scripts'))
from plan_graph import FreeGraph
from path_smoothing import smooth_free_path


class GlobalPathSmoothingTest(unittest.TestCase):
    def assert_edges_free(self,path,graph):
        for a,b in zip(path[:-1],path[1:]):
            for t in np.linspace(0.,1.,17):
                p=a+(b-a)*t;key=tuple(np.floor(p[:2]/graph.resolution).astype(int))
                self.assertTrue(any(abs(graph.points[i,2]-p[2])<=graph.resolution+1e-6
                                    for i in graph.columns.get(key,())))

    def test_grid_staircase_becomes_smooth_with_endpoints_fixed(self):
        points=np.array([[(x+.5)*.2,(y+.5)*.2,.1] for x in range(-3,20) for y in range(-3,20)])
        graph=FreeGraph(points,.2)
        path=np.array([[.1+.2*i,.1+.2*(i//2),.1] for i in range(15)])
        result=smooth_free_path(path,graph)
        def roughness(p):
            d=np.diff(p[:,:2],axis=0);yaw=np.unwrap(np.arctan2(d[:,1],d[:,0]));return np.sum(np.diff(yaw)**2)
        self.assertLess(roughness(result),.4*roughness(path))
        np.testing.assert_allclose(result[[0,-1]],path[[0,-1]])
        self.assert_edges_free(result,graph)

    def test_smoothing_cannot_shortcut_across_a_missing_free_region(self):
        points=np.array([[(x+.5)*.2,(y+.5)*.2,.1] for x in range(-3,15) for y in range(-4,15)
                         if not (3<=x<=7 and 3<=y<=7)])
        graph=FreeGraph(points,.2)
        path=graph.astar(graph.nearest([.1,1.1,.1],.1),graph.nearest([2.3,1.1,.1],.1))
        result=smooth_free_path(path,graph)
        self.assert_edges_free(result,graph)
        self.assertTrue(np.all(~((result[:,0]>.6)&(result[:,0]<1.6)&(result[:,1]>.6)&(result[:,1]<1.6))))

    def test_stair_profile_and_separate_floors_keep_their_heights(self):
        def height(x):return .1 if x<5 else .3 if x<10 else .5
        points=np.array([[(x+.5)*.2,(y+.5)*.2,z] for x in range(-2,16) for y in range(-3,4)
                         for z in [height(x),height(x)+2.]])
        graph=FreeGraph(points,.2)
        path=np.array([[(x+.5)*.2,.1,height(x)] for x in range(14)])
        result=smooth_free_path(path,graph)
        arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
        expected=np.interp(np.linspace(0.,arc[-1],len(result)),arc,path[:,2])
        np.testing.assert_allclose(result[:,2],expected)
        self.assertLessEqual(result[:,2].max(),.5)
        self.assert_edges_free(result,graph)


if __name__=='__main__':unittest.main()
