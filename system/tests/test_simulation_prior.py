#!/usr/bin/env python3
"""FREE topology and robot support must govern selected simulation goals."""
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'simulation'))
from prior_navigation import PriorNavigation, REFERENCE_OFFSET, rotate


class SimulationPriorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.prior=PriorNavigation('large_obstacle',[
            ('static_box',np.array([0.,0.,.4]),np.array([.5,.55,.4]))],lambda x,y:0.)

    def test_prior_excludes_obstacle_and_unrecorded_open_ground(self):
        self.assertFalse(self.prior.contains([0.,0.]))
        self.assertFalse(self.prior.contains([0.,2.]))
        self.assertTrue(self.prior.contains([0.,-1.2]))

    def test_near_side_goal_without_body_clearance_is_rejected(self):
        with self.assertRaisesRegex(ValueError,'无法安全停靠'):
            self.prior.validate_body_goal({'x':.87,'y':0.,'yaw':0.})

    def test_reachable_far_side_goal_routes_around_inside_the_prior(self):
        goal=self.prior.validate_body_goal({'x':1.2,'y':0.,'yaw':0.})
        start=np.array([-2.278546,.018929,.02])
        path=self.prior.route(start,goal,0.)
        self.assertLess(path[:,1].min(),-.9)
        self.assertGreater(np.linalg.norm(path[-1]-path[-2]),.075)
        for a,b in zip(path[:-1],path[1:]):
            yaw=np.arctan2(b[1]-a[1],b[0]-a[0])
            for u in np.linspace(0.,1.,8):
                self.assertTrue(self.prior.pose_supported(a+(b-a)*u,yaw))

    def test_goal_outside_prior_is_rejected_without_snapping(self):
        with self.assertRaisesRegex(ValueError,'不在绿色先验'):
            self.prior.validate_body_goal({'x':2.,'y':2.,'yaw':0.})


class StairPriorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Physical top faces, independent of the simulation's index-based floor
        # callback. Keep FREE at these actual elevations throughout the test.
        cls.boxes=[(f'stair_{i}',np.array([.15+.3*i,0.,.06+.12*i]),np.array([.15,.8,.06])) for i in range(12)]
        cls.boxes += [('landing',np.array([4.15,0.,1.44]),np.array([.55,.9,.06])),
                      ('wall_stair_n',np.array([1.8,.86,.75]),np.array([2.,.04,.75])),
                      ('wall_stair_s',np.array([1.8,-.86,.75]),np.array([2.,.04,.75]))]
        def floor(x,y):
            return max([0.]+[c[2]+h[2] for name,c,h in cls.boxes
                            if name.startswith(('stair_','landing')) and abs(x-c[0])<=h[0]+1e-9 and abs(y-c[1])<=h[1]])
        cls.prior=PriorNavigation('stairs',cls.boxes,floor)

    def test_prior_covers_each_tread_at_its_physical_height(self):
        for name,c,h in self.boxes:
            if not name.startswith(('stair_','landing')):continue
            self.assertTrue(self.prior.contains(c[:2]),name)
            i,j=self.prior._indices(c[:2])
            self.assertAlmostEqual(self.prior.z[i,j],c[2]+h[2],msg=name)
        self.assertFalse(self.prior.contains([2.,.9]))

    def test_up_and_down_routes_keep_whole_body_supported_at_risers(self):
        for start_x,goal_x,yaw in [(-.8,4.,0.),(4.,-.8,np.pi)]:
            with self.subTest(yaw=yaw):
                start=self.prior.validate_body_goal(dict(x=start_x,y=0.,yaw=yaw))
                goal=self.prior.validate_body_goal(dict(x=goal_x,y=0.,yaw=yaw))
                path=self.prior.route(start,goal,yaw)
                np.testing.assert_allclose(path[0],start)
                np.testing.assert_allclose(path[-1],goal)
                self.assertAlmostEqual(abs(path[-1,2]-path[0,2]),1.5)
                for a,b in zip(path[:-1],path[1:]):
                    heading=np.arctan2(b[1]-a[1],b[0]-a[0])
                    for u in np.linspace(0.,1.,21):
                        self.assertTrue(self.prior.pose_supported(a+(b-a)*u,heading),(a,b,u))
                # Reference heights follow the body, while the raw FREE surface
                # stays under the tread rather than being lifted to meet a path.
                for k in range(1,len(path)-1):
                    heading=np.arctan2(*(path[k+1,:2]-path[k,:2])[::-1])
                    body=path[k,:2]+rotate(REFERENCE_OFFSET,heading)
                    self.assertAlmostEqual(path[k,2],self.prior.floor(*body))


if __name__=='__main__':unittest.main()
