import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'nav/src/map_for_plan/scripts'))
from cloud_contract import validate_request

class ContractTests(unittest.TestCase):
    def test_correlated_goal(self):
        c={'instance_id':'boot','asset_id':'asset','map_request_id':'load','frame_id':'map'}
        v={**{k:c[k] for k in ('instance_id','asset_id','map_request_id')},'coordinate_frame':'map','request_id':'goal1','owner':'web','pose':{'x':0,'y':0,'z':0,'qx':0,'qy':0,'qz':0,'qw':1}}
        self.assertEqual(validate_request(v,c),v)
        self.assertEqual(validate_request(dict(v,map_label="floor1"),c)["map_label"],"floor1")
        for bad in [dict(v,instance_id='old'),dict(v,owner='native'),dict(v,map_label='bad'),dict(v,map_label=2),dict(v,pose=dict(v['pose'],x=float('nan')))]:
            with self.assertRaises(ValueError):validate_request(bad,c)
