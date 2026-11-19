import sys,time,threading,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'nav/src/map_for_plan/scripts'))
import rospy
from nav_msgs.msg import Odometry
from global_planner import GlobalPlanner

class ReceiptTests(unittest.TestCase):
    def test_busy_planner_receives_source_stamps_without_faking_freshness(self):
        p=GlobalPlanner.__new__(GlobalPlanner)
        p.lock=threading.RLock();p.initial_pose_after=None;p.planning_only=False
        p.cloud_localization='';p.cloud_localization_at=0.
        p.cloud_asset='asset';p.cloud_instance='instance';p.cloud_map_request='load';p.frame_id='map'
        m=Odometry();m.header.stamp=rospy.Time.from_sec(100.)
        with p.lock:
            t=threading.Thread(target=p._on_odom,args=(m,));t.start();t.join(.2)
            self.assertFalse(t.is_alive(), 'odometry blocked behind graph planning')
            self.assertEqual(p.odom_receipt[0],m.header.stamp)
            with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(100.1)),patch.object(rospy,'get_param',side_effect=lambda key,default=None: True if key=='/integration/simulation' else 'web'):
                c=p._cloud_context();self.assertTrue(c['ready']);self.assertEqual(c['localization_stamp'],100.)
            with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(100.6)),patch.object(rospy,'get_param',return_value=True):
                self.assertFalse(p._cloud_context()['ready'])
            p.initial_pose_after=rospy.Time.from_sec(101.)
            with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(100.1)),patch.object(rospy,'get_param',return_value=True):
                self.assertFalse(p._cloud_context()['ready'])
            previous=p.odom_receipt;p._on_odom(m);self.assertIs(p.odom_receipt,previous)

if __name__=='__main__':unittest.main()
