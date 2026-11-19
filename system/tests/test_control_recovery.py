"""Stop/replan must restore the arbiter only for a still-active stopped goal."""
import importlib.util
from pathlib import Path
import sys
import time
import unittest
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock, patch
import rospy
from nav_msgs.msg import Odometry
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'nav/src/dog_nav_bringup/scripts'))
from scan_command_gate import CommandGate

class ControlRecoveryTest(unittest.TestCase):
    def gate(self, active=True):
        gate=CommandGate.__new__(CommandGate);now=time.monotonic()
        gate.stop=Mock();gate.stop_future=Future();gate.stop_future.set_result((True,'stopped'))
        gate.stop_feedback_since=now-1;gate.collision_settle_time=.3
        gate.chassis_feedback_msg=Odometry();gate.chassis_feedback_msg.header.stamp=rospy.Time.from_sec(10.)
        gate.chassis_feedback_received=now;gate.stopped_linear=.01;gate.stopped_yaw=.03
        gate.contact_active=False;gate.require_contact_feedback=False
        gate.fresh_pose=lambda:True;gate.has_obstacle_input=lambda:True
        gate.path_stamp=rospy.Time.from_sec(9.) if active else None
        gate.goal=(0.,0.,0.) if active else None;gate.phase='blocked' if active else 'idle'
        gate.resume_service_name='/mission/claim_global';gate.resume_service=Mock(return_value=SimpleNamespace(success=True,message='global'))
        gate.collision_stop_pending=True;gate.collision_control_pub=Mock();gate.collision_status=Mock();gate.collision_reason='obstacle'
        gate.position_recovery_stamp=None;gate.fail=Mock()
        return gate,now

    @patch('scan_command_gate.rospy.wait_for_service')
    @patch('scan_command_gate.rospy.Time.now',return_value=rospy.Time.from_sec(10.))
    def test_active_stopped_goal_reclaims_before_replan(self,*_):
        gate,now=self.gate();order=[]
        gate.resume_service.side_effect=lambda:(order.append('claim') or SimpleNamespace(success=True))
        gate.collision_control_pub.publish.side_effect=lambda _:order.append('replan')
        gate.collision_watchdog(now)
        self.assertEqual(order,['claim','replan']);self.assertFalse(gate.curve_ready)
        self.assertFalse(gate.collision_stop_pending)

    @patch('scan_command_gate.rospy.Time.now',return_value=rospy.Time.from_sec(10.))
    def test_cancelled_goal_does_not_reclaim(self,*_):
        gate,now=self.gate(False);gate.collision_watchdog(now);gate.resume_service.assert_not_called()

    @patch('scan_command_gate.rospy.wait_for_service')
    @patch('scan_command_gate.rospy.Time.now',return_value=rospy.Time.from_sec(10.))
    def test_arbiter_rejection_keeps_stop_and_rejects_replan(self,*_):
        gate,now=self.gate();gate.resume_service.return_value=SimpleNamespace(success=False,message='Arm motion lock held')
        gate.collision_watchdog(now);self.assertTrue(gate.collision_stop_pending)
        gate.collision_control_pub.publish.assert_not_called();gate.fail.assert_called_once()

    @patch('scan_command_gate.rospy.Time.now',return_value=rospy.Time.from_sec(10.))
    def test_moving_body_cannot_reclaim(self,*_):
        gate,now=self.gate();gate.chassis_feedback_msg.twist.twist.linear.x=.1
        gate.collision_watchdog(now);gate.resume_service.assert_not_called()
        gate.collision_control_pub.publish.assert_not_called()

    @patch('scan_command_gate.rospy.Time.now',return_value=rospy.Time.from_sec(10.))
    def test_legacy_without_arbiter_keeps_original_path(self,*_):
        gate,now=self.gate();gate.resume_service_name='';gate.collision_watchdog(now)
        gate.resume_service.assert_not_called();gate.collision_control_pub.publish.assert_called_once()

if __name__=='__main__':unittest.main()
