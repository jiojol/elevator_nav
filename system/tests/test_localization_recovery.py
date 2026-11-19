"""Temporary localization gaps must hold, retain the goal, then acknowledge recovery."""
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
import sys,threading,unittest
from unittest.mock import patch
import rospy
from nav_msgs.msg import Odometry,Path as NavPath
from geometry_msgs.msg import PoseStamped
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'nav/src/dog_nav_bringup/scripts'))
from scan_command_gate import CommandGate
from start_alignment_policy import StartAlignmentPolicy

class LocalizationRecoveryTest(unittest.TestCase):
 def setUp(self):
  self.time_patch=patch('scan_command_gate.rospy.Time.now',return_value=rospy.Time(101));self.time_patch.start();self.addCleanup(self.time_patch.stop)
  self.mono_patch=patch('scan_command_gate.time.monotonic',return_value=1.);self.mono_patch.start();self.addCleanup(self.mono_patch.stop)
  g=self.g=CommandGate.__new__(CommandGate)
  g.lock=threading.RLock();g.shutting_down=False;g.phase='tracking';g.goal=(1.,0.,.4,0.);g.pose=(0.,0.,.4,0.)
  g.path_stamp=rospy.Time(100);g.cancel_stamp=rospy.Time(99);g.frame='map';g.body_height=.4
  g.start_alignment=StartAlignmentPolicy();g.curve_ready=True;g.command_received=.9
  g.collision_stop_pending=False;g.require_contact_feedback=False;g.contact_active=False;g.localization_pause=None
  g.odom_stamp=rospy.Time.from_sec(100.41);g.odom_received=.9;g.cloud_received=.1
  g.stopped_linear=.03;g.stopped_yaw=.08;g.collision_settle_time=.3;g.position_recovery_stamp=None
  g.turn_min=.15;g.turn_started=0.;g.yaw_holding=False
  self.commands=[];self.events=[];self.failures=[];self.status=[]
  g.stop=lambda:self.commands.append('zero');g.fail=self.failures.append
  g.collision_control_pub=SimpleNamespace(publish=lambda m:self.events.append(m.data))
  g.collision_status=lambda state,reason:self.status.append(state)
  g.chassis_feedback_msg=Odometry();g.chassis_feedback_msg.header.stamp=rospy.Time(101);g.chassis_feedback_received=1.
 def pause(self):
  self.g.watchdog(None);self.assertFalse(self.failures);self.assertTrue(self.g.collision_stop_pending)
  self.assertEqual(self.g.phase,'blocked');self.assertEqual(self.events,['hold']);self.assertEqual(self.g.path_stamp,rospy.Time(100))
 def acknowledged(self):
  f=Future();f.set_result((True,'ack'));self.g.stop_future=f;self.g.stop_feedback_since=.5
 def test_header_age_gap_holds_without_permanent_goal_failure(self):
  self.pause();self.assertIn('WAITING_LOCALIZATION',self.status)
 def test_fresh_pose_and_physical_stop_release_same_goal_for_new_curve(self):
  self.pause();self.acknowledged();self.g.odom_stamp=rospy.Time(101);self.g.collision_watchdog(1.)
  self.assertEqual(self.events,['hold','replan']);self.assertFalse(self.g.collision_stop_pending);self.assertFalse(self.g.curve_ready)
  self.assertEqual(self.g.goal,(1.,0.,.4,0.));self.assertIn('LOCALIZATION_RECOVERED',self.status)
 def test_stale_pose_keeps_hold_even_after_stop_ack(self):
  self.pause();self.acknowledged();self.g.collision_watchdog(1.);self.assertTrue(self.g.collision_stop_pending);self.assertNotIn('replan',self.events)
 def test_stop_rejection_cannot_be_bypassed_by_fresh_pose(self):
  self.pause();f=Future();f.set_result((False,'rejected'));self.g.stop_future=f;self.g.odom_stamp=rospy.Time(101)
  self.g.collision_watchdog(1.);self.assertTrue(self.g.collision_stop_pending);self.assertNotIn('replan',self.events)
 def test_cancel_during_pause_never_resumes_the_old_goal(self):
  self.pause();self.g.cancel(None);self.assertIsNone(self.g.localization_pause)
  self.acknowledged();self.g.odom_stamp=rospy.Time(101);self.g.collision_watchdog(1.)
  self.assertEqual(self.g.phase,'idle');self.assertIsNone(self.g.goal);self.assertNotIn('LOCALIZATION_RECOVERED',self.status)
 def test_final_turn_resumes_settling_without_waiting_for_finished_fsm_goal(self):
  self.g.phase='turning';self.pause();self.acknowledged();self.g.odom_stamp=rospy.Time(101);self.g.collision_watchdog(1.)
  self.assertEqual(self.g.phase,'settling');self.assertFalse(self.g.yaw_holding)
 def test_new_goal_during_pause_does_not_resume_old_final_turn(self):
  self.g.phase='turning';self.pause();path=NavPath();path.header.frame_id='map';path.header.stamp=rospy.Time.from_sec(100.5)
  pose=PoseStamped();pose.pose.position.x=2.;pose.pose.orientation.w=1.;path.poses.append(pose);self.g.path(path)
  self.acknowledged();self.g.odom_stamp=rospy.Time(101);self.g.collision_watchdog(1.)
  self.assertEqual(self.g.phase,'blocked');self.assertEqual(self.g.goal[0],2.);self.assertNotIn('LOCALIZATION_RECOVERED',self.status)

if __name__=='__main__':unittest.main()
