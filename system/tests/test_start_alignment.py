import json,math,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'nav/src/dog_nav_bringup/scripts'))
from start_alignment_policy import StartAlignmentPolicy

def event(angle=math.pi,stamp=0.,allowed=True):
 return dict(goal_stamp_ns=1234567890123456789,request_id='goal:1',frame='map',stamp=stamp,allowed=allowed,cancel=False,start_yaw=0.,current_yaw=0.,remaining_angle=angle,target_yaw=StartAlignmentPolicy.wrap(angle),signed_angle=angle,x=0.,y=0.,z=.4)
class StartAlignmentTest(unittest.TestCase):
 def receive(self,p,e,t,pose=(0.,0.,.4,0.)):
  e=dict(e,current_yaw=pose[3],remaining_angle=e['signed_angle']-StartAlignmentPolicy.wrap(pose[3]-e['start_yaw']))
  return p.receive(e,1234567890123456789,'map',pose,t,t)
 def test_reverse_goal_finishes_from_rest_without_translation(self):
  for angle in [math.pi,-math.pi,2.8,-2.8]:
   p=StartAlignmentPolicy();yaw=0.;rate=0.;done=False
   for i in range(900):
    t=i*.02;pose=(0.,0.,.4,yaw)
    if i%3==0:self.assertTrue(self.receive(p,event(angle,t),t,pose))
    rate,state=p.step(pose,abs(rate)<.08,t,t)
    self.assertLessEqual(abs(rate),.5+1e-9)
    yaw=StartAlignmentPolicy.wrap(yaw+rate*.02)
    if state=='done':done=True;break
   self.assertTrue(done,(angle,state,p.remaining))
   self.assertLess(abs(StartAlignmentPolicy.wrap(yaw-angle)),.21)
 def test_faster_start_profile_reduces_half_turn_time(self):
  durations=[]
  for p in [StartAlignmentPolicy(.4,.15,.8),StartAlignmentPolicy()]:
   yaw=rate=0.
   for i in range(900):
    t=i*.02;pose=(0.,0.,.4,yaw)
    if i%3==0:self.receive(p,event(math.pi,t),t,pose)
    rate,state=p.step(pose,abs(rate)<.08,t,t)
    yaw=StartAlignmentPolicy.wrap(yaw+rate*.02)
    if state=='done':durations.append(t);break
  self.assertEqual(len(durations),2)
  self.assertLess(durations[1],.85*durations[0])
 def test_wrong_goal_frame_and_stale_permit_cannot_start(self):
  p=StartAlignmentPolicy()
  self.assertFalse(p.receive(event(),1,'map',(0,0,.4,0),0,0))
  self.assertFalse(p.receive(event(),1234567890123456789,'odom',(0,0,.4,0),0,0))
  self.assertFalse(self.receive(p,event(),1.))
  self.assertIsNone(p.intent)
 def test_obstacle_revokes_permission_and_timeout_stops(self):
  p=StartAlignmentPolicy();self.receive(p,event(),0)
  for i in range(1,6):rate,state=p.step((0,0,.4,0),True,i*.02,i*.02)
  self.assertGreater(rate,0.)
  self.receive(p,event(stamp=.1,allowed=False),.1)
  self.assertEqual(p.step((0,0,.4,0),False,.12,.12)[0],0.)
  self.receive(p,event(stamp=.15),.15)
  self.assertEqual(p.step((0,0,.4,0),True,.5,.5),(0.,'waiting_permission'))
 def test_body_motion_and_localization_jump_invalidate_turn(self):
  p=StartAlignmentPolicy();self.receive(p,event(),0)
  self.assertEqual(p.step((.13,0,.4,0),True,.02,.02)[1],'position_displaced')
  self.assertEqual(p.step((0,0,.4,1.),True,.02,.02)[1],'pose_reset')
 def test_start_waits_for_chassis_stop_and_done_waits_for_stop(self):
  p=StartAlignmentPolicy();self.receive(p,event(),0)
  self.assertEqual(p.step((0,0,.4,0),False,.02,.02),(0.,'waiting_stop'))
  self.assertGreater(p.step((0,0,.4,0),True,.04,.04)[0],0.)
 def test_cancel_and_replayed_old_goal_have_no_authority(self):
  p=StartAlignmentPolicy();self.receive(p,event(),0);p.cancel()
  self.assertEqual(p.step((0,0,.4,0),True,.02,.02),(0.,'inactive'))
  e=event();e['goal_stamp_ns']-=1;self.assertFalse(self.receive(p,e,0))
 def test_heartbeat_does_not_reset_accumulated_rotation(self):
  p=StartAlignmentPolicy();self.receive(p,event(),0)
  for i in range(1,8):
   pose=(0,0,.4,i*.05);self.receive(p,event(stamp=i*.02),i*.02,pose);p.step(pose,True,i*.02,i*.02)
  self.assertAlmostEqual(p.remaining,math.pi-.35)
 def test_changed_target_under_same_request_is_rejected(self):
  p=StartAlignmentPolicy();self.receive(p,event(),0);e=event(angle=2.8,stamp=.05)
  self.assertFalse(self.receive(p,e,.05))
if __name__=='__main__':unittest.main()
