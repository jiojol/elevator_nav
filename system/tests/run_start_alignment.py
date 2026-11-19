#!/usr/bin/env python3
"""Real planner/gate integration on a private loopback master; no SDK/DDS."""
import argparse,json,math,os,signal,socket,subprocess,sys,time,xmlrpc.client
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]

def driver(case,out,params_file=None):
 import rospy,yaml
 from geometry_msgs.msg import PoseStamped,Twist
 from nav_msgs.msg import Odometry,Path as NavPath
 from sensor_msgs.point_cloud2 import create_cloud_xyz32
 from sensor_msgs.msg import PointCloud2
 from std_msgs.msg import Header,String,Empty,Bool
 from std_srvs.srv import Trigger,TriggerResponse
 from visualization_msgs.msg import Marker
 prefix=ROOT/'simulation/artifacts/local_build/devel';procs=[];handles=[]
 def launch(name,args):
  h=(out/(name+'.log')).open('w');handles.append(h);p=subprocess.Popen(args,env=os.environ.copy(),cwd=ROOT,stdout=h,stderr=subprocess.STDOUT,start_new_session=True);procs.append(p);return p
 try:
  master=launch('master',['/opt/ros/noetic/bin/roscore','-p',os.environ['TEST_ROS_PORT']]);api=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
  for _ in range(100):
   try:
    if api.getPid('/alignment_test')[0]==1:break
   except OSError:pass
   time.sleep(.05)
  rospy.init_node('alignment_test',disable_signals=True)
  params=yaml.safe_load(params_file.read_text() if params_file else subprocess.check_output(
   ['/opt/ros/noetic/bin/roslaunch','--dump-params',str(ROOT/'simulation/isolated.launch')],env=os.environ,text=True))
  for key,value in params.items():
   if key.startswith(('/scan_planner_node','/scan_command_gate','/dog_nav/geometry','/closed_loop_controller')):rospy.set_param(key,value)
  rospy.set_param('/scan_planner_node/grid_map/ground_height',-2.5);rospy.set_param('/scan_planner_node/grid_map/map_sliding_en',False)
  rospy.set_param('/body_pose_topic','/planning/body_odometry')
  stop_calls=[]
  service=rospy.Service('/inspection/stop',Trigger,lambda req:(stop_calls.append(time.monotonic()) or TriggerResponse(success=True,message='isolated test only')))
  commands=[];permits=[];permit_times=[];splines=[];acks=[];statuses=[];footprints=[]
  def permit_received(message):
   permits.append(json.loads(message.data));permit_times.append((time.monotonic(),permits[-1]))
  rospy.Subscriber('/dog_nav/footprint',Marker,lambda m:footprints.append(m) if m.ns=='scan_native_cylinders' else None)
  rospy.Subscriber('/cmd_vel',__import__('geometry_msgs.msg',fromlist=['Twist']).Twist,lambda m:commands.append((time.monotonic(),m.linear.x,m.angular.z)))
  rospy.Subscriber('/planning/start_alignment',String,permit_received)
  rospy.Subscriber('/planning/start_alignment_done',String,lambda m:acks.append(m.data))
  from scan_planner.msg import Bspline
  rospy.Subscriber('/planning/bspline',Bspline,splines.append)
  rospy.Subscriber('/planning/local_planner_status',String,lambda m:statuses.append(json.loads(m.data)))
  prior=rospy.Publisher('/plan_map/free',PointCloud2,queue_size=1,latch=True);body=rospy.Publisher('/planning/body_odometry',Odometry,queue_size=1)
  chassis=rospy.Publisher('/go2/odom',Odometry,queue_size=1);sensor=rospy.Publisher('/dog_nav/sensor_pose',Odometry,queue_size=1)
  cloud=rospy.Publisher('/navigation/obstacle_points',PointCloud2,queue_size=1);path=rospy.Publisher('/initial_path',NavPath,queue_size=1,latch=True);cancel=rospy.Publisher('/plan_map/cancel_scan',Empty,queue_size=1);replay=rospy.Publisher('/planning/start_alignment',String,queue_size=1);controller=rospy.Publisher('/planning/scan_cmd_vel',Twist,queue_size=1);heartbeat=rospy.Publisher('/planning/go2_execution_frozen',Bool,queue_size=1)
  launch('planner',[str(prefix/'lib/scan_planner/scan_planner_node'),'/grid_map/sensor_pose:=/dog_nav/sensor_pose','/grid_map/cloud:=/navigation/obstacle_points','/grid_map/body_pose:=/planning/body_odometry']);launch('gate',['/usr/bin/python3',str(ROOT/'nav/src/dog_nav_bringup/scripts/scan_command_gate.py'),'__name:=scan_command_gate'])
  for _ in range(100):
   if path.get_num_connections()>=2 and body.get_num_connections()>=2 and sensor.get_num_connections()>=1 and cloud.get_num_connections()>=2:break
   assert all(p.poll() is None for p in procs),'child process exited';time.sleep(.05)
  def sensors(yaw,rate,points):
   stamp=rospy.Time.now();q=Odometry();q.header=Header(stamp=stamp,frame_id='map');q.child_frame_id='base_link';q.pose.pose.position.z=.4;q.pose.pose.orientation.z=math.sin(yaw/2);q.pose.pose.orientation.w=math.cos(yaw/2);q.twist.twist.angular.z=rate
   body.publish(q);chassis.publish(q);sensor.publish(q);cloud.publish(create_cloud_xyz32(q.header,points));heartbeat.publish(Bool(data=False));controller.publish(Twist())
  prior.publish(create_cloud_xyz32(Header(stamp=rospy.Time.now(),frame_id='map'),[((x+.5)*.2,(y+.5)*.2,0.) for x in range(-16,17) for y in range(-7,8)]))
  wall=[(.625,(y+.5)*.05,.425) for y in range(-32,32)]
  cleared_wall=[(3.125,5*(y+dy),.4+5*(z+dz-.4)) for x,y,z in wall for dy in [-.015,0.,.015] for dz in [-.015,0.,.015]]
  for _ in range(8):sensors(0.,0.,wall if case=='recovery' else [(0.,2.,.4)]);time.sleep(.1)
  route=NavPath();route.header=Header(stamp=rospy.Time.now(),frame_id='map')
  for x in [0.,-.25,-.5,-1.,-1.5,-2.]:
   p=PoseStamped(header=route.header);p.pose.position.x=-x if case=='recovery' else x;p.pose.orientation.w=1.;route.poses.append(p)
  path.publish(route);yaw=rate=0.;started=time.monotonic();last=started;next_sense=0.;revoked=False;cancelled=False;acked_at=None
  while time.monotonic()-started<16.:
   now=time.monotonic();elapsed=now-started;dt=min(.05,now-last);last=now
   rate=commands[-1][2] if commands else 0.;yaw=math.atan2(math.sin(yaw+rate*dt),math.cos(yaw+rate*dt))
   if elapsed>=next_sense:
    obstacle=case=='dynamic' and 1.1<elapsed<2.2
    points=(wall if elapsed<2. else cleared_wall) if case=='recovery' else ([(.025,.325,.425)] if obstacle else [(.15,1.95,.55)])
    sensors(yaw,rate,points);next_sense=elapsed+.08
   if case=='cancel' and elapsed>.9 and not cancelled:cancel.publish(Empty());cancelled=True;cancel_at=now
   if any(not p['allowed'] and not p['cancel'] for p in permits):revoked=True
   if case=='cancel' and cancelled and now-cancel_at>.35:break
   if case=='recovery' and any(not s.obstacle_hold and len(s.pos_pts)>6 for s in splines):
    assert elapsed>=2.,'blocking wall was bypassed outside the prior';break
   if acks:
    if acked_at is None:acked_at=now
    # A delayed heartbeat from a completed action must not retake command ownership.
    if now-acked_at<.2:
     old=dict(permits[-1],stamp=rospy.Time.now().to_sec());replay.publish(String(data=json.dumps(old)))
    if now-acked_at>.4 and any(not s.obstacle_hold and len(s.pos_pts)>6 for s in splines):break
   assert all(p.poll() is None for p in procs),'child process exited';time.sleep(.01)
  (out/'permits.json').write_text(json.dumps(permits,indent=2)+'\n')
  if case!='recovery':assert permits,'planner never issued a start-rotation permit'
  assert all(abs(v)<1e-10 for _,v,_ in commands),'translation emitted during start rotation'
  radius=rospy.get_param('/scan_planner_node/grid_map/double_cylinder_radius')
  if params_file is None:assert abs(radius-.225)<1e-10,'launch radius was not updated'
  assert footprints and all(abs(m.scale.x-2*radius)<1e-10 and abs(m.scale.y-2*radius)<1e-10 for m in footprints),'visual and collision cylinder parameters differ'
  if case=='recovery':
   assert stop_calls,'blocked wall never caused a stop'
   assert not permits,'forward goal unexpectedly required reverse alignment'
   assert any(not s.obstacle_hold and len(s.pos_pts)>6 for s in splines),'no trajectory after the wall cleared'
   assert elapsed-2.<1.5,'recovery stayed blocked despite fresh free rays'
   assert any(s['reason']=='retry with fresh obstacle map' for s in statuses),'newly fused inputs did not trigger an early retry'
  elif case=='cancel':
   assert not acks,'cancelled action completed';assert all(abs(w)<1e-10 for t,v,w in commands if t>cancel_at+.15),'rotation continued after cancellation'
  else:
   assert acks and any(not s.obstacle_hold and len(s.pos_pts)>6 for s in splines),'no walking trajectory after alignment'
   assert abs(math.atan2(math.sin(math.pi-yaw),math.cos(math.pi-yaw)))<.3
   assert len(acks)==1,'completed request replayed'
   if case=='dynamic':
    assert revoked,'updated occupancy never revoked rotation'
    # Consecutive permit intervals are short; check against the whole denied span.
    first=next(t for t,p in permit_times if not p['allowed'])
    resumed=next(t for t,p in permit_times if t>first and p['allowed'])
    stopped_samples=[w for t,v,w in commands if first+.10<t<resumed]
    assert stopped_samples and max(abs(w) for w in stopped_samples)<1e-10,'rotation continued while blocked'
  if case!='recovery':assert not stop_calls,'unexpected execution failure in a stationary isolated test'
  report=dict(case=case,collision_radius=radius,recovery_latency_s=elapsed-2. if case=='recovery' else None,max_plan_attempt_ms=max((s.get('last_solve_ms',0.) for s in statuses),default=0.),alignment_permits=len(permits),acks=acks,walking_trajectory=any(not s.obstacle_hold and len(s.pos_pts)>6 for s in splines),yaw=yaw,seconds=time.monotonic()-started,obstacle_revoked=revoked,translation_commands=sum(abs(v)>1e-10 for t,v,w in commands),stop_service_calls=len(stop_calls),master=os.environ['ROS_MASTER_URI'],robot_connected=False)
  (out/'statuses.json').write_text(json.dumps(statuses,indent=2)+'\n');(out/'result.json').write_text(json.dumps(report,indent=2)+'\n');(out/'permits.json').write_text(json.dumps(permits,indent=2)+'\n');print(json.dumps(report),flush=True)
 finally:
  for p in reversed(procs):
   if p.poll() is None:
    os.killpg(p.pid,signal.SIGINT)
    try:p.wait(timeout=5)
    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
  for h in handles:h.close()

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--case',choices=['normal','dynamic','cancel','recovery'],default='normal');parser.add_argument('--driver',action='store_true');parser.add_argument('--params',type=Path,help='Optional recorded ROS parameter dump');args=parser.parse_args()
 out=ROOT/'simulation/artifacts/reverse_start_fix/integration'/args.case;out.mkdir(parents=True,exist_ok=True)
 if args.driver:driver(args.case,out,args.params);return
 with socket.socket() as sock:sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
 env=os.environ.copy();env.pop('ROS_HOSTNAME',None);env.pop('ROS_NAMESPACE',None)
 env.update(CMAKE_PREFIX_PATH=str(ROOT/'simulation/artifacts/local_build/devel')+':/opt/ros/noetic',ROS_PACKAGE_PATH=str(ROOT/'simulation/artifacts/local_build/devel/share')+':/opt/ros/noetic/share',ROS_MASTER_URI=f'http://127.0.0.1:{port}',ROS_IP='127.0.0.1',TEST_ROS_PORT=str(port),ROS_LOG_DIR=str(out/'ros_logs'),PYTHONPATH=str(ROOT/'simulation/artifacts/local_build/devel/lib/python3/dist-packages')+':/opt/ros/noetic/lib/python3/dist-packages',LD_LIBRARY_PATH=str(ROOT/'simulation/artifacts/local_build/devel/lib')+':'+str(ROOT/'simulation/artifacts/local_dependencies/tf/lib')+':/opt/ros/noetic/lib')
 subprocess.run(['/usr/bin/python3',__file__,'--driver','--case',args.case]+(['--params',str(args.params.resolve())] if args.params else []),cwd=ROOT,env=env,check=True,timeout=45)
if __name__=='__main__':main()
