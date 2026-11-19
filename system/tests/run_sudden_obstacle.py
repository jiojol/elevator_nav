#!/usr/bin/env python3
"""Isolated real planner/controller/gate with simulated chassis braking; no SDK."""
import argparse,json,math,os,signal,socket,subprocess,time,xmlrpc.client
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
def driver(case,out):
 import rospy,yaml
 from geometry_msgs.msg import PoseStamped,Twist
 from nav_msgs.msg import Odometry,Path as NavPath
 from sensor_msgs.msg import PointCloud2
 from sensor_msgs.point_cloud2 import create_cloud_xyz32
 from std_msgs.msg import Header,String
 from std_srvs.srv import Trigger,TriggerResponse
 from scan_planner.msg import Bspline
 prefix=ROOT/'simulation/artifacts/local_build/devel';procs=[];logs=[];cruise=os.environ.get('SUDDEN_CRUISE')=='1'
 def launch(name,args):
  h=(out/(name+'.log')).open('w');logs.append(h);p=subprocess.Popen(args,env=os.environ.copy(),cwd=ROOT,stdout=h,stderr=subprocess.STDOUT,start_new_session=True);procs.append(p);return p
 try:
  launch('master',['/opt/ros/noetic/bin/roscore','-p',os.environ['TEST_ROS_PORT']]);api=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
  for _ in range(100):
   try:
    if api.getPid('/sudden_test')[0]==1:break
   except OSError:pass
   time.sleep(.05)
  rospy.init_node('sudden_test',disable_signals=True)
  params=yaml.safe_load(subprocess.check_output(['/opt/ros/noetic/bin/roslaunch','--dump-params',str(ROOT/'simulation/isolated.launch')],env=os.environ,text=True))
  for k,v in params.items():rospy.set_param(k,v)
  rospy.set_param('/closed_loop_controller/max_vyaw',.5);rospy.set_param('/scan_planner_node/grid_map/ground_height',-2.5);rospy.set_param('/scan_planner_node/grid_map/map_sliding_en',cruise)
  commands=[];statuses=[];stops=[];splines=[];goals=[];visualizations=[];failures=[]
  if os.environ.get('SUDDEN_VIS')=='1':
   for topic in ['/grid_map/occupancy','/grid_map/occupancy_inflate']:
    rospy.Subscriber(topic,PointCloud2,lambda m:visualizations.append((time.monotonic(),m.header.stamp.to_sec(),m.width)),queue_size=1)
  def stop_request(req):stops.append(time.monotonic());return TriggerResponse(success=True,message='isolated simulated chassis only')
  rospy.Service('/inspection/stop',Trigger,stop_request)
  rospy.Subscriber('/cmd_vel',Twist,lambda m:commands.append((time.monotonic(),m.linear.x,m.angular.z)))
  rospy.Subscriber('/planning/local_planner_status',String,lambda m:statuses.append((time.monotonic(),json.loads(m.data))))
  rospy.Subscriber('/planning/bspline',Bspline,lambda m:splines.append((time.monotonic(),m)))
  rospy.Subscriber('/planning/goal_reached',Header,lambda m:goals.append(time.monotonic()))
  rospy.Subscriber('/planning/goal_failed',Header,lambda m:failures.append(time.monotonic()))
  body=rospy.Publisher('/planning/body_odometry',Odometry,queue_size=1);chassis=rospy.Publisher('/go2/odom',Odometry,queue_size=1)
  sensor=rospy.Publisher('/dog_nav/sensor_pose',Odometry,queue_size=1);cloud=rospy.Publisher('/navigation/obstacle_points',PointCloud2,queue_size=1)
  prior=rospy.Publisher('/plan_map/free',PointCloud2,queue_size=1,latch=True);path=rospy.Publisher('/initial_path',NavPath,queue_size=1,latch=True)
  launch('planner',[str(prefix/'lib/scan_planner/scan_planner_node'),'/grid_map/sensor_pose:=/dog_nav/sensor_pose','/grid_map/cloud:=/navigation/obstacle_points','/grid_map/body_pose:=/planning/body_odometry'])
  launch('controller',[str(prefix/'lib/scan_planner/closed_loop_controller'),'cmd_vel:=/planning/scan_cmd_vel','__name:=closed_loop_controller'])
  launch('gate',['/usr/bin/python3',str(ROOT/'nav/src/dog_nav_bringup/scripts/scan_command_gate.py'),'__name:=scan_command_gate'])
  for _ in range(150):
   if body.get_num_connections()>=3 and path.get_num_connections()>=2 and cloud.get_num_connections()>=2:break
   assert all(p.poll() is None for p in procs),('child exited: '+str([p.poll() for p in procs]));time.sleep(.05)
  x=y=yaw=v=w=0.;obstacle=None;introduced=None;removed=None;next_sense=0.;minimum_gap=100.;samples=[];last=time.monotonic()
  gap_start=None;gap_end=None
  def sensors(points,send_cloud=True,send_pose=True):
   q=Odometry();q.header=Header(stamp=rospy.Time.now(),frame_id='map');q.child_frame_id='base_link';q.pose.pose.position.x=x;q.pose.pose.position.y=y;q.pose.pose.position.z=.4;q.pose.pose.orientation.z=math.sin(yaw/2);q.pose.pose.orientation.w=math.cos(yaw/2);q.twist.twist.linear.x=v;q.twist.twist.angular.z=w
   chassis.publish(q)
   if send_pose:body.publish(q);sensor.publish(q)
   if send_cloud:cloud.publish(create_cloud_xyz32(q.header,points))
  prior.publish(create_cloud_xyz32(Header(stamp=rospy.Time.now(),frame_id='map'),[((a+.5)*.2,(b+.5)*.2,0.) for a in range(-16,40 if cruise else 25) for b in range(-9,10)]))
  for _ in range(10):sensors([(4.,1.7,.4)]);time.sleep(.08)
  route=NavPath();route.header=Header(stamp=rospy.Time.now(),frame_id='map')
  for xx in ([0.,.5,1.,2.,3.,4.,6.] if cruise else [0.,.5,1.,2.,3.,4.]):
   p=PoseStamped(header=route.header);p.pose.position.x=xx;p.pose.orientation.w=1.;route.poses.append(p)
  path.publish(route);started=last=time.monotonic()
  while time.monotonic()-started<24.:
   now=time.monotonic();dt=min(.03,now-last);last=now;elapsed=now-started
   target_v,target_w=commands[-1][1:] if commands and now-commands[-1][0]<.12 else (0.,0.)
   v+=max(-.5*dt,min(.5*dt,target_v-v));w+=max(-1.6*dt,min(1.6*dt,target_w-w))
   yaw=math.atan2(math.sin(yaw+w*dt),math.cos(yaw+w*dt));x+=v*math.cos(yaw)*dt;y+=v*math.sin(yaw)*dt
   if case!='clear' and obstacle is None and v>(.79 if cruise else .73) and x>(1.2 if cruise else .6):
    # At cruising speed, 1.10 m to the point is inside the braking window
    # after accounting for body length and the two-hit occupancy update.
    obstacle=(x+(2.4 if case=='far' else (1.35 if cruise else 1.10)),y);introduced=now;introduced_speed=v
   if obstacle is not None and removed is None:
    gap=min(math.hypot(obstacle[0]-x-off*math.cos(yaw),obstacle[1]-y-off*math.sin(yaw))-.225 for off in [-.18,.18]);minimum_gap=min(minimum_gap,gap)
    if case=='near' and stops and abs(v)<.03 and now-stops[-1]>.5:removed=now
   if now>=next_sense:
    points=[(x+4.,y+1.7,.4)]
    if obstacle is not None:
     if removed is None:points.append((obstacle[0],obstacle[1],.425))
     else:
      d=math.hypot(obstacle[0]-x,obstacle[1]-y);factor=3./max(.01,d)
      points += [(x+factor*(obstacle[0]-x),y+factor*(obstacle[1]-y+dy),.4+factor*.025) for dy in [-.015,0,.015]]
    if (os.environ.get('SUDDEN_GAP')=='1' or os.environ.get('SUDDEN_POSE_GAP')=='1') and gap_start is None and x>.1:gap_start=now;gap_end=now+.7
    in_gap=gap_start is not None and gap_start<=now<gap_end
    sensors(points,not in_gap,not(in_gap and os.environ.get('SUDDEN_POSE_GAP')=='1'));next_sense=now+.08
   samples.append([elapsed,x,y,yaw,v,w,target_v,target_w])
   if goals:break
   if case!='far' and obstacle is not None and x>obstacle[0]+.4:break
   if case=='far' and obstacle and x>obstacle[0]+.4:break
   assert all(p.poll() is None for p in procs),('child exited: '+str([p.poll() for p in procs]));time.sleep(.01)
  report=dict(case=case,cruise=cruise,introduction_speed=locals().get('introduced_speed'),minimum_gap=minimum_gap,stop_requests=len(stops),elapsed=time.monotonic()-started,obstacle=obstacle,final_xy=[x,y],recovered=bool(x>3.5) if case=='clear' else bool(obstacle and x>obstacle[0]+.4),robot_connected=False)
  report['visualization_messages']=len(visualizations)
  if gap_start is not None:
   report['cloud_gap_s']=gap_end-gap_start
   report['stops_during_cloud_gap']=sum(gap_start<=t<gap_end for t in stops)
   if os.environ.get('SUDDEN_POSE_GAP')!='1':assert report['stops_during_cloud_gap']==0,'cloud age caused an artificial stop'
  if os.environ.get('SUDDEN_VIS')=='1':assert visualizations,'visualization subscriber test received no map'
  if os.environ.get('SUDDEN_POSE_GAP')=='1':
   assert not failures,'temporary localization gap permanently failed the goal'
   assert stops,'missing localization did not request a confirmed stop'
   assert report['recovered'],'same goal did not resume after fresh localization'
  if introduced:
   zero=next((t for t,vv,ww in commands if t>introduced and abs(vv)<1e-8),None);report['zero_after_introduction_s']=zero-introduced if zero else None
  (out/'result.json').write_text(json.dumps(report,indent=2)+'\n');(out/'samples.json').write_text(json.dumps(samples));(out/'statuses.json').write_text(json.dumps(statuses,indent=2));print(json.dumps(report),flush=True)
  assert obstacle is not None or case=='clear','test never reached cruising speed'
  assert minimum_gap>=0.,'simulated chassis contacted the sudden obstacle'
  if case!='far':
   assert report['recovered'],'near obstacle was neither bypassed nor recovered after a necessary stop'
   assert not any(s['state']=='ALIGN_START' for t,s in statuses),'forward-route recovery turned back toward stale progress'
  else:assert (not stops or os.environ.get('SUDDEN_POSE_GAP')=='1') and report['recovered'],'far obstacle was not bypassed while preserving motion'
 finally:
  for p in reversed(procs):
   if p.poll() is None:
    os.killpg(p.pid,signal.SIGINT)
    try:p.wait(timeout=5)
    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
  for f in logs:f.close()
def main():
 p=argparse.ArgumentParser();p.add_argument('--case',choices=['clear','near','near_persistent','far'],default='near');p.add_argument('--label',default='fixed');p.add_argument('--driver',action='store_true');a=p.parse_args();out=ROOT/'simulation/artifacts/sudden_obstacle'/a.label/a.case;out.mkdir(parents=True,exist_ok=True)
 if a.driver:driver(a.case,out);return
 with socket.socket() as sock:sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
 prefix=ROOT/'simulation/artifacts/local_build/devel';env=os.environ.copy();env.pop('ROS_HOSTNAME',None);env.pop('ROS_NAMESPACE',None)
 env.update(ROS_MASTER_URI=f'http://127.0.0.1:{port}',ROS_IP='127.0.0.1',TEST_ROS_PORT=str(port),ROS_LOG_DIR=str(out/'ros_logs'),CMAKE_PREFIX_PATH=str(prefix)+':/opt/ros/noetic',ROS_PACKAGE_PATH=str(prefix/'share')+':/opt/ros/noetic/share',PYTHONPATH=str(prefix/'lib/python3/dist-packages')+':/opt/ros/noetic/lib/python3/dist-packages',LD_LIBRARY_PATH=str(prefix/'lib')+':'+str(ROOT/'simulation/artifacts/local_dependencies/tf/lib')+':/opt/ros/noetic/lib')
 subprocess.run(['/usr/bin/python3',__file__,'--driver','--case',a.case,'--label',a.label],cwd=ROOT,env=env,check=True,timeout=55)
if __name__=='__main__':main()
