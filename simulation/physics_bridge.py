#!/usr/bin/env python3
"""MuJoCo hover-body or Go2 ONNX locomotion on an isolated ROS master.

No Unitree SDK, DDS interface or hardware navigation launch is imported.
Hover mode integrates commands over terrain; dynamic mode uses actuator forces.
"""
import argparse
import hashlib
import json
import math
import os
import sys
import time
from pathlib import Path
import xml.etree.ElementTree as ET
os.environ.setdefault('MUJOCO_GL', 'egl')
import mujoco
import numpy as np
import rospy
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, Bool, String, Empty
from std_srvs.srv import Trigger, TriggerResponse
from scan_planner.msg import Bspline
from scipy.interpolate import BSpline
from prior_navigation import PriorNavigation
from path_smoothing import smooth_free_path
from hover_motion import HoverMotion, BODY_HALF

MOUNT_T=np.array([.221454,.018929,.133359])
MOUNT_R=Rotation.from_quat([.011587544788341213,.1274579512112959,.0001888026941301217,.9917762670241408])


def scene(assets,case,output,motion='dynamic'):
    if motion=='hover':
        root=ET.Element('mujoco',model='go2_hover')
        ET.SubElement(root,'option',timestep='.004',gravity='0 0 0')
        world=ET.SubElement(root,'worldbody')
        base=ET.SubElement(world,'body',name='base')
        ET.SubElement(base,'freejoint')
        size=' '.join(map(str,BODY_HALF))
        ET.SubElement(base,'geom',name='base_hover_collision',type='box',size=size,group='3',mass='15')
        ET.SubElement(base,'geom',name='hover_visual',type='box',size=size,group='1',contype='0',conaffinity='0',mass='0',rgba='.06 .5 .44 1')
        ET.SubElement(base,'site',name='lidar',size='.025',rgba='.15 .25 .3 1')
    else:root=ET.parse(assets/'models/unitree_go2/go2_mjx.xml').getroot()
    for element in root.iter():
        if 'file' in element.attrib:
            element.set('file',str((assets/'models/unitree_go2'/element.get('file')).resolve()))
    lidar=next(s for s in root.iter('site') if s.get('name')=='lidar')
    lidar.set('pos',' '.join(map(str,MOUNT_T)))
    q=MOUNT_R.as_quat();lidar.set('quat',' '.join(map(str,[q[3],q[0],q[1],q[2]])))
    world=root.find('worldbody')
    # All robot geometry is excluded from LiDAR, not from physics contacts.
    if motion=='dynamic':
        for body in world.findall('body'):
            for g in body.iter('geom'):g.set('group','3')
    ET.SubElement(world,'geom',name='floor',type='plane',size='7 4 .1',pos='0 0 0',friction='1.2 .005 .0001',rgba='.7 .7 .7 1')
    boxes=[]
    def box(name,center,half):
        ET.SubElement(world,'geom',name=name,type='box',pos=' '.join(map(str,center)),size=' '.join(map(str,half)),group='0',rgba='.5 .6 .7 1')
        boxes.append((name,np.array(center,dtype=float),np.array(half,dtype=float)))
    box('wall_n',(0,3,1),(6,.08,1));box('wall_s',(0,-3,1),(6,.08,1))
    box('wall_e',(6,0,1),(.08,3,1));box('wall_w',(-6,0,1),(.08,3,1))
    start=np.array([-2.5,0.,.32]);goal=np.array([2.5,0.,0.]);goal_yaw=0.
    if case=='final_yaw':goal_yaw=math.pi*.92
    if case=='large_obstacle':box('static_box',(0,0,.4),(.5,.55,.4))
    if case.startswith('door'):
        gap=.65 if case=='door_065' else .55
        box('wall_door_n',(0,(3+gap/2)/2,.8),(.10,(3-gap/2)/2,.8))
        box('wall_door_s',(0,-(3+gap/2)/2,.8),(.10,(3-gap/2)/2,.8))
    if case=='blocking_then_clear':
        box('wall_corridor_n',(0,.48,.8),(6,.08,.8));box('wall_corridor_s',(0,-.48,.8),(6,.08,.8))
    if case=='stairs':
        start=np.array([-.8,0.,.32]);goal=np.array([4.4,0.,1.50])
        for i in range(12):box('stair_%02d'%i,(.15+.3*i,0.,.06+.12*i),(.15,.8,.06))
        box('landing',(4.15,0.,1.44),(.55,.9,.06))
        box('wall_stair_n',(1.8,.86,.75),(2.,.04,.75));box('wall_stair_s',(1.8,-.86,.75),(2.,.04,.75))
    dynamic=ET.SubElement(world,'body',name='dynamic_box',mocap='true',pos='0 8 .4')
    ET.SubElement(dynamic,'geom',name='dynamic_box_geom',type='box',size='.22 .22 .4',group='0',rgba='.8 .2 .2 1')
    root.find('option').set('integrator','implicitfast');root.find('option').set('iterations','10')
    path=output/'scene.xml';ET.ElementTree(root).write(path,encoding='unicode')
    return path,boxes,start,goal,goal_yaw


def surface_points(boxes):
    arrays=[]
    grid=np.array([(x,y,0.) for x in np.arange(-6.,6.01,.12) for y in np.arange(-3.,3.01,.12)])
    arrays.append(grid)
    for _,center,half in boxes:
        for axis in range(3):
            others=[i for i in range(3) if i!=axis]
            for sign in (-1,1):
                for a in np.arange(-half[others[0]],half[others[0]]+.05,.12):
                    for b in np.arange(-half[others[1]],half[others[1]]+.05,.12):
                        p=center.copy();p[axis]+=sign*half[axis];p[others[0]]+=a;p[others[1]]+=b;arrays.append(p[None])
    return np.vstack(arrays)


class Simulation:
    def __init__(self,args):
        self.args=args;self.output=args.output;self.output.mkdir(parents=True,exist_ok=True)
        self.hover_mode=args.motion=='hover'
        self.navigation_height=args.hover_height+MOUNT_T[2] if self.hover_mode else .4
        self.xml,self.boxes,start,self.goal,self.goal_yaw=scene(args.assets,args.case,self.output,args.motion)
        if self.hover_mode:start[2]=self.hover_support(*start[:2])+args.hover_height
        self.prior=PriorNavigation(args.case,self.boxes,self.floor)
        rospy.set_param('/plan_map/resolution',self.prior.resolution)
        np.save(self.output/'prior_free.npy',self.prior.points)
        (self.output/'prior_meta.json').write_text(json.dumps({'source':'scene-derived fixed traversability corridor','resolution':self.prior.resolution,'free_voxels':len(self.prior.points),'global_search':'map_for_plan.plan_graph.FreeGraph','global_graph_nodes':len(self.prior.graph.points)},indent=2))
        self.model=mujoco.MjModel.from_xml_path(str(self.xml));self.data=mujoco.MjData(self.model)
        self.cmd=np.zeros(3,dtype=np.float32);self.command_at=0.;self.zero_since=None
        self.policy=None;self.policy_sha=None
        if not self.hover_mode:
            import onnxruntime as ort
            sys.path.insert(0,str(args.assets/'examples'))
            from cpp_deploy_controller import CppDeployController,DEFAULT_POLICY_DIR
            self.policy=CppDeployController(self.model,n_substeps=5,stand=True,keyboard_cmd=self,backend='cpu')
            options=ort.SessionOptions();options.intra_op_num_threads=1;options.inter_op_num_threads=1
            self.policy.session=ort.InferenceSession(str(DEFAULT_POLICY_DIR/'exported/policy.onnx'),sess_options=options,providers=['CPUExecutionProvider'])
            self.policy_sha=hashlib.sha256((DEFAULT_POLICY_DIR/'exported/policy.onnx').read_bytes()).hexdigest()
            self.data.qpos[7:]=self.policy.default_dof;self.data.ctrl[:]=self.policy.default_dof
        self.data.qpos[:3]=start;self.data.qpos[3:7]=[1,0,0,0]
        self.hover=HoverMotion(self.model,self.data,self.hover_support,args.hover_height) if self.hover_mode else None
        mujoco.mj_forward(self.model,self.data)
        self.base=int(self.model.body('base').id);self.mocap=int(self.model.body_mocapid[self.model.body('dynamic_box').id])
        self.trajectory=[];self.events=[];self.received_goal=False;self.result=None;self.goal_stamp=None;self.sent=False;self.first_trajectory=None;self.recovery_requested=False;self.recoveries=0
        self.interactive=args.interactive;self.manual_hold=self.interactive;self.goal_active=False
        self.view=self.live_renderer=None;self.ui_status='等待站稳';self.ready_at=2.;self.fallen=False
        self.requested_target=None;self.goal_history=[];self.goal_outcome=None;self.curve=[];self.reference=[];self.reset_count=0;self.frame_id=0;self.trail_start=0
        self.counts={'body_contacts':0,'obstacle_contacts':0,'position_reached':0,'position_lost':0,'goals_reached':0,'goals_failed':0}
        self.last_sensor=None;self.last_R=None;self.last_body=None;self.last_pub_time=None;self.last_cmd=np.zeros(3);self.contact_active=False
        self.pose_pub=rospy.Publisher('/dog_nav/odometry',Odometry,queue_size=1)
        self.sensor_pub=rospy.Publisher('/dog_nav/sensor_pose',Odometry,queue_size=1)
        self.chassis_pub=rospy.Publisher('/go2/odom',Odometry,queue_size=1)
        self.cloud_pub=rospy.Publisher('/navigation/obstacle_points',PointCloud2,queue_size=1)
        self.background_pub=rospy.Publisher('/globalmap',PointCloud2,queue_size=1,latch=True)
        self.free_pub=rospy.Publisher('/plan_map/free',PointCloud2,queue_size=1,latch=True)
        self.path_pub=rospy.Publisher('/plan_map/global_path',NavPath,queue_size=1,latch=True)
        self.contact_pub=rospy.Publisher('/dog_nav/contact_stop',Bool,queue_size=1)
        self.cancel_pub=rospy.Publisher('/plan_map/cancel_scan',Empty,queue_size=1)
        rospy.Subscriber('/cmd_vel',Twist,self.velocity,queue_size=1)
        rospy.Subscriber('/planning/goal_reached',Header,self.reached,queue_size=10)
        rospy.Subscriber('/planning/goal_failed',Header,self.failed,queue_size=10)
        rospy.Subscriber('/planning/goal_position_reached',Header,lambda m:self.event('position_reached',{'stamp':m.stamp.to_sec()}),queue_size=10)
        rospy.Subscriber('/planning/goal_position_lost',Header,self.position_lost,queue_size=10)
        rospy.Subscriber('/planning/local_planner_status',String,lambda m:self.message('planner',m),queue_size=100)
        rospy.Subscriber('/dog_nav/safety_stop',String,lambda m:self.message('safety',m),queue_size=10)
        rospy.Subscriber('/dog_nav/collision_status',String,lambda m:self.message('collision',m),queue_size=100)
        rospy.Subscriber('/planning/tracking_status',String,lambda m:self.message('tracking',m),queue_size=100)
        if self.interactive:
            rospy.Subscriber('/planning/bspline',Bspline,self.visual_curve,queue_size=1)
            rospy.Subscriber('/planning/tracking_path',NavPath,self.visual_reference,queue_size=1)
        self.service=rospy.Service('/inspection/stop',Trigger,self.stop_service)
        rng=np.random.default_rng(13);self.theta=rng.uniform(-math.pi,math.pi,2048).astype(np.float32);self.phi=rng.uniform(-.8,.5,2048).astype(np.float32)
        self.snapshots={};self.snapshot_sub=None;self.map_snapshot_requested=False;self.states=[];self.mocap_states=[];self.roots=[];self.initial=start;self.goal_published_at=None
        if self.interactive:
            from interactive_view import InteractiveView, LiveRenderer
            self.view=InteractiveView(args.ui_port,self.boxes,args.case,self.prior.validate_body_goal)
            self.live_renderer=LiveRenderer(self.xml)
            print(f'交互页面：http://127.0.0.1:{self.view.port}，请在地图上点选目标。',flush=True)

    def command(self):
        if self.manual_hold:return np.zeros(3,dtype=np.float32)
        return self.cmd.copy() if time.monotonic()-self.command_at<.3 else np.zeros(3,dtype=np.float32)

    def velocity(self,msg):
        incoming=np.array([msg.linear.x,msg.linear.y,msg.angular.z],dtype=np.float32)
        if np.linalg.norm(self.cmd)>.03 and np.linalg.norm(incoming)<.005:self.event('zero_command',{})
        self.cmd=incoming;self.command_at=time.monotonic()

    def event(self,name,data):
        if name in self.counts:self.counts[name]+=1
        self.events.append({'sim_time':self.data.time,'event':name,**data})

    def message(self,name,msg):
        try:data=json.loads(msg.data)
        except ValueError:data={'raw':msg.data}
        if name=='planner' and data.get('state')=='EXECUTE' and self.first_trajectory is None:self.first_trajectory=self.data.time
        if self.interactive and self.goal_active:
            if name=='planner':
                self.ui_status={'GENERATE':'正在规划','EXECUTE':'导航中','REPLAN':'正在重新规划','EMERGENCY':'已停车，等待重新规划'}.get(data.get('state'),self.ui_status)
            elif name=='collision' and data.get('state')=='STOP_REQUESTED':self.ui_status='检测到停止条件，正在停车'
        self.event(name,data)

    def reached(self,msg):
        if self.goal_stamp is not None and msg.stamp==self.goal_stamp:
            self.counts['goals_reached']+=1;self.received_goal=True
            if self.interactive:self.finish_goal('reached')
            else:self.result='reached'

    def failed(self,msg):
        if self.goal_stamp is not None and msg.stamp==self.goal_stamp:
            self.counts['goals_failed']+=1
            if self.interactive:self.finish_goal('failed')
            else:self.result='failed'

    def finish_goal(self,outcome):
        if not self.goal_active:return
        self.goal_active=False;self.goal_outcome=outcome;self.manual_hold=True
        labels={'reached':'已到达，可继续选点','failed':'导航失败，可重新选点或重置','recovery_limit':'位置恢复次数已用尽，可重新选点','fell':'机身失稳，请重置场景'}
        self.ui_status=labels[outcome]
        self.goal_history.append(f'{self.data.time:.1f} s · {self.ui_status}')
        self.event('interactive_goal_result',{'outcome':outcome})

    def visual_curve(self,msg):
        if len(msg.pos_pts)<4:return
        try:
            controls=np.array([[p.x,p.y,p.z] for p in msg.pos_pts])
            curve=BSpline(msg.knots,controls,msg.order)
            self.curve=curve(np.linspace(msg.knots[msg.order],msg.knots[-msg.order-1],100)).tolist()
        except ValueError:self.curve=[]

    def visual_reference(self,msg):
        self.reference=[[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in msg.poses]

    def cancel_navigation(self,status='已停止，可重新选点'):
        self.manual_hold=True;self.cmd[:]=0.;self.command_at=time.monotonic()
        self.goal_stamp=None;self.goal_active=False;self.sent=False;self.recovery_requested=False
        self.curve=[];self.reference=[];self.cancel_pub.publish(Empty());self.ui_status=status
        self.event('interactive_cancel',{})

    def reset_scene(self):
        self.cancel_navigation('正在重置，等待站稳')
        now=self.data.time
        mujoco.mj_resetData(self.model,self.data);self.data.time=now
        self.data.qpos[:3]=self.initial;self.data.qpos[3:7]=[1.,0.,0.,0.]
        if self.policy is not None:
            self.data.qpos[7:]=self.policy.default_dof;self.data.ctrl[:]=self.policy.default_dof
            self.policy.reset_policy_state();self.policy.stand=True
        if self.hover is not None:self.hover.reset()
        self.data.mocap_pos[self.mocap]=[0.,8.,.4]
        mujoco.mj_forward(self.model,self.data)
        self.last_sensor=self.last_R=self.last_body=self.last_pub_time=None
        self.contact_active=self.fallen=False;self.ready_at=now+2.;self.requested_target=None
        self.goal_published_at=None;self.zero_since=None;self.reset_count+=1
        self.goal_outcome=None
        self.trail_start=len(self.trajectory)
        self.contact_pub.publish(Bool(data=False));self.event('interactive_reset',{})

    def interactive_commands(self):
        import queue
        while True:
            try:command=self.view.commands.get_nowait()
            except queue.Empty:break
            action=command['action']
            if action=='goal':
                if self.fallen or self.data.time<self.ready_at:continue
                self.goal_yaw=command['yaw']
                c,s=math.cos(self.goal_yaw),math.sin(self.goal_yaw)
                offset=np.array([[c,-s],[s,c]])@MOUNT_T[:2]
                ground=self.floor(command['x'],command['y'])
                self.requested_target=[command['x'],command['y'],ground]
                self.goal=np.array([command['x']+offset[0],command['y']+offset[1],ground])
                self.manual_hold=True;self.cmd[:]=0.;self.recoveries=0;self.goal_outcome=None
                self.goal_published_at=None;self.ui_status='正在规划';self.path()
            elif action=='stop':self.cancel_navigation()
            elif action=='reset':self.reset_scene()
            elif action=='close':self.cancel_navigation('测试已结束');self.result='session_closed'

    def update_view(self):
        body=self.data.xpos[self.base].copy()
        yaw=Rotation.from_matrix(self.data.xmat[self.base].reshape(3,3)).as_euler('xyz')[2]
        if self.data.time>=self.ready_at and self.ui_status in ('等待站稳','正在重置，等待站稳'):self.ui_status='等待选择目标'
        self.frame_id+=1
        frame=self.live_renderer.frame(self.data,self.requested_target,self.curve)
        state={'ready':self.data.time>=self.ready_at,'fallen':self.fallen,'case':self.args.case,
               'motion':self.args.motion,'hover_height':self.args.hover_height if self.hover_mode else None,
               'status':self.ui_status,'time':self.data.time,'body':body.tolist(),'yaw':float(yaw),
               'cmd':self.command().tolist(),'target':self.requested_target,'target_yaw':self.goal_yaw,
               'distance':float(np.linalg.norm(body[:2]-np.array(self.requested_target[:2]))) if self.requested_target else None,
               'boxes':[{'name':n,'center':c.tolist(),'half':h.tolist()} for n,c,h in self.boxes],
               'dynamic':self.data.mocap_pos[self.mocap].tolist(),
               'trail':[[row[1],row[2]] for row in self.trajectory[max(self.trail_start,len(self.trajectory)-4000)::5]],
               'curve':self.curve,'reference':self.reference,'history':self.goal_history[-5:],
               'prior_free':self.prior.display_points,'prior_resolution':2.*self.prior.resolution,
               'frame_id':self.frame_id,'goal_active':self.goal_active,'goal_outcome':self.goal_outcome}
        self.view.update(state,frame)

    def position_lost(self,msg):
        if self.goal_stamp is not None and msg.stamp==self.goal_stamp:
            self.event('position_lost',{});self.recovery_requested=True

    def map_snapshot(self,msg):
        self.snapshots['occupancy_inflate']=np.asarray(list(point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True)))
        if self.snapshot_sub is not None:self.snapshot_sub.unregister();self.snapshot_sub=None

    def stop_service(self,_req):
        self.cmd[:]=0.;self.command_at=time.monotonic()
        if self.policy is not None:self.policy.stand=True
        self.event('stop_service',{});return TriggerResponse(success=True,message='MuJoCo command cleared; measured velocity is published separately')

    def floor(self,x,y):
        if self.args.case=='stairs' and abs(y)<.82:
            if 0.<=x<3.6:return .12*(int(x/.3)+1)
            if 3.6<=x<=4.7:return 1.5
        return 0.

    def hover_support(self,x,y):
        # Interpolate tread centres for a continuous support profile; collision
        # geometry and raw FREE keep their actual stepped heights.
        if self.args.case=='stairs' and abs(y)<.82:
            return float(np.interp(x,[-.15,*[.15+.3*i for i in range(12)],3.75,4.7],
                                  [0.,*[.12*(i+1) for i in range(12)],1.5,1.5])) if x<=4.7 else 0.
        return self.floor(x,y)

    def static_inputs(self):
        header=Header(stamp=rospy.Time.now(),frame_id='map')
        self.background_pub.publish(point_cloud2.create_cloud_xyz32(header,surface_points(self.boxes)))
        self.free_pub.publish(point_cloud2.create_cloud_xyz32(header,self.prior.points))

    def path(self):
        # Full original pose goal; SCAN's terminal adapter owns lidar lever-arm compensation.
        start=self.data.site('lidar').xpos.copy()
        start[2]-=self.navigation_height
        try:
            points=self.prior.route(start,self.goal,self.goal_yaw)
            points=smooth_free_path(points,self.prior.graph)
        except ValueError as exc:
            if not self.interactive:self.result='prior_route_failed'
            self.cancel_navigation(str(exc))
            self.manual_hold=True;self.goal_active=False;self.goal_outcome='prior_route_failed';self.ui_status=str(exc);self.goal_history.append(str(exc));self.goal_stamp=None;self.sent=True
            self.event('prior_route_failed',{'reason':str(exc)})
            return
        route=NavPath();route.header=Header(stamp=rospy.Time.now(),frame_id='map');self.goal_stamp=route.header.stamp
        for x,y,z in points:
            p=PoseStamped();p.header=route.header
            p.pose.position.x=x;p.pose.position.y=y;p.pose.position.z=z+self.navigation_height
            p.pose.orientation.w=1.;route.poses.append(p)
        route.poses[-1].pose.position.z=self.goal[2]+self.navigation_height
        route.poses[-1].pose.orientation.z=math.sin(self.goal_yaw/2);route.poses[-1].pose.orientation.w=math.cos(self.goal_yaw/2)
        self.path_pub.publish(route);self.sent=True;self.reference=points.tolist()
        self.goal_active=True;self.manual_hold=False
        if self.goal_published_at is None:self.goal_published_at=self.data.time
        self.event('goal_sent',{'goal_reference_xyz':self.goal.tolist(),'goal_yaw':self.goal_yaw,'requested_body_xyz':self.requested_target,'global_route':points.tolist()})

    def odometry(self):
        stamp=rospy.Time.now();t=self.data.time
        sensor=self.data.site('lidar').xpos.copy();R=self.data.site('lidar').xmat.reshape(3,3).copy()
        body=self.data.xpos[self.base].copy();bodyR=self.data.xmat[self.base].reshape(3,3).copy()
        dt=t-self.last_pub_time if self.last_pub_time is not None else .02
        vs=(sensor-self.last_sensor)/dt if self.last_sensor is not None else np.zeros(3)
        vb=(body-self.last_body)/dt if self.last_body is not None else np.zeros(3)
        omega=(Rotation.from_matrix(self.last_R).inv()*Rotation.from_matrix(R)).as_rotvec()/dt if self.last_R is not None else np.zeros(3)
        for pub,p,r,v,w in [(self.pose_pub,sensor,R,R.T@vs,omega),(self.sensor_pub,sensor,R,R.T@vs,omega),(self.chassis_pub,body,bodyR,bodyR.T@vb,omega)]:
            msg=Odometry();msg.header=Header(stamp=stamp,frame_id='map');msg.child_frame_id='base_link'
            msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z=map(float,p)
            q=Rotation.from_matrix(r).as_quat();msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w=map(float,q)
            msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z=map(float,v)
            msg.twist.twist.angular.x,msg.twist.twist.angular.y,msg.twist.twist.angular.z=map(float,w);pub.publish(msg)
        self.last_sensor,self.last_R,self.last_body,self.last_pub_time=sensor,R,body,t
        euler=Rotation.from_matrix(bodyR).as_euler('xyz');self.trajectory.append([t,*body,*euler,*self.command(),*sensor]);self.states.append(self.data.qpos.copy());self.mocap_states.append(self.data.mocap_pos.copy())
        contact=False
        if self.hover is not None and self.hover.blocked:
            self.counts['body_contacts']+=1;self.counts['obstacle_contacts']+=1;contact=True
        for i in range(self.data.ncon):
            c=self.data.contact[i]
            if c.dist>=0.:continue
            names=[mujoco.mj_id2name(self.model,mujoco.mjtObj.mjOBJ_GEOM,int(g)) or '' for g in (c.geom1,c.geom2)]
            world=[n for n in names if n.startswith(('wall','static_box','dynamic_box','floor','stair','landing'))]
            base_hit=any(n.startswith('base') for n in names)
            obstacle=any(n.startswith(('wall','static_box','dynamic_box')) for n in world)
            if base_hit and world:self.counts['body_contacts']+=1;contact=True
            if obstacle:self.counts['obstacle_contacts']+=1;contact=True
        if contact and not self.contact_active:self.event('contact_begin',{})
        self.contact_active=contact
        self.contact_pub.publish(Bool(data=contact))
        if body[2]<.12 or abs(euler[0])>.9 or abs(euler[1])>.9:
            if self.interactive and not self.fallen:
                self.finish_goal('fell');self.fallen=True;self.cancel_navigation('机身失稳，请重置场景')
            elif not self.interactive:self.result='fell'

    def lidar(self):
        if self.hover_mode:
            theta=self.theta+self.data.time*.37;phi=self.phi
            directions=np.column_stack((np.cos(phi)*np.cos(theta),np.cos(phi)*np.sin(theta),np.sin(phi)))
            origin=self.data.site('lidar').xpos.copy()
            rays=np.ascontiguousarray(directions@self.data.site('lidar').xmat.reshape(3,3).T)
            ids=np.empty(len(rays),np.int32);distances=np.empty(len(rays),np.float64)
            mujoco.mj_multiRay(self.model,self.data,origin,rays.reshape(-1),np.array([1,1,1,0,1,1],np.uint8),True,self.base,ids,distances,len(rays),8.)
            valid=(distances>=.5)&(distances<=8.)
            world=origin+rays[valid]*distances[valid,None]
            self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(Header(stamp=rospy.Time.now(),frame_id='map'),world))
            return
        theta,phi=self.policy.livox_generator.sample_ray_angles()
        indices=np.linspace(0,len(theta)-1,min(4096,len(theta)),dtype=int)
        self.policy.lidar.trace_rays(self.data,np.ascontiguousarray(theta[indices],dtype=np.float32),np.ascontiguousarray(phi[indices],dtype=np.float32))
        points=np.asarray(self.policy.lidar.get_hit_points());distance=np.asarray(self.policy.lidar.get_distances())
        points=points[(distance>=.5)&(distance<=8.)]
        world=points@self.policy.lidar.sensor_rotation.T+self.policy.lidar.sensor_position
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(Header(stamp=rospy.Time.now(),frame_id='map'),world))

    def run(self):
        start_wall=time.monotonic();last_pub=last_cloud=last_progress=last_view=-1.;self.static_inputs()
        while not rospy.is_shutdown() and self.data.time<self.args.duration and self.result is None:
            t=self.data.time
            if self.interactive:self.interactive_commands()
            if self.args.case=='dynamic_crossing' and self.sent:
                elapsed=t-self.goal_published_at
                self.data.mocap_pos[self.mocap]=[0.,max(-2.,1.8-.45*max(0.,elapsed-1.)),.4]
            if self.args.case=='blocking_then_clear' and self.sent:
                elapsed=t-self.goal_published_at
                self.data.mocap_pos[self.mocap]=[0.,0. if 1.<elapsed<9. else 8.,.4]
            cmd=self.command()
            if self.hover is not None:self.hover.step(cmd)
            else:
                if np.linalg.norm(cmd)<.005:
                    if self.zero_since is None:self.zero_since=t
                    if t-self.zero_since>.2:self.policy.stand=True
                else:self.zero_since=None;self.policy.stand=False
                self.policy.get_control(self.model,self.data);mujoco.mj_step(self.model,self.data)
            if t-last_pub>=.019:self.odometry();last_pub=t
            if t-last_cloud>=.095:self.lidar();last_cloud=t
            if self.args.case=='blocking_then_clear' and t>18. and not self.map_snapshot_requested:
                self.map_snapshot_requested=True;self.snapshot_sub=rospy.Subscriber('/grid_map/occupancy_inflate',PointCloud2,self.map_snapshot,queue_size=1)
            if t>2. and not self.sent and not self.interactive and self.path_pub.get_num_connections()>=1:self.path()
            if self.recovery_requested:
                self.recovery_requested=False
                if self.recoveries>=2:
                    if self.interactive:self.finish_goal('recovery_limit')
                    else:self.result='recovery_limit'
                else:self.recoveries+=1;self.path()
            if self.interactive and time.monotonic()-last_view>=.10:
                self.update_view();last_view=time.monotonic()
            if t-last_progress>=2.:
                print(json.dumps({'case':self.args.case,'time':round(t,2),'body':self.data.xpos[self.base].round(3).tolist(),'cmd':cmd.round(3).tolist(),'result':self.result}),flush=True);last_progress=t
            delay=start_wall+self.data.time-time.monotonic()
            if delay>0.:time.sleep(delay)
        self.cmd[:]=0.;self.command_at=time.monotonic()
        for _ in range(3):self.contact_pub.publish(Bool(data=False))
        report={'case':self.args.case,'result':self.result or ('session_closed' if self.interactive else 'timeout'),
                'physics':'kinematic hover body with swept MuJoCo collision checks' if self.hover_mode else 'MuJoCo dynamic free-root + ONNX actuator policy; no kinematic root integration',
                'motion':self.args.motion,'hover_height':self.args.hover_height if self.hover_mode else None,
                'duration_sim':float(self.data.time),'duration_wall':time.monotonic()-start_wall,'counts':self.counts,
                'first_trajectory_sim_time':self.first_trajectory,'position_recoveries':self.recoveries,'policy_sha256':self.policy_sha,
                'goal_reference_xyz':self.goal.tolist(),'goal_yaw':self.goal_yaw,'final_body_xyz':self.data.xpos[self.base].tolist(),
                'interactive':self.interactive,'goal_history':self.goal_history,'reset_count':self.reset_count,
                'source_reference':'saved lidar mount','navigation_height':float(self.navigation_height),
                'lidar_sampling':'changing 2048-ray 360 degree pattern' if self.hover_mode else 'changing Livox Mid360 pattern, at most 4096 rays per 0.1 s','ros_master':os.environ.get('ROS_MASTER_URI','')}
        self.output.joinpath('result.json').write_text(json.dumps(report,indent=2));self.output.joinpath('events.json').write_text(json.dumps(self.events,indent=2));np.save(self.output/'trajectory.npy',self.trajectory);np.save(self.output/'qpos.npy',self.states);np.save(self.output/'mocap.npy',self.mocap_states);np.savez_compressed(self.output/'map_snapshots.npz',**self.snapshots)
        print(json.dumps(report),flush=True)
        if self.live_renderer is not None:self.live_renderer.close()
        if self.view is not None:self.view.close()


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--case',required=True);p.add_argument('--assets',type=Path,default=Path('/home/xzq/代码合集/MuJoCo-LiDAR'));p.add_argument('--output',type=Path,required=True);p.add_argument('--duration',type=float,default=45.)
    p.add_argument('--interactive',action='store_true');p.add_argument('--ui-port',type=int,default=11882)
    p.add_argument('--motion',choices=['hover','dynamic'],default='hover');p.add_argument('--hover-height',type=float,default=.4)
    args=p.parse_args();rospy.init_node('go2_physics_simulation');Simulation(args).run();rospy.signal_shutdown('simulation complete')
