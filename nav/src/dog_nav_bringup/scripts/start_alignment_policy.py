"""Versioned, permit-gated zero-translation start alignment; no ROS/SDK I/O."""
import math

class StartAlignmentPolicy:
    def __init__(self,max_yaw=.5,min_yaw=.15,accel=1.6,tolerance=.20,stable=.15,ttl=.20):
        self.max_yaw,self.min_yaw,self.accel=max_yaw,min_yaw,accel
        self.tolerance,self.stable,self.ttl=tolerance,stable,ttl;self.cancel()
    @staticmethod
    def wrap(x):return math.atan2(math.sin(x),math.cos(x))
    def cancel(self):
        self.intent=None;self.allowed=False;self.rate=0.;self.holding=False;self.stable_since=None
    def receive(self,event,goal_ns,frame,pose,scene_now,now,may_start=True):
        try:
            if event['goal_stamp_ns']!=goal_ns or event['frame']!=frame:return False
            if not isinstance(event['request_id'],str) or not event['request_id']:return False
            if not math.isfinite(event['stamp']) or not -.02<=scene_now-event['stamp']<=self.ttl:return False
            if event.get('cancel'):
                if self.intent and event['request_id']==self.intent['request_id']:self.cancel()
                return False
            values=[event[k] for k in ['start_yaw','target_yaw','signed_angle','current_yaw','remaining_angle','x','y','z']]
            if not all(math.isfinite(x) for x in values) or not isinstance(event['allowed'],bool):return False
            if not .01<abs(event['signed_angle'])<=2*math.pi+.001:return False
            if abs(self.wrap(event['start_yaw']+event['signed_angle']-event['target_yaw']))>1e-5:return False
            if abs(event['remaining_angle'])>2*math.pi+.001 or abs(self.wrap(event['current_yaw']+event['remaining_angle']-event['target_yaw']))>1e-5:return False
            new=self.intent is None or event['request_id']!=self.intent['request_id']
            if new:
                if not may_start or not event['allowed'] or pose is None:return False
                if math.dist(pose[:3],[event[k] for k in ['x','y','z']])>.08:return False
                if abs(self.wrap(pose[3]-event['current_yaw']))>.2:return False
                self.intent=dict(event);self.remaining=event['remaining_angle']-self.wrap(pose[3]-event['current_yaw'])
                self.last_yaw=pose[3];self.anchor=pose[:3];self.begun=False;self.active_seconds=0.
                self.last_update=now;self.rate=0.;self.holding=False;self.stable_since=None
            elif any(abs(self.wrap(event[k]-self.intent[k]))>1e-6 for k in ['target_yaw','start_yaw']) or abs(event['signed_angle']-self.intent['signed_angle'])>1e-6:
                return False
            if math.dist(pose[:3],[event[k] for k in ['x','y','z']])>.08:return False
            self.allowed=event['allowed'];self.received=now;self.scene_stamp=event['stamp'];self.certified_pose=[event[k] for k in ['x','y','z']]
            return True
        except (KeyError,TypeError,ValueError):return False
    def step(self,pose,stationary,scene_now,now):
        if self.intent is None:return 0.,'inactive'
        dt=max(0.,min(.05,now-self.last_update));self.last_update=now
        if math.dist(pose[:3],self.anchor)>.12:self.rate=0.;return 0.,'position_displaced'
        delta=self.wrap(pose[3]-self.last_yaw)
        if abs(delta)>.5:self.rate=0.;return 0.,'pose_reset'
        self.remaining-=delta;self.last_yaw=pose[3]
        if not self.allowed or not -.02<=scene_now-self.scene_stamp<=self.ttl or not 0<=now-self.received<=self.ttl:
            self.rate=0.;self.stable_since=None;return 0.,'waiting_permission'
        if math.dist(pose[:3],self.certified_pose)>.08:self.rate=0.;return 0.,'waiting_permission'
        if not self.begun:
            if not stationary:self.rate=0.;return 0.,'waiting_stop'
            self.begun=True
        self.active_seconds+=dt
        if self.active_seconds>30.:self.rate=0.;return 0.,'timeout'
        error=self.remaining
        if self.holding and abs(error)>self.tolerance+.1:self.holding=False;self.stable_since=None
        if abs(error)<=self.tolerance:self.holding=True
        if self.holding:
            self.rate=0.
            if not stationary:self.stable_since=None;return 0.,'settling'
            if self.stable_since is None:self.stable_since=now
            return 0.,'done' if now-self.stable_since>=self.stable else 'settling'
        target=math.copysign(min(self.max_yaw,max(self.min_yaw,1.5*abs(error))),error)
        self.rate+=max(-self.accel*dt,min(self.accel*dt,target-self.rate))
        return self.rate,'turning'
