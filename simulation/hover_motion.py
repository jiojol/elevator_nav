"""Kinematic body over terrain, with swept MuJoCo geometry contact checks."""
import math
import mujoco
import numpy as np

BODY_HALF=np.array([.35,.175,.10])


class HoverMotion:
    def __init__(self,model,data,support,height):
        self.model,self.data,self.support,self.height=model,data,support,height
        self.base=model.body('base').id
        self.blocked=False
        self.contact_until=-1.

    def reset(self):
        self.blocked=False;self.contact_until=-1.

    def _contact(self):
        for i in range(self.data.ncon):
            contact=self.data.contact[i]
            if contact.dist>=0.:continue
            if self.base in self.model.geom_bodyid[[contact.geom1,contact.geom2]]:return True
        return False

    def step(self,command):
        dt=float(self.model.opt.timestep);q=self.data.qpos
        before=q[:7].copy();yaw=2.*math.atan2(q[6],q[3])
        vx,vy,wz=map(float,command)
        count=max(1,math.ceil(math.hypot(vx,vy)*dt/.01),math.ceil(abs(wz)*dt/.02))
        hit=False
        for _ in range(count):
            previous=q[:7].copy();middle=yaw+.5*wz*dt/count
            c,s=math.cos(middle),math.sin(middle)
            q[:2]+=np.array([c*vx-s*vy,s*vx+c*vy])*dt/count
            yaw+=wz*dt/count
            q[2]=self.support(*q[:2])+self.height
            q[3:7]=[math.cos(yaw/2),0.,0.,math.sin(yaw/2)]
            mujoco.mj_forward(self.model,self.data)
            if self._contact():
                q[:7]=previous;hit=True;break
        self.data.time+=dt
        if hit:self.contact_until=self.data.time+.2
        self.blocked=hit or self.data.time<self.contact_until
        self.data.qvel[:]=0.
        self.data.qvel[:3]=(q[:3]-before[:3])/dt
        actual_yaw=2.*math.atan2(q[6],q[3])
        self.data.qvel[5]=math.atan2(math.sin(actual_yaw-2.*math.atan2(before[6],before[3])),math.cos(actual_yaw-2.*math.atan2(before[6],before[3])))/dt
        mujoco.mj_forward(self.model,self.data)
