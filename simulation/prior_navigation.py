"""Scene-matched fixed FREE support and the repository's global graph search."""
import math
from pathlib import Path
import sys
import numpy as np
from scipy.ndimage import binary_erosion

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'nav/src/map_for_plan/scripts'))
from plan_graph import FreeGraph

REFERENCE_OFFSET=np.array([-.221454,-.018929])


def rotate(xy,yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return np.array([c*xy[0]-s*xy[1],s*xy[0]+c*xy[1]])


class PriorNavigation:
    def __init__(self,case,boxes,floor,resolution=.05):
        self.resolution=resolution;self.floor=floor;self.boxes=boxes;self.case=case
        self.ix0,self.iy0=-116,-56
        self.nx,self.ny=232,112
        xs=(np.arange(self.ix0,self.ix0+self.nx)+.5)*resolution
        ys=(np.arange(self.iy0,self.iy0+self.ny)+.5)*resolution
        self.x,self.y=np.meshgrid(xs,ys,indexing='ij')
        self.z=np.array([[floor(x,y) for y in ys] for x in xs])
        self.mask=(abs(self.x)<5.5)&(abs(self.y)<2.55)
        if case=='large_obstacle':
            line=np.array([[-4.,0.],[-1.25,0.],[-1.25,-1.2],[1.25,-1.2],[1.25,0.],[4.,0.]])
            distance=np.full_like(self.x,np.inf)
            q=np.stack((self.x,self.y),axis=-1)
            for a,b in zip(line[:-1],line[1:]):
                d=b-a;u=np.clip(np.sum((q-a)*d,axis=-1)/float(d@d),0.,1.)
                distance=np.minimum(distance,np.linalg.norm(q-(a+u[...,None]*d),axis=-1))
            self.mask &= distance<=.75
        elif case=='stairs':
            self.mask &= (self.x>=-1.8)&(self.x<=4.65)&(abs(self.y)<=.69)
        elif case=='blocking_then_clear':
            self.mask &= abs(self.y)<.4
        for name,center,half in boxes:
            if name.startswith(('stair_','landing')):continue
            # Retain a voxel only when its whole XY surface is outside the box.
            inside=(abs(self.x-center[0])<half[0]+.5*resolution)&(abs(self.y-center[1])<half[1]+.5*resolution)
            self.mask &= ~inside
        self.points=np.column_stack((self.x[self.mask],self.y[self.mask],self.z[self.mask]))
        painted=self.mask[::2,::2]&self.mask[1::2,::2]&self.mask[::2,1::2]&self.mask[1::2,1::2]
        self.display_points=np.column_stack(((self.x[::2,::2]+.5*resolution)[painted],(self.y[::2,::2]+.5*resolution)[painted])).tolist()
        # Global graph spacing is .1 m, while the support remains .05 m.
        coarse=self.mask.copy();coarse[1::2,:]=False;coarse[:,1::2]=False
        points=np.column_stack((self.x[coarse],self.y[coarse],self.z[coarse]))
        self.graph=FreeGraph(points,2.*resolution,max_step=.25,center_weight=2.)
        self.heading_masks={}
        self._filter_edges()

    def _indices(self,xy):
        key=np.floor(np.asarray(xy)/self.resolution).astype(int)
        return key[...,0]-self.ix0,key[...,1]-self.iy0

    def contains(self,xy):
        i,j=self._indices(xy)
        return bool(0<=i<self.nx and 0<=j<self.ny and self.mask[i,j])

    def _offsets(self,yaw):
        r=self.resolution;c,s=math.cos(yaw),math.sin(yaw)
        offset=rotate(REFERENCE_OFFSET,yaw)
        hx,hy=.38,.205
        ex,ey=abs(c)*hx+abs(s)*hy,abs(s)*hx+abs(c)*hy
        dx=np.arange(math.floor((offset[0]-ex)/r)-1,math.ceil((offset[0]+ex)/r)+2)
        dy=np.arange(math.floor((offset[1]-ey)/r)-1,math.ceil((offset[1]+ey)/r)+2)
        xx,yy=np.meshgrid(dx,dy,indexing='ij');x=xx*r-offset[0];y=yy*r-offset[1];h=.5*r
        hit=(abs(x)<=ex+h)&(abs(y)<=ey+h)&(abs(c*x+s*y)<=hx+h*(abs(c)+abs(s)))&(abs(-s*x+c*y)<=hy+h*(abs(c)+abs(s)))
        return np.column_stack((xx[hit],yy[hit]))

    def _static_clear(self,x,y,z,yaw):
        center=rotate(REFERENCE_OFFSET,yaw)
        clear=np.ones(np.broadcast(x,y,z).shape,dtype=bool)
        for sign in (-1.,1.):
            cx=x+center[0]+sign*.18*math.cos(yaw)
            cy=y+center[1]+sign*.18*math.sin(yaw)
            for name,box,half in self.boxes:
                if name.startswith(('stair_','landing')):continue
                overlap=(z+.5>=box[2]-half[2])&(z+.3<=box[2]+half[2])
                dx=np.maximum(abs(cx-box[0])-half[0],0.)
                dy=np.maximum(abs(cy-box[1])-half[1],0.)
                clear &= ~overlap | (dx*dx+dy*dy>.25**2)
        return clear

    def _heading_mask(self,yaw):
        key=round(yaw,8)
        if key not in self.heading_masks:
            offsets=self._offsets(yaw)
            extent=int(np.max(abs(offsets)))
            structure=np.zeros((2*extent+1,2*extent+1),bool)
            structure[offsets[:,0]+extent,offsets[:,1]+extent]=True
            mask=binary_erosion(self.mask,structure=structure,border_value=0)
            mask &= self._static_clear(self.x,self.y,self.z,yaw)
            self.heading_masks[key]=mask
        return self.heading_masks[key]

    def _filter_edges(self):
        for i,neighbors in enumerate(self.graph.adj):
            a=self.graph.points[i];kept=[]
            for j,cost in neighbors:
                b=self.graph.points[j];d=b-a
                yaw=math.atan2(d[1],d[0])
                mask=self._heading_mask(yaw)
                samples=np.vstack((a,(a+b)*.5,b))
                ix,iy=self._indices(samples[:,:2])
                if np.all(mask[ix,iy]):kept.append((j,cost))
            self.graph.adj[i]=kept

    def pose_supported(self,reference,yaw):
        # Exact cell SAT at the requested position, matching PlanFreeSpace.
        r=self.resolution;c,s=math.cos(yaw),math.sin(yaw)
        center=np.asarray(reference[:2])+rotate(REFERENCE_OFFSET,yaw)
        hx,hy=.38,.205;ex,ey=abs(c)*hx+abs(s)*hy,abs(s)*hx+abs(c)*hy;h=.5*r
        ix=np.arange(math.floor((center[0]-ex)/r),math.floor((center[0]+ex)/r)+1)
        iy=np.arange(math.floor((center[1]-ey)/r),math.floor((center[1]+ey)/r)+1)
        xx,yy=np.meshgrid(ix,iy,indexing='ij');dx=(xx+.5)*r-center[0];dy=(yy+.5)*r-center[1]
        hit=(abs(dx)<=ex+h)&(abs(dy)<=ey+h)&(abs(c*dx+s*dy)<=hx+h*(abs(c)+abs(s)))&(abs(-s*dx+c*dy)<=hy+h*(abs(c)+abs(s)))
        x,y=xx[hit]-self.ix0,yy[hit]-self.iy0
        if np.any(x<0)|np.any(x>=self.nx)|np.any(y<0)|np.any(y>=self.ny):return False
        if not np.all(self.mask[x,y]):return False
        if not np.all(abs(self.z[x,y]-reference[2])<=.3+.5*r):return False
        return bool(self._static_clear(np.asarray(reference[0]),np.asarray(reference[1]),np.asarray(reference[2]),yaw))

    def validate_body_goal(self,command):
        body=np.array([command['x'],command['y'],self.floor(command['x'],command['y'])])
        reference=body.copy();reference[:2]-=rotate(REFERENCE_OFFSET,command['yaw'])
        if not self.contains(body[:2]):raise ValueError('目标不在绿色先验可通行区域内')
        if not self.pose_supported(reference,command['yaw']):raise ValueError('目标过近障碍或先验边界，当前机身模型无法安全停靠，请选择更有余量的位置')
        return reference

    def _body_ground(self,reference_xy,yaw):
        # The reference XY is the lidar pose. Its ground-relative height must
        # follow the body support, not the tread under the forward-mounted lidar.
        body_xy=np.asarray(reference_xy)+rotate(REFERENCE_OFFSET,yaw)
        return self.floor(*body_xy)

    def route(self,start,goal,goal_yaw):
        if not self.pose_supported(goal,goal_yaw):raise ValueError('目标姿态不满足先验或机身净空，停止规划')
        search_goal=np.asarray(goal).copy();suffix=[]
        approach=search_goal.copy();approach[:2]-=.5*np.array([math.cos(goal_yaw),math.sin(goal_yaw)])
        approach[2]=self._body_ground(approach[:2],goal_yaw)
        # Arrive in the requested heading when a straight final approach really
        # fits. Near a box this may be impossible; then use the connected side
        # approach and SCAN's existing final-pose reference compensation.
        if (np.linalg.norm(np.asarray(start)-goal)>.8 and
                all(self.pose_supported(approach+(goal-approach)*u,goal_yaw) for u in np.linspace(0.,1.,21))):
            search_goal=approach;suffix=[goal]
        candidates=np.argsort(np.linalg.norm(self.graph.points-np.asarray(start),axis=1))[:16]
        targets=np.argsort(np.linalg.norm(self.graph.points-search_goal,axis=1))[:16]
        for a in candidates:
            if np.linalg.norm(self.graph.points[a]-start)>.35:continue
            for b in targets:
                if np.linalg.norm(self.graph.points[b]-search_goal)>.2:continue
                raw=self.graph.astar(int(a),int(b))
                if raw is None:continue
                # Do not let a millimetre-sized grid-to-goal connector reverse
                # the final approach heading used by lidar/body compensation.
                while len(raw)>1 and np.linalg.norm(raw[-1]-search_goal)<.075:raw=raw[:-1]
                while len(raw)>1 and np.linalg.norm(raw[0]-start)<.075:raw=raw[1:]
                path=np.vstack((start,raw,search_goal,*suffix))
                for k in range(1,len(path)-1):
                    delta=path[k+1,:2]-path[k,:2]
                    yaw=math.atan2(delta[1],delta[0]) if np.linalg.norm(delta)>1e-6 else goal_yaw
                    path[k,2]=self._body_ground(path[k,:2],yaw)
                valid=True
                for p,q in zip(path[:-1],path[1:]):
                    d=q-p;yaw=math.atan2(d[1],d[0]) if np.linalg.norm(d[:2])>1e-6 else goal_yaw
                    count=max(1,int(np.ceil(np.linalg.norm(d)/(.5*self.resolution))))
                    if not all(self.pose_supported(p+(q-p)*t,yaw) for t in np.linspace(0.,1.,count+1)):
                        valid=False;break
                if valid:return path
        raise ValueError('先验可通行区域内没有满足机身约束的连通路线')
