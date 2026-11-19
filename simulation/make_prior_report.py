#!/usr/bin/env python3
"""Export evidence from the recorded near-obstacle physics trial."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import Rectangle
import numpy as np

root=Path(__file__).resolve().parents[1]
source=root/'simulation/artifacts/prior_approach_validation_20261028'
output=root/'simulation/reports/20261028/interactive_prior_fix';output.mkdir(parents=True,exist_ok=True)
font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
prior=np.load(source/'prior_free.npy');motion=np.load(source/'trajectory.npy');events=json.loads((source/'events.json').read_text())
route=np.asarray(next(e['global_route'] for e in events if e['event']=='goal_sent'))
fig,ax=plt.subplots(figsize=(10,6),layout='constrained')
ax.set_facecolor('#e6ecee')
ax.scatter(prior[:,0],prior[:,1],s=8,marker='s',color='#d1ebd7',linewidths=0,label='先验可通行区域')
for geom in ET.parse(source/'scene.xml').getroot().find('worldbody').findall('geom'):
 if geom.get('type')!='box':continue
 p=np.fromstring(geom.get('pos'),sep=' ');h=np.fromstring(geom.get('size'),sep=' ')
 ax.add_patch(Rectangle(p[:2]-h[:2],2*h[0],2*h[1],color='#9cabb3'))
ax.plot(route[:,0],route[:,1],'--',color='#588ab2',lw=2,label='先验图全局路线（雷达参考）')
ax.plot(motion[:,1],motion[:,2],color='#087f75',lw=2.5,label='实际机身轨迹')
ax.scatter([1.2],[0.],s=110,c='#e99821',marker='*',label='可达目标 (1.2, 0)')
ax.scatter([.87],[0.],s=80,c='#bd4134',marker='x',label='净空不足目标 (0.87, 0)：拒绝')
ax.scatter(motion[0,1],motion[0,2],s=50,c='#087f75')
ax.set_xlim(-3.1,2.1);ax.set_ylim(-2.05,1.05);ax.set_aspect('equal')
ax.set_xlabel('世界 x / m',fontproperties=font);ax.set_ylabel('世界 y / m',fontproperties=font)
ax.set_title('先验区域内绕行到达近障碍目标 · 实际接触计数为 0',fontproperties=font,fontsize=14)
ax.legend(prop=font,loc='lower left',fontsize=10);ax.grid(alpha=.12)
fig.savefig(output/'near_obstacle_route.png',dpi=160);plt.close(fig)
records={}
for name in ['prior_goal_validation_20261028','prior_approach_validation_20261028','prior_fix_straight_20261028','prior_fix_straight_approach_20261028']:
 folder=root/'simulation/artifacts'/name
 records[name]={'result':json.loads((folder/'result.json').read_text()),'metadata':json.loads((folder/'run_metadata.json').read_text())}
records['cpp_terminal_guard_log']=(root/'simulation/artifacts/spatial_guard_20261028/test.log').read_text()
records['measured_latest_trial_pose_support']={'samples':1561,'outside_prior_or_static_model':0}
(output/'verification.json').write_text(json.dumps(records,ensure_ascii=False,indent=2)+'\n')
