#!/usr/bin/env python3
"""Exercise the real C++ publication guard on a separate loopback master."""
import os
from pathlib import Path
import signal
import subprocess
import time
import xmlrpc.client
import yaml

root=Path(__file__).resolve().parents[2]
prefix=root/'simulation/artifacts/local_build/devel'
output=root/'simulation/artifacts/spatial_guard_20261028';output.mkdir(parents=True,exist_ok=True)
env=os.environ.copy();env.pop('ROS_HOSTNAME',None)
env.update(ROS_MASTER_URI='http://127.0.0.1:11792',ROS_IP='127.0.0.1',
           ROS_LOG_DIR=str(output/'ros_logs'),CMAKE_PREFIX_PATH=str(prefix)+':/opt/ros/noetic',
           ROS_PACKAGE_PATH=str(prefix/'share')+':/opt/ros/noetic/share',
           LD_LIBRARY_PATH=str(prefix/'lib')+':'+str(root/'simulation/artifacts/local_dependencies/tf/lib')+':/opt/ros/noetic/lib',
           PYTHONPATH=str(prefix/'lib/python3/dist-packages')+':/opt/ros/noetic/lib/python3/dist-packages')
master=xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
try:
 master.getPid('/guard_test')
 raise SystemExit('11792 already occupied; left it unchanged')
except OSError:pass
with (output/'master.log').open('w') as log:
 process=subprocess.Popen(['/opt/ros/noetic/bin/roscore','-p','11792'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 try:
  for _ in range(100):
   try:
    if master.getPid('/guard_test')[0]==1:break
   except OSError:pass
   time.sleep(.1)
  params=yaml.safe_load(subprocess.check_output(['/opt/ros/noetic/bin/roslaunch','--dump-params',str(root/'simulation/isolated.launch')],env=env,text=True))
  for key,value in params.items():master.setParam('/guard_test',key,value)
  results=[]
  for name in ['scan_spatial_guard_test','body_reference_collision_test','map_publication_test']:
   result=subprocess.run([str(prefix/'lib/scan_planner'/name)],env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=30)
   results.append(result.stdout);print(result.stdout)
   if result.returncode:raise SystemExit(result.returncode)
  (output/'test.log').write_text('\n'.join(results))
 finally:
  os.killpg(process.pid,signal.SIGINT);process.wait(timeout=10)
