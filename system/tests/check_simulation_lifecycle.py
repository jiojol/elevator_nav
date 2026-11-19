#!/usr/bin/env python3
"""Local integration: runner kill, proven orphan recovery, then normal close."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import urllib.request
import xmlrpc.client

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'simulation'))
from process_lifecycle import identity
BASE=ROOT/'simulation/artifacts/lifecycle_check_20261028'
BASE.mkdir(exist_ok=True)
MASTER='http://127.0.0.1:11791';UI='http://127.0.0.1:11882'
opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))


def rpc():return xmlrpc.client.ServerProxy(MASTER)


def ready(process):
    for _ in range(200):
        if process.poll() is not None:raise RuntimeError('runner exited before UI was ready')
        try:
            with opener.open(UI+'/api/state',timeout=.5) as response:state=json.load(response)
            if state.get('ready'):return state
        except OSError:pass
        time.sleep(.1)
    raise RuntimeError('UI readiness timeout')


def wait_clean(pids):
    deadline=time.monotonic()+15
    while time.monotonic()<deadline:
        try:rpc().getPid('/lifecycle_check');master_alive=True
        except OSError:master_alive=False
        survivors=[pid for pid in pids if identity(pid) is not None]
        if not master_alive and not survivors:return
        time.sleep(.1)
    raise AssertionError(('surviving_nodes',survivors,'master_alive',master_alive))


def node_pids():
    master=rpc();state=master.getSystemState('/lifecycle_check')[2]
    names={n for group in state for _,nodes in group for n in nodes}
    return [xmlrpc.client.ServerProxy(master.lookupNode('/lifecycle_check',name)[2]).getPid('/lifecycle_check')[2] for name in names]


def launch_runner(folder):
    folder.mkdir(exist_ok=True)
    handle=(folder/'console.log').open('w')
    process=subprocess.Popen([sys.executable,str(ROOT/'simulation/run_interactive.py'),'--case','stairs',
        '--duration','120','--no-browser','--output',str(folder)],stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
    return process,handle


def main():
    try:rpc().getPid('/lifecycle_check')
    except OSError:pass
    else:raise SystemExit('11791 already active; preserved it, no tests started')
    forced=BASE/'forced';process,handle=launch_runner(forced)
    try:
        state=ready(process);assert state['case']=='stairs' and not state['goal_active']
        pids=node_pids();process.kill();process.wait(timeout=5);wait_clean(pids)
        assert (forced/'result.json').exists(),'physics must save a result even after master exit'
        print('PASS forced exit: all nodes/master stopped, physics result saved',flush=True)
    finally:
        if process.poll() is None:process.send_signal(signal.SIGINT);process.wait(timeout=30)
        handle.close()
    # Construct the exact legacy condition: finished output, no runner/lock,
    # but its known local launchers and nodes remain alive. No physical robot.
    fixture=BASE/'ended_orphan';fixture.mkdir(exist_ok=True)
    (fixture/'result.json').write_text(json.dumps({'result':'session_closed','physics':'MuJoCo lifecycle test fixture'}))
    env=os.environ.copy();env.pop('ROS_HOSTNAME',None)
    prefix=ROOT/'simulation/artifacts/local_build/devel'
    env.update(ROS_MASTER_URI=MASTER,ROS_IP='127.0.0.1',ROS_LOG_DIR=str(fixture/'ros_logs'),
        CMAKE_PREFIX_PATH=str(prefix)+':/opt/ros/noetic',ROS_PACKAGE_PATH=str(prefix/'share')+':/opt/ros/noetic/share',
        LD_LIBRARY_PATH=str(prefix/'lib')+':'+str(ROOT/'simulation/artifacts/local_dependencies/tf/lib')+':/opt/ros/noetic/lib',
        PYTHONPATH=str(prefix/'lib/python3/dist-packages')+':/opt/ros/noetic/lib/python3/dist-packages')
    handles=[];old=[];process=None
    def legacy(command,name):
        handle=(fixture/name).open('w');handles.append(handle)
        child=subprocess.Popen(command,env=env,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True);old.append(child);return child
    try:
        legacy(['/opt/ros/noetic/bin/roscore','-p','11791'],'master.log')
        for _ in range(100):
            try:
                if rpc().getPid('/lifecycle_check')[0]==1:break
            except OSError:pass
            time.sleep(.1)
        legacy(['/opt/ros/noetic/bin/roslaunch',str(ROOT/'simulation/isolated.launch')],'planner.log')
        for _ in range(100):
            names={n for g in rpc().getSystemState('/lifecycle_check')[2] for _,ns in g for n in ns}
            if {'/scan_planner_node','/closed_loop_controller','/scan_command_gate'}<=names:break
            time.sleep(.1)
        legacy_ids=node_pids()
        recovered=BASE/'recovered';process,handle=launch_runner(recovered);handles.append(handle)
        state=ready(process);assert state['case']=='stairs'
        assert not any(identity(pid) is not None for pid in legacy_ids),'old nodes remained alive'
        log=(recovered/'console.log').read_text();assert '清理已结束仿真' in log
        print('PASS legacy recovery: ended own session cleaned; stairs UI ready',flush=True)
        pids=node_pids()
        request=urllib.request.Request(UI+'/api/command',data=b'{"action":"close"}',headers={'Content-Type':'application/json'})
        with opener.open(request,timeout=2) as response:assert response.status==200
        assert process.wait(timeout=30)==0;wait_clean(pids)
        print('PASS normal close: result saved and all nodes/master stopped',flush=True)
        (BASE/'verification.json').write_text(json.dumps({'forced_exit_clean':True,'forced_exit_result_saved':True,
            'legacy_recovery':True,'stairs_ui_ready':True,'normal_close_clean':True,'remote_connections':[]},indent=2))
    finally:
        if process is not None and process.poll() is None:process.send_signal(signal.SIGINT);process.wait(timeout=30)
        for child in reversed(old):
            if child.poll() is None:os.killpg(child.pid,signal.SIGINT);child.wait(timeout=10)
        for handle in handles:handle.close()


if __name__=='__main__':main()
