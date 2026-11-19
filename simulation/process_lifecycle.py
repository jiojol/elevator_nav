"""Bound simulation children to their runner; recover only proven ended runs."""
import ctypes
from pathlib import Path
import json
import os
import signal
import time
import urllib.parse
import xmlrpc.client

_LIBC=ctypes.CDLL(None,use_errno=True)


def parent_exit_hook():
    parent=os.getpid()
    def configure():
        # Linux PR_SET_PDEATHSIG: even SIGKILL of the runner stops its launchers.
        if _LIBC.prctl(1,signal.SIGINT,0,0,0)!=0:
            raise OSError(ctypes.get_errno(),'could not bind simulation child lifetime')
        if os.getppid()!=parent:
            os.kill(os.getpid(),signal.SIGINT)
    return configure


def install_exit_handlers():
    def terminate(signum,_frame):
        raise SystemExit(128+signum)
    for signum in (signal.SIGTERM,signal.SIGHUP):
        signal.signal(signum,terminate)


def identity(pid):
    try:
        fields=(Path('/proc')/str(pid)/'stat').read_text().rsplit(')',1)[1].split()
        return None if fields[0]=='Z' else fields[19]
    except (FileNotFoundError,ProcessLookupError):
        return None


def finished_processes(root,master_url):
    """A name alone is never ownership: require argv, master and ended output."""
    root=root.resolve();artifacts=root/'simulation/artifacts'
    expected_binaries={root/'simulation/artifacts/local_build/devel/lib/scan_planner/scan_planner_node',
                       root/'simulation/artifacts/local_build/devel/lib/scan_planner/closed_loop_controller',
                       root/'simulation/artifacts/local_build/devel/lib/dog_nav_bringup/reference_path_transform',
                       Path('/opt/ros/noetic/lib/tf2_ros/static_transform_publisher'),Path('/opt/ros/noetic/lib/rosout/rosout')}
    scripts={str(root/'nav/src/dog_nav_bringup/scripts/scan_command_gate.py'),
             str(root/'simulation/physics_bridge.py')}
    found=[]
    for process in Path('/proc').glob('[0-9]*'):
        pid=int(process.name);stamp=identity(pid)
        if stamp is None:continue
        try:
            argv=[os.fsdecode(a) for a in (process/'cmdline').read_bytes().split(b'\0') if a]
            env=dict(item.split('=',1) for item in (process/'environ').read_bytes().decode().split('\0') if '=' in item)
            if env.get('ROS_MASTER_URI','').rstrip('/')!=master_url.rstrip('/'):continue
            folder=Path(env.get('ROS_LOG_DIR','')).resolve().parent
            folder.relative_to(artifacts)
            result=json.loads((folder/'result.json').read_text())
            if result.get('result') not in {'session_closed','reached','failed','fell','timeout','recovery_limit','prior_route_failed'}:continue
            if not (str(result.get('physics','')).startswith('MuJoCo') or result.get('physics')=='kinematic hover body with swept MuJoCo collision checks'):continue
            exe=Path(os.readlink(process/'exe'))
            role='node'
            if '/opt/ros/noetic/bin/roslaunch' in argv and str(root/'simulation/isolated.launch') in argv:role='planner'
            elif '/opt/ros/noetic/bin/roscore' in argv and '-p' in argv and str(urllib.parse.urlparse(master_url).port) in argv:role='master'
            elif exe not in expected_binaries and not scripts.intersection(argv):continue
            if identity(pid)!=stamp:continue
            found.append({'pid':pid,'stamp':stamp,'group':os.getpgid(pid),'folder':str(folder),'role':role})
        except (OSError,ValueError,KeyError):continue
    return found


class _TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self,host):
        connection=super().make_connection(host);connection.timeout=1.
        return connection


def recover_finished_simulation(master,master_url,root):
    known={'/scan_command_gate','/closed_loop_controller','/scan_planner_node',
           '/reference_path_transform','/scan_visual_frame','/go2_physics_simulation','/rosout'}
    code,_,state=master.getSystemState('/simulation_recovery')
    names={n for group in state for _,nodes in group for n in nodes}
    if code!=1 or names-known:return False
    owned=finished_processes(root,master_url)
    if not owned:return False
    by_pid={p['pid']:p for p in owned}
    own_master=any(p['role']=='master' for p in owned)
    for name in names:
        if name=='/rosout':continue
        code,_,uri=master.lookupNode('/simulation_recovery',name)
        if code!=1 or urllib.parse.urlparse(uri).hostname not in {'127.0.0.1','localhost','::1'}:return False
        try:
            result=xmlrpc.client.ServerProxy(uri,transport=_TimeoutTransport()).getPid('/simulation_recovery')
            if result[0]!=1 or result[2] not in by_pid:return False
        except (OSError,xmlrpc.client.Error):
            if not own_master:return False
    print('清理已结束仿真的本机残留：'+', '.join(f"{p['role']} PID {p['pid']}" for p in owned),flush=True)
    def send(process,signum):
        if identity(process['pid'])==process['stamp']:
            try:os.kill(process['pid'],signum)
            except ProcessLookupError:pass
    # Let roslaunch shut down its own children before stopping the master.
    for role in ('planner','node','master'):
        targets=[p for p in owned if p['role']==role]
        for process in targets:send(process,signal.SIGINT)
        deadline=time.monotonic()+3.
        while time.monotonic()<deadline and any(identity(p['pid'])==p['stamp'] for p in targets):time.sleep(.1)
        for process in targets:send(process,signal.SIGTERM)
        deadline=time.monotonic()+2.
        while time.monotonic()<deadline and any(identity(p['pid'])==p['stamp'] for p in targets):time.sleep(.1)
        if any(identity(p['pid'])==p['stamp'] for p in targets):
            raise RuntimeError('已确认的旧仿真进程未及时退出，未启动新任务')
    return True
