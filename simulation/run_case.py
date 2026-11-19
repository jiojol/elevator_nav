#!/usr/bin/env python3
"""Run SCAN and MuJoCo entirely on this computer, using loopback ROS only."""
import argparse
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import platform
import signal
import socket
import subprocess
import sys
import time
import urllib.parse
import urllib.request
import xmlrpc.client
from build_local import DEFAULT_BUILD, verify
from runtime_lock import acquire, SimulationBusy
from process_lifecycle import parent_exit_hook, install_exit_handlers, recover_finished_simulation

ROOT = Path(__file__).resolve().parents[1]
CASES = ['straight', 'final_yaw', 'large_obstacle', 'door_065', 'door_055',
         'dynamic_crossing', 'blocking_then_clear', 'stairs']


def loopback_host(host):
    if host == 'localhost':
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


def local_master(url):
    parsed = urllib.parse.urlparse(url)
    host = parsed.hostname
    if parsed.scheme != 'http' or parsed.port is None or parsed.port == 11311:
        raise ValueError('仿真需要独立的本机 ROS 端口，例如 http://127.0.0.1:11791')
    if not loopback_host(host):
        raise ValueError('仅允许本机回环地址；不会连接机器狗或远端 ROS master')
    if parsed.username or parsed.password:
        raise ValueError('ROS master 地址不应包含认证信息')
    return parsed.port


def nodes(master):
    code, message, state = master.getSystemState('/simulation_orchestrator')
    if code != 1:
        raise RuntimeError(message)
    return {n for group in state for _, names in group for n in names}


def stop(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--case', required=True, choices=CASES)
    p.add_argument('--mode', choices=['native'], default='native')
    p.add_argument('--motion', choices=['hover', 'dynamic'], default='hover', help='Hover body for navigation tests, or the original ONNX gait')
    p.add_argument('--hover-height', type=float, default=.4, help='Hover body centre height above the support surface (metres)')
    p.add_argument('--duration', type=float, default=None)
    p.add_argument('--master', default='http://127.0.0.1:11791')
    p.add_argument('--build', type=Path, default=DEFAULT_BUILD)
    p.add_argument('--physics-python', type=Path, default=ROOT/'simulation/artifacts/physics_env/bin/python')
    p.add_argument('--uv', type=Path, default=ROOT/'simulation/artifacts/physics_env/bin/uv')
    p.add_argument('--assets', type=Path, default=Path.home()/'代码合集/MuJoCo-LiDAR')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--planner-debug', action='store_true')
    p.add_argument('--wait-for-slot', type=float, default=0., help='Seconds to wait for another local simulation to finish')
    p.add_argument('--quiet-result', action='store_true', help='Keep the full result in result.json instead of printing it')
    p.add_argument('--interactive', action='store_true', help='Live view with manual map goals')
    p.add_argument('--ui-port', type=int, default=11882)
    p.add_argument('--no-browser', action='store_true', help='Print the interactive URL without opening a browser')
    args = p.parse_args()
    if args.duration is None:args.duration=3600. if args.interactive else 45.
    try:
        port = local_master(args.master)
    except ValueError as exc:
        p.error(str(exc))
    if args.duration <= 2.:
        p.error('--duration must exceed the 2 second settling period')
    if not .1 < args.hover_height < float('inf'):
        p.error('--hover-height must be finite and greater than the .1 m body half-height')
    if not 1024 <= args.ui_port <= 65535:p.error('--ui-port must be in 1024..65535')
    if args.wait_for_slot < 0. or not args.wait_for_slot < float('inf'):
        p.error('--wait-for-slot must be finite and nonnegative')
    required=[args.physics_python,args.uv]
    if args.motion=='dynamic':required.append(args.assets/'models/unitree_go2/go2_mjx.xml')
    for path in required:
        if not path.is_file():
            p.error('缺少本机仿真文件：'+str(path))
    build = args.build.resolve()
    try:
        verify(build)
    except (OSError, ValueError, RuntimeError) as exc:
        p.error(str(exc)+'；请先执行 python3 simulation/build_local.py')
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    if (args.output/'result.json').exists():
        p.error('该目录已有试验结果，请使用新的 --output 目录')
    # One local command chain at a time. Do not interfere with another trial.
    lock = acquire(build, {'case': args.case, 'mode': args.mode,
                         'output': str(args.output), 'master': args.master}, args.wait_for_slot)
    prefix = build/'devel'
    env = os.environ.copy()
    for name in ['ROS_HOSTNAME', 'ROS_NAMESPACE', 'GO2_SIM_SSH_PASSWORD']:
        env.pop(name, None)
    env.update(ROS_MASTER_URI=args.master, ROS_IP='127.0.0.1',
               ROS_LOG_DIR=str(args.output/'ros_logs'),
               OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
    env['PATH'] = '/opt/ros/noetic/bin:/usr/bin:/bin:'+env.get('PATH', '')
    env['CMAKE_PREFIX_PATH'] = str(prefix)+':/opt/ros/noetic'
    env['ROS_PACKAGE_PATH'] = str(prefix/'share')+':/opt/ros/noetic/share'
    env['LD_LIBRARY_PATH'] = str(prefix/'lib')+':'+str(ROOT/'simulation/artifacts/local_dependencies/tf/lib')+':/opt/ros/noetic/lib'
    env['PYTHONPATH'] = os.pathsep.join([
        str(prefix/'lib/python3/dist-packages'), '/opt/ros/noetic/lib/python3/dist-packages'])
    master = xmlrpc.client.ServerProxy(args.master)
    roscore = planner = physics = None
    handles = []
    started = time.monotonic()
    observed_nodes = {}

    def launch(command, log):
        handle = (args.output/log).open('w')
        handles.append(handle)
        return subprocess.Popen(command, env=env, stdout=handle, stderr=subprocess.STDOUT,
                                start_new_session=True,preexec_fn=parent_exit_hook())

    def ensure_master():
        nonlocal roscore
        try:
            master.getPid('/simulation_orchestrator')
        except OSError:
            roscore = launch(['/opt/ros/noetic/bin/roscore', '-p', str(port)], 'master.log')
            for _ in range(100):
                try:
                    if master.getPid('/simulation_orchestrator')[0] == 1:
                        break
                except OSError:
                    pass
                if roscore.poll() is not None:
                    raise RuntimeError('本机 ROS master 启动失败，见 master.log')
                time.sleep(.1)
            else:
                raise RuntimeError('本机 ROS master 启动超时')

    try:
        ensure_master()
        local_master(master.getUri('/simulation_orchestrator')[2])
        existing = nodes(master)
        if existing-{'/rosout'} and recover_finished_simulation(master,args.master,ROOT):
            ensure_master()
            existing=nodes(master)
        if existing-{'/rosout'}:
            raise RuntimeError('该端口仍有未确认归属的活动节点，保留原进程：'+str(existing))
        planner = launch(['/opt/ros/noetic/bin/roslaunch', str(ROOT/'simulation/isolated.launch'),
                          'body_height:='+str(args.hover_height+.133359 if args.motion=='hover' else .4)], 'planner.log')
        for _ in range(150):
            active = nodes(master)
            if {'/scan_planner_node', '/closed_loop_controller', '/scan_command_gate'} <= active:
                break
            if planner.poll() is not None:
                raise RuntimeError('本机规划器启动失败，见 planner.log')
            time.sleep(.1)
        else:
            raise RuntimeError('本机规划节点启动超时，见 planner.log')
        for name in nodes(master):
            code, message, uri = master.lookupNode('/simulation_orchestrator', name)
            if code != 1 or not loopback_host(urllib.parse.urlparse(uri).hostname):
                raise RuntimeError('发现非本机节点，停止本次试验：'+name)
            observed_nodes[name] = uri
        if args.planner_debug:
            subprocess.run(['/opt/ros/noetic/bin/rosservice', 'call', '/scan_planner_node/set_logger_level',
                            '{logger: ros, level: DEBUG}'], env=env, check=True, stdout=subprocess.DEVNULL)
        uv_python = [str(args.uv.resolve()), 'run', '--no-project', str(args.physics_python.absolute())]
        physics_command=[*uv_python, str(ROOT/'simulation/physics_bridge.py'), '--case', args.case,
                          '--duration', str(args.duration), '--assets', str(args.assets.resolve()),
                          '--output', str(args.output),'--motion',args.motion,'--hover-height',str(args.hover_height)]
        if args.interactive:physics_command+=['--interactive','--ui-port',str(args.ui_port)]
        physics = launch(physics_command, 'physics.log')
        print(f'本机仿真已启动：{args.case} / {args.mode} / {args.motion}，输出：{args.output}', flush=True)
        if args.interactive:
            url=f'http://127.0.0.1:{args.ui_port}'
            opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
            for _ in range(150):
                if physics.poll() is not None:break
                try:
                    with opener.open(url+'/api/state',timeout=.3):pass
                    print('交互页面：'+url+'，请在地图上选点；无需自动跑批量任务。',flush=True)
                    if not args.no_browser:
                        subprocess.Popen(['/usr/bin/xdg-open',url],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
                    break
                except OSError:time.sleep(.1)
            else:raise RuntimeError('交互页面启动超时，见 physics.log')
        try:
            code = physics.wait(timeout=args.duration*2+40.)
        except subprocess.TimeoutExpired:
            raise RuntimeError('物理仿真未及时结束，见 physics.log') from None
        if code:
            raise RuntimeError('物理仿真退出，见 physics.log（exit '+str(code)+'）')
    finally:
        for signum in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(signum,signal.SIG_IGN)
        for process in [physics, planner]:
            stop(process)
        # A master opened by the user remains open; only stop our own process.
        stop(roscore)
        for handle in handles:
            handle.close()
        metadata = {
            'case': args.case, 'mode': args.mode, 'execution': 'local_only',
            'architecture': platform.machine(), 'host': socket.gethostname(),
            'isolated_master': args.master, 'observed_planning_node_uris': observed_nodes,
            'remote_connections': [], 'robot_bridge_loaded': False,
            'wall_seconds': time.monotonic()-started,
            'local_source_revision': subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip(),
            'local_build_manifest_sha256': hashlib.sha256((build/'manifest.json').read_bytes()).hexdigest(),
            'runner_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'physics_sha256': hashlib.sha256((ROOT/'simulation/physics_bridge.py').read_bytes()).hexdigest(),
            'interactive':args.interactive,
            'motion':args.motion,'hover_height':args.hover_height if args.motion=='hover' else None,
        }
        (args.output/'run_metadata.json').write_text(json.dumps(metadata, indent=2)+'\n')
    if not args.quiet_result:
        print((args.output/'result.json').read_text())


if __name__ == '__main__':
    install_exit_handlers()
    try:
        main()
    except SimulationBusy as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(75) from None
    except (OSError, RuntimeError, ValueError, xmlrpc.client.Error) as exc:
        raise SystemExit(str(exc)) from None
