#!/usr/bin/env python3
"""Run navigation with one automatically retained recording session."""
import argparse
from datetime import datetime, timedelta, timezone
import fcntl
import json
import os
from pathlib import Path
import selectors
import shutil
import signal
import subprocess
import sys
import time


TOPICS = (
    '/tf', '/tf_static', '/rosout', '/rosout_agg',
    '/livox/lidar', '/livox/imu', '/livox/points',
    '/fastlio_nav/odometry', '/fastlio_nav/cloud_body',
    '/dog_nav/odometry', '/dog_nav/sensor_pose', '/dog_nav/localization_status',
    '/planning/body_odometry', '/grid_map/occupancy',
    '/dog_nav/goal_pose',
    '/initialpose', '/move_base_simple/goal', '/initial_path', '/planning/tracking_path',
    '/navigation/obstacle_points', '/globalmap',
    '/plan_map/free', '/plan_map/path', '/plan_map/global_path',
    '/plan_map/query_start', '/plan_map/query_goal', '/plan_map/cancel_scan',
    '/plan_map/start_pose', '/plan_map/goal_pose',
    '/planning/bspline', '/planning/data_display', '/planning/tracking_status', '/planning/local_planner_status',
    '/planning/collision_stop', '/planning/collision_control', '/planning/execution_stall',
    '/planning/start_alignment', '/planning/start_alignment_done',
    '/dog_nav/contact_stop', '/dog_nav/collision_status', '/dog_nav/footprint',
    '/planning/go2_execution_frozen', '/planning/local_segment_end',
    '/planning/goal_position_reached', '/planning/goal_position_lost',
    '/planning/goal_reached', '/planning/goal_failed', '/planning/reference_refresh',
    '/cmd_vel', '/planning/scan_cmd_vel', '/go2/odom', '/go2/imu',
    '/go2/status', '/go2/sportmodestate', '/go2/sportmodestate_lf',
    '/go2_ros_bridge/control_status',
    '/map_manager/status', '/map_manager/result', '/map_manager/cancel',
    '/map_manager/current_map',
    '/dog_nav/safety_stop', '/dog_nav/safety/reference', '/dog_nav/safety/bspline',
    '/dog_nav/safety/odometry', '/dog_nav/safety/obstacle_points',
)


def snapshot_parameters(destination):
    import socket
    import rosparam
    import yaml
    socket.setdefaulttimeout(3.)
    data = {}
    for name in ('/scan_planner_node', '/closed_loop_controller', '/scan_command_gate',
                 '/global_planner', '/fastlio_map_localizer', '/fastlio_nav',
                 '/go2_ros_bridge', '/dog_nav/geometry', '/plan_map/resolution', '/body_pose_topic', '/use_sim_time'):
        try:
            data[name] = rosparam.get_param(name)
        except Exception as exc:
            data[name] = {'snapshot_error': str(exc)}
    Path(destination).write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=True))


def stop_process(process, grace=20.):
    if process is None or process.poll() is not None:
        return
    for sig, timeout in ((signal.SIGINT, grace), (signal.SIGTERM, 5.), (signal.SIGKILL, 2.)):
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=timeout)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            continue


def signal_group(process, sig):
    try:
        os.killpg(process.pid, sig)
    except ProcessLookupError:
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', required=True, type=Path)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command or shutil.which(command[0]) is None or shutil.which('rosbag') is None:
        parser.error('navigation command and rosbag must both be available')
    import roslib.packages
    record_programs = roslib.packages.find_node('rosbag', 'record')
    if not record_programs:
        parser.error('native rosbag recorder executable is unavailable')
    workspace = args.workspace.resolve()
    if not (workspace / 'nav/src/dog_nav_bringup').is_dir():
        parser.error('workspace is not a dog_nav workspace')
    bags = workspace / 'system/bags'
    bags.mkdir(parents=True, exist_ok=True)
    lock = (bags / '.navigation_session.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('已有自动录包导航在运行，请先结束上一轮导航。')
    output = bags / 'navigation_latest'
    marker = output / '.dog_nav_auto_record'
    if output.is_symlink():
        raise SystemExit('自动录包目录不能是符号链接。')
    if output.exists():
        if not marker.is_file() or marker.read_text().strip() != str(workspace):
            raise SystemExit('录包目录没有匹配的自动录包标记，未删除：' + str(output))
        shutil.rmtree(output)
    output.mkdir()
    marker.write_text(str(workspace) + '\n')
    (output / 'recorded_topics.txt').write_text('\n'.join(TOPICS) + '\n')
    for source, name in (
        (workspace / 'nav/src/dog_nav_bringup/config/prior_local.yaml', 'scan.yaml'),
        (workspace / 'nav/src/dog_nav_bringup/config/fastlio_localization.yaml', 'fastlio_localization.yaml'),
        (workspace / 'nav/devel/scan_build_manifest.json', 'scan_build_manifest.json'),
    ):
        if source.is_file():
            shutil.copy2(source, output / name)
    map_meta = Path(next((arg.split(':=', 1)[1] for arg in reversed(command)
                          if arg.startswith('plan_meta:=')),
                         os.environ.get('DOG_NAV_PLAN_META', str(workspace / 'system/map/dog_plan.json'))))
    if map_meta.is_file():
        shutil.copy2(map_meta, output / 'plan_meta.json')
    session = dict(started_at=datetime.now(timezone(timedelta(hours=8))).isoformat(),
                   workspace=str(workspace), command=command,
                   map_pcd=os.environ.get('DOG_NAV_BUILDING_PCD'),
                   planning_map_meta=str(map_meta), retention='replace on next navigation start')
    (output / 'session.json').write_text(json.dumps(session, indent=2, ensure_ascii=False) + '\n')
    print('自动录包：' + str(output), flush=True)
    print('下一次启动导航时替换本目录；本轮结束时自动关闭 bag。', flush=True)

    recorder = navigation = None
    snapshots = []
    stop_signal = [0]
    def request_stop(sig, _frame):
        stop_signal[0] = sig
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, request_stop)
    selector = selectors.DefaultSelector()
    reason, result = 'startup_error', 1
    with (output / 'recorder.log').open('wb') as record_log, \
         (output / 'launch.log').open('wb', buffering=0) as launch_log, \
         (output / 'snapshot.log').open('wb') as snapshot_log:
        def snapshot(phase):
            proc = subprocess.Popen([sys.executable, str(Path(__file__).resolve()),
                                     '--snapshot', str(output / ('params_' + phase + '.yaml'))],
                                    stdout=snapshot_log, stderr=subprocess.STDOUT,
                                    start_new_session=True)
            snapshots.append((proc, time.monotonic()))
        def drain():
            for key, _ in selector.select(timeout=.10):
                try:
                    chunk = os.read(key.fd, 65536)
                except BlockingIOError:
                    continue
                if not chunk:
                    selector.unregister(key.fileobj)
                    continue
                launch_log.write(chunk)
                try:
                    os.write(sys.stdout.fileno(), chunk)
                except (BrokenPipeError, OSError):
                    stop_signal[0] = signal.SIGHUP
        try:
            recorder = subprocess.Popen(
                [record_programs[0], '--lz4', '--split', '--size=1024', '--buffsize=128',
                 '-O', str(output / 'navigation.bag'), *TOPICS],
                stdout=record_log, stderr=subprocess.STDOUT, start_new_session=True,
                pass_fds=(lock.fileno(),))
            navigation = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                          start_new_session=True, pass_fds=(lock.fileno(),),
                                          env=dict(os.environ, PYTHONUNBUFFERED='1'))
            os.set_blocking(navigation.stdout.fileno(), False)
            selector.register(navigation.stdout, selectors.EVENT_READ)
            started, stop_started, stop_stage = time.monotonic(), None, 0
            start_snapshot = False
            while navigation.poll() is None:
                drain()
                elapsed = time.monotonic() - started
                if not start_snapshot and elapsed >= 3.:
                    snapshot('start')
                    start_snapshot = True
                if recorder.poll() is not None and not stop_signal[0]:
                    reason = 'recorder_exited'
                    print('录包进程退出，停止本轮导航；详见 recorder.log。', file=sys.stderr, flush=True)
                    stop_signal[0] = signal.SIGTERM
                if elapsed > 15. and not any(output.glob('*.bag*')) and not stop_signal[0]:
                    reason = 'recorder_did_not_open_bag'
                    print('录包未生成 bag，停止本轮导航。', file=sys.stderr, flush=True)
                    stop_signal[0] = signal.SIGTERM
                if stop_signal[0] and stop_started is None:
                    signal_group(navigation, signal.SIGINT)
                    stop_started = time.monotonic()
                    if reason == 'startup_error':
                        reason = 'signal_%d' % stop_signal[0]
                if stop_started is not None and time.monotonic() - stop_started > (20. if stop_stage == 0 else 5.):
                    signal_group(navigation, signal.SIGTERM if stop_stage == 0 else signal.SIGKILL)
                    stop_stage += 1
                    stop_started = time.monotonic()
                for proc, began in snapshots:
                    if proc.poll() is None and time.monotonic() - began > 10.:
                        signal_group(proc, signal.SIGTERM)
            drain()
            result = navigation.returncode
            if stop_signal[0]:
                result = 1 if reason.startswith('recorder_') else 128 + stop_signal[0]
            else:
                reason = 'navigation_exited'
        finally:
            stop_process(navigation)
            # Keep recording through navigation shutdown and its final stop.
            stop_process(recorder)
            snapshot('end')
            for proc, _ in snapshots:
                try:
                    proc.wait(timeout=4.)
                except subprocess.TimeoutExpired:
                    stop_process(proc, grace=1.)
            selector.close()
            session.update(ended_at=datetime.now(timezone(timedelta(hours=8))).isoformat(),
                           exit_reason=reason, exit_code=result)
            (output / 'session.json').write_text(json.dumps(session, indent=2, ensure_ascii=False) + '\n')
            print('本轮导航录包已保留：' + str(output), flush=True)
    return result


if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[1] == '--snapshot':
        snapshot_parameters(sys.argv[2])
    else:
        raise SystemExit(main())
