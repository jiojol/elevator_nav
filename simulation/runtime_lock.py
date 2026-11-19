"""Serialize local simulation runs and retain readable ownership information."""
import fcntl
import json
import os
import time


class SimulationBusy(RuntimeError):
    pass


def owner_description(handle):
    handle.seek(0)
    try:
        owner = json.loads(handle.read())
    except (ValueError, TypeError):
        return '另一个本机仿真进程'
    return f"{owner.get('case', '?')} / {owner.get('mode', '?')}，PID {owner.get('pid', '?')}，输出 {owner.get('output', '?')}"


def acquire(build, owner, wait_seconds=0.):
    # Opening with 'w' would truncate the active owner's metadata on contention.
    handle = (build/'simulation.lock').open('a+')
    deadline = time.monotonic()+wait_seconds
    announced = False
    try:
        while True:
            try:
                fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except BlockingIOError:
                description = owner_description(handle)
                if time.monotonic() >= deadline:
                    raise SimulationBusy('本机仿真被占用：'+description+'。可等待原任务结束，或使用 --wait-for-slot 180。') from None
                if not announced:
                    print('等待仿真空闲：'+description+f'（最多等待 {wait_seconds:g} 秒）', flush=True)
                    announced = True
                time.sleep(min(.2, max(0., deadline-time.monotonic())))
        handle.seek(0)
        handle.truncate()
        handle.write(json.dumps({'pid': os.getpid(), **owner}, ensure_ascii=False))
        handle.flush()
        if announced:
            print('仿真资源已空闲，开始本次任务。', flush=True)
        return handle
    except BaseException:
        handle.close()
        raise
