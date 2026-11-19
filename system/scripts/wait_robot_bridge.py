#!/usr/bin/env python3
"""Wait for the restarted bridge to load its configured limits; issue no commands."""
import math
import os
from pathlib import Path
import socket
import sys
import time

import rospy
import yaml

socket.setdefaulttimeout(1.)
path = Path(os.environ.get('DOG_NAV_BRIDGE_CONFIG',
    '/home/nvidia/.local/state/modern-visual-cutover/20261027-web-static/sdk-control.yaml'))
config = yaml.safe_load(path.read_text())
if not isinstance(config, dict):
    raise SystemExit('底盘桥配置格式无效：' + str(path))
expected = {name: float(config[name]) for name in ('max_linear_x', 'max_angular_z')}
if not all(math.isfinite(value) and value > 0 for value in expected.values()):
    raise SystemExit('底盘桥配置中的速度上限无效。')
deadline = time.monotonic() + 20.
last_error = '底盘桥尚未就绪'
while time.monotonic() < deadline:
    try:
        if not rospy.get_param('/go2_ros_bridge/control_enabled', False):
            raise RuntimeError('底盘桥控制尚未启用')
        actual = {name: float(rospy.get_param('/go2_ros_bridge/' + name)) for name in expected}
        if all(math.isclose(actual[name], expected[name], rel_tol=1e-6, abs_tol=1e-6) for name in expected):
            print('底盘桥配置已加载：前进上限 %.3f m/s，转向上限 %.3f rad/s。' %
                  (actual['max_linear_x'], actual['max_angular_z']), flush=True)
            sys.exit(0)
        last_error = '运行限速仍与配置不一致：' + str(actual)
    except Exception as error:
        last_error = str(error)
    time.sleep(.25)
raise SystemExit('底盘桥启动等待超时：' + last_error +
    '\n请查看 sudo journalctl -u modern-visual-state.service -n 40 --no-pager')
