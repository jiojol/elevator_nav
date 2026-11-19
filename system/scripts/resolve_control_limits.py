#!/usr/bin/env python3
"""Resolve planner speed limits against the deployed chassis bridge, without motion."""
import math
import os
from pathlib import Path
import socket
import sys

linear, yaw = map(float, sys.argv[1:3])
if not all(math.isfinite(v) and v > 0 for v in (linear, yaw)):
    raise SystemExit('导航速度上限必须是有限正数。')
if sys.argv[3] == 'false':
    import rospy
    socket.setdefaulttimeout(3.)
    # This device's service loads these limits at process startup. A changed
    # file is not a live update; reject stale runtime settings before recording.
    config_path = Path(os.environ.get('DOG_NAV_BRIDGE_CONFIG',
        '/home/nvidia/.local/state/modern-visual-cutover/20261027-web-static/sdk-control.yaml'))
    configured = {}
    if config_path.is_file():
        import yaml
        configured = yaml.safe_load(config_path.read_text()) or {}
        if not isinstance(configured, dict):
            raise SystemExit('底盘桥配置格式无效：' + str(config_path))
    for key, requested in (('max_linear_x', linear), ('max_angular_z', yaw)):
        try:
            limit = rospy.get_param('/go2_ros_bridge/' + key, None)
        except Exception as error:
            raise SystemExit('无法读取底盘桥限速，请检查 ROS master：' + str(error))
        if limit is None:
            print('未读取到底盘桥 %s，保留请求值 %.3f。' % (key, requested), file=sys.stderr)
            continue
        limit = float(limit)
        if not math.isfinite(limit) or limit <= 0:
            raise SystemExit('底盘桥 %s 无效。' % key)
        expected = configured.get(key)
        if expected is not None:
            expected = float(expected)
            if not math.isfinite(expected) or expected <= 0:
                raise SystemExit('底盘桥配置 %s 无效。' % key)
            if not math.isclose(limit, expected, rel_tol=1e-6, abs_tol=1e-6):
                raise SystemExit(
                    '底盘桥尚未加载新限速：%s 运行值 %.3f，配置值 %.3f。\n'
                    '请先停止导航，再执行 sudo systemctl restart modern-visual-state.service，随后重新启动导航。'
                    % (key, limit, expected))
        effective = min(requested, limit)
        if effective < requested:
            print('导航 %s 请求 %.3f，按底盘桥上限 %.3f 配速。' % (key, requested, effective), file=sys.stderr)
        if key == 'max_linear_x':
            linear = effective
        else:
            yaw = effective
print('scan_max_vel:=%.9g' % linear)
print('scan_max_yaw:=%.9g' % yaw)
