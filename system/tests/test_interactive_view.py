#!/usr/bin/env python3
"""Local goal input and control commands for the interactive simulator."""
import json
import math
from pathlib import Path
import sys
import unittest
import urllib.error
import urllib.request

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'simulation'))
from interactive_view import InteractiveView, validate_command

BOXES=[('static_box',[0.,0.,.4],[.5,.55,.4])]


class InteractiveViewTest(unittest.TestCase):
    def test_goal_validation_rejects_invalid_and_occupied_targets(self):
        for goal in [dict(action='goal',x=math.nan,y=0.,yaw=0.),dict(action='goal',x=0.,y=0.,yaw=0.),dict(action='goal',x=6.,y=0.,yaw=0.)]:
            with self.assertRaises(ValueError):validate_command(goal,BOXES,'large_obstacle')
        valid=validate_command(dict(action='goal',x=2.,y=1.,yaw=3.*math.pi),BOXES,'large_obstacle')
        self.assertAlmostEqual(abs(valid['yaw']),math.pi)

    def test_stair_surface_can_be_selected_but_outside_prior_cannot(self):
        stairs=[('stair_01',[.45,0.,.18],[.15,.8,.06])]
        goal=dict(action='goal',x=.45,y=0.,yaw=0.)
        self.assertEqual(validate_command(goal,stairs,'stairs')['x'],.45)
        with self.assertRaises(ValueError):validate_command({**goal,'y':1.},stairs,'stairs')

    def test_http_gates_goals_until_ready_but_accepts_stop_and_reset(self):
        view=InteractiveView(0,BOXES,'large_obstacle')
        opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
        url=f'http://127.0.0.1:{view.port}'
        def post(command):
            return opener.open(urllib.request.Request(url+'/api/command',data=json.dumps(command).encode(),headers={'Content-Type':'application/json'}))
        try:
            goal=dict(action='goal',x=2.,y=1.,yaw=0.)
            with self.assertRaises(urllib.error.HTTPError) as error:post(goal)
            self.assertEqual(error.exception.code,400)
            self.assertTrue(view.commands.empty())
            with post({'action':'stop'}) as response:self.assertEqual(response.status,200)
            self.assertEqual(view.commands.get_nowait()['action'],'stop')
            view.update({'ready':True,'fallen':False,'status':'等待选择目标'},b'frame')
            with post(goal) as response:self.assertEqual(response.status,200)
            self.assertEqual(view.commands.get_nowait()['action'],'goal')
            for action in ['reset','close']:
                with post({'action':action}) as response:self.assertEqual(response.status,200)
                self.assertEqual(view.commands.get_nowait()['action'],action)
            view.update({'ready':True,'fallen':True},b'frame')
            with self.assertRaises(urllib.error.HTTPError):post(goal)
        finally:view.close()

    def test_server_listens_on_loopback_and_serves_current_state(self):
        view=InteractiveView(0,BOXES,'large_obstacle')
        try:
            self.assertEqual(view.server.server_address[0],'127.0.0.1')
            view.update({'ready':True,'status':'导航中'},b'jpeg-frame')
            opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
            with opener.open(f'http://127.0.0.1:{view.port}/api/state') as response:
                self.assertEqual(json.load(response)['status'],'导航中')
            with opener.open(f'http://127.0.0.1:{view.port}/api/frame') as response:
                self.assertEqual(response.read(),b'jpeg-frame')
        finally:view.close()


if __name__=='__main__':unittest.main()
