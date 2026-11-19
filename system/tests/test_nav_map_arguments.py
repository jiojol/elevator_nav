#!/usr/bin/env python3
"""Exercise start_nav.sh path forwarding without starting ROS or hardware."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class MapArgumentTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='dog-nav-map-arguments-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        scripts = self.root / 'system/scripts'
        scripts.mkdir(parents=True)
        for name in ('start_nav.sh', 'env.sh', 'navigation_env.sh', 'resolve_control_limits.py'):
            shutil.copy2(ROOT / 'system/scripts' / name, scripts / name)
        # Keep argument forwarding isolated from the recording supervisor.
        (scripts / 'navigation_session.py').write_text(
            'import os,sys\ncommand=sys.argv[sys.argv.index("--")+1:]\n'
            'os.execvp(command[0], command)\n')
        plan_scripts = self.root / 'nav/src/map_for_plan/scripts' 
        plan_scripts.mkdir(parents=True)
        shutil.copy2(ROOT / 'nav/src/map_for_plan/scripts/plan_io.py', plan_scripts / 'plan_io.py')
        setup = self.root / 'nav/devel/setup.bash'
        setup.parent.mkdir(parents=True)
        setup.write_text('# isolated test setup\n')
        self.ros_setup = self.root / 'ros_setup.bash'
        self.ros_setup.write_text('# isolated test setup\n')
        maps = self.root / 'system/map'
        maps.mkdir(parents=True)
        (maps / 'site1.pcd').write_text('localization fixture')
        (maps / 'site1_free.pcd').write_text('free map fixture')
        (maps / 'site1_path.tum').write_text('0 0 0 0 0 0 0 1\n')
        metadata = {'free_pcd': 'site1_free.pcd', 'path_tum': 'site1_path.tum'}
        for name in ('site1_plan.json', 'alternate_plan.json'):
            (maps / name).write_text(json.dumps(metadata))
        (maps / 'dog.pcd').write_text('latest mapping localization fixture')
        (maps / 'dog_free.pcd').write_text('latest mapping free fixture')
        (maps / 'dog_path.tum').write_text('0 0 0 0 0 0 0 1\n')
        (maps / 'dog_plan.json').write_text(json.dumps(
            {'free_pcd': 'dog_free.pcd', 'path_tum': 'dog_path.tum'}))
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        node = self.bin / 'rosnode'
        node.write_text('#!/bin/sh\nexit 0\n')
        node.chmod(0o755)
        launcher = self.bin / 'roslaunch'
        launcher.write_text('#!/usr/bin/python3\nimport json,os,sys\n'
                            'with open(os.environ["DOG_NAV_TEST_ARGS"], "w") as f: json.dump(sys.argv[1:], f)\n')
        launcher.chmod(0o755)
        self.capture = self.root / 'launch_args.json'

    def launch(self, *arguments, use_defaults=False):
        env = os.environ.copy()
        for name in list(env):
            if name.startswith(('DOG_NAV_',)):
                env.pop(name)
        env.update(DOG_NAV_HOME=str(self.root),
                   ROS_SETUP=str(self.ros_setup), DOG_NAV_TEST_ARGS=str(self.capture),
                   PATH=str(self.bin) + os.pathsep + env['PATH'])
        if not use_defaults:
            env['DOG_NAV_BUILDING_PCD'] = str(self.root / 'system/map/site1.pcd')
        subprocess.run(['bash', str(self.root / 'system/scripts/start_nav.sh'),
                        'planner_backend:=tum', 'planning_only:=true', 'start_rviz:=false',
                        *arguments], cwd='/tmp', env=env, check=True, capture_output=True, text=True)
        return json.loads(self.capture.read_text())

    def test_default_navigation_uses_the_current_mapping_output(self):
        args = self.launch(use_defaults=True)
        self.assertIn('globalmap_pcd:=' + str(self.root / 'system/map/dog.pcd'), args)
        self.assertIn('plan_meta:=' + str(self.root / 'system/map/dog_plan.json'), args)
        self.assertFalse(any('site1' in arg for arg in args))

    def test_missing_current_map_does_not_fall_back_to_site1(self):
        (self.root / 'system/map/dog.pcd').unlink()
        with self.assertRaises(subprocess.CalledProcessError) as error:
            self.launch(use_defaults=True)
        self.assertIn('dog.pcd', error.exception.stderr)
        self.assertFalse(self.capture.exists())

    def test_relative_metadata_is_forwarded_once_as_an_absolute_path(self):
        args = self.launch('plan_meta:=system/map/site1_plan.json', 'scan_max_vel:=0.20')
        self.assertEqual([x for x in args if x.startswith('plan_meta:=')],
                         ['plan_meta:=' + str(self.root / 'system/map/site1_plan.json')])
        speed = next(x.split(':=', 1)[1] for x in args if x.startswith('scan_max_vel:='))
        self.assertEqual(float(speed), .20)

    def test_imported_free_pcd_map_without_tum_can_start(self):
        path = self.root / 'system/map/pcd_only_plan.json'
        path.write_text(json.dumps({'free_pcd': 'site1_free.pcd', 'resolution': .2}))
        args = self.launch('plan_meta:=' + str(path))
        self.assertIn('plan_meta:=' + str(path), args)

    def test_free_backend_does_not_load_a_legacy_tum_reference(self):
        path = self.root / 'system/map/missing_tum_plan.json'
        path.write_text(json.dumps({'free_pcd': 'site1_free.pcd', 'path_tum': 'missing.tum'}))
        args = self.launch('plan_meta:=' + str(path))
        self.assertIn('planner_backend:=free', args)
        self.assertNotIn('planner_backend:=tum', args)

    def test_missing_free_pcd_is_still_rejected(self):
        path = self.root / 'system/map/missing_free_plan.json'
        path.write_text(json.dumps({'free_pcd': 'missing_free.pcd'}))
        with self.assertRaises(subprocess.CalledProcessError):
            self.launch('plan_meta:=' + str(path))

    def test_absolute_metadata_path_is_preserved(self):
        path = self.root / 'system/map/site1_plan.json'
        args = self.launch('plan_meta:=' + str(path))
        self.assertEqual([x for x in args if x.startswith('plan_meta:=')], ['plan_meta:=' + str(path)])

    def test_last_metadata_selection_wins_without_reintroducing_a_relative_override(self):
        args = self.launch('plan_meta:=system/map/site1_plan.json', 'plan_meta:=system/map/alternate_plan.json')
        self.assertEqual([x for x in args if x.startswith('plan_meta:=')],
                         ['plan_meta:=' + str(self.root / 'system/map/alternate_plan.json')])


if __name__ == '__main__':
    unittest.main()
