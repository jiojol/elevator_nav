#!/usr/bin/env python3
"""The local simulation entry must reject remote masters before connecting."""
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'simulation'))
from run_case import local_master


class LocalSimulationTest(unittest.TestCase):
    def test_only_loopback_on_an_independent_port_is_allowed(self):
        for url in ['http://127.0.0.1:11791', 'http://localhost:11791', 'http://[::1]:11791']:
            self.assertEqual(local_master(url), 11791)
        for url in ['http://192.168.31.31:11791', 'http://192.168.31.9:11781',
                    'http://robot:11791', 'http://127.0.0.1:11311', 'http://127.0.0.1',
                    'https://127.0.0.1:11791', 'http://user:secret@127.0.0.1:11791']:
            with self.subTest(url=url), self.assertRaises(ValueError):
                local_master(url)

    def test_remote_master_is_rejected_before_build_or_ros_access(self):
        result = subprocess.run([sys.executable, str(ROOT/'simulation/run_case.py'),
            '--case', 'stairs', '--master', 'http://192.168.31.31:11791',
            '--output', '/tmp/unused-local-simulation-output'], capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 2)
        self.assertIn('仅允许本机回环地址', result.stderr)
        self.assertNotIn('Traceback', result.stderr)

    def test_old_remote_launch_options_cannot_enable_a_remote_mode(self):
        result = subprocess.run([sys.executable, str(ROOT/'simulation/run_case.py'),
            '--case', 'stairs', '--remote', '192.168.31.31', '--output', '/tmp/unused-local-simulation-output'],
            capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 2)
        self.assertIn('unrecognized arguments: --remote', result.stderr)


if __name__ == '__main__':
    unittest.main()
