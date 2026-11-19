#!/usr/bin/env python3
"""Termination must run cleanup and never recover unrelated ROS nodes."""
import io
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock,patch

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'simulation'))
from process_lifecycle import recover_finished_simulation,identity


class ProcessLifecycleTest(unittest.TestCase):
    def test_sigterm_executes_runner_finally(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'cleaned'
            script=("import sys,time;from pathlib import Path;sys.path.insert(0,sys.argv[1]);"
                    "from process_lifecycle import install_exit_handlers;install_exit_handlers();print('READY',flush=True)\n"
                    "try: time.sleep(30)\nfinally: Path(sys.argv[2]).write_text('cleaned')\n")
            process=subprocess.Popen([sys.executable,'-c',script,str(ROOT/'simulation'),str(path)],stdout=subprocess.PIPE,text=True)
            try:
                self.assertEqual(process.stdout.readline().strip(),'READY')
                process.terminate();self.assertEqual(process.wait(timeout=3),128+signal.SIGTERM)
                self.assertEqual(path.read_text(),'cleaned')
            finally:
                if process.poll() is None:process.kill();process.wait()
                process.stdout.close()

    def test_parent_kill_delivers_interrupt_to_child(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'child_cleaned'
            script=("import subprocess,sys,time;sys.path.insert(0,sys.argv[1]);from process_lifecycle import parent_exit_hook\n"
                    "child=\"import signal,sys,time;from pathlib import Path\\ndef stop(*_): Path(sys.argv[1]).write_text('cleaned');raise SystemExit(0)\\nsignal.signal(signal.SIGINT,stop);print('READY',flush=True);time.sleep(30)\"\n"
                    "p=subprocess.Popen([sys.executable,'-c',child,sys.argv[2]],preexec_fn=parent_exit_hook());print('CHILD '+str(p.pid),flush=True);time.sleep(30)\n")
            process=subprocess.Popen([sys.executable,'-c',script,str(ROOT/'simulation'),str(path)],stdout=subprocess.PIPE,text=True,start_new_session=True)
            child=None
            try:
                lines=[process.stdout.readline().strip() for _ in range(2)]
                self.assertIn('READY',lines);child=int(next(v for v in lines if v.startswith('CHILD ')).split()[1])
                process.kill();process.wait(timeout=3)
                deadline=time.monotonic()+3
                while not path.exists() and time.monotonic()<deadline:time.sleep(.02)
                self.assertEqual(path.read_text(),'cleaned')
                deadline=time.monotonic()+3
                while identity(child) is not None and time.monotonic()<deadline:time.sleep(.02)
                self.assertIsNone(identity(child))
            finally:
                if process.poll() is None:process.kill();process.wait()
                if child is not None and identity(child) is not None:os.kill(child,signal.SIGKILL)
                process.stdout.close()

    def test_unrelated_master_nodes_are_never_recovered(self):
        master=Mock();master.getSystemState.return_value=[1,'',[[['/cmd_vel',['/real_robot_controller']]],[],[]]]
        with patch('process_lifecycle.finished_processes') as inspect:
            self.assertFalse(recover_finished_simulation(master,'http://127.0.0.1:11791',ROOT))
            inspect.assert_not_called()


if __name__=='__main__':unittest.main()
