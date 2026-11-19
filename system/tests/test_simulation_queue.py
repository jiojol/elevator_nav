#!/usr/bin/env python3
"""Concurrent runs must queue without losing the active owner's information."""
from contextlib import redirect_stdout
import io
import json
import multiprocessing
import os
from pathlib import Path
import signal
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'simulation'))
from runtime_lock import acquire, SimulationBusy
import run_matrix
from run_matrix import run_logged


def hold_slot(directory, ready, release):
    with acquire(Path(directory), {'case': 'stairs', 'mode': 'native', 'output': 'active_stairs'}):
        ready.set()
        release.wait(timeout=5)


class SimulationQueueTest(unittest.TestCase):
    def test_contention_keeps_owner_and_can_wait_for_release(self):
        with tempfile.TemporaryDirectory() as directory:
            ready, release = multiprocessing.Event(), multiprocessing.Event()
            process = multiprocessing.Process(target=hold_slot, args=(directory, ready, release))
            process.start()
            try:
                self.assertTrue(ready.wait(timeout=3))
                before = (Path(directory)/'simulation.lock').read_text()
                with self.assertRaisesRegex(SimulationBusy, 'stairs / native.*PID'):
                    acquire(Path(directory), {'case': 'straight'}, wait_seconds=0.)
                self.assertEqual((Path(directory)/'simulation.lock').read_text(), before)
                # A separate releaser lets this thread block on the real flock.
                timer = threading.Timer(.2, release.set)
                timer.start()
                output = io.StringIO()
                with redirect_stdout(output):
                    with acquire(Path(directory), {'case': 'straight'}, wait_seconds=3.):
                        self.assertEqual(json.loads((Path(directory)/'simulation.lock').read_text())['case'], 'straight')
                timer.join()
                self.assertIn('等待仿真空闲：stairs', output.getvalue())
                self.assertIn('仿真资源已空闲', output.getvalue())
            finally:
                release.set()
                process.join(timeout=3)
                if process.is_alive():
                    process.terminate()
                    process.join()

    def test_old_owner_text_without_a_lock_does_not_block_a_new_run(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'simulation.lock'
            path.write_text('{"case":"old","pid":123}')
            with acquire(Path(directory), {'case': 'new'}):
                self.assertEqual(json.loads(path.read_text())['case'], 'new')

    def test_child_error_is_visible_in_terminal_and_saved_in_log(self):
        with tempfile.TemporaryDirectory() as directory:
            output = io.StringIO()
            path = Path(directory)/'console.log'
            with redirect_stdout(output):
                code = run_logged([sys.executable, '-c', "print('startup failed: test reason', flush=True); raise SystemExit(2)"], path)
            self.assertEqual(code, 2)
            self.assertIn('startup failed: test reason', output.getvalue())
            self.assertEqual(output.getvalue(), path.read_text())

    def test_interrupt_allows_child_cleanup_to_finish(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'console.log'
            cleaned = Path(directory)/'cleaned'
            script = ("import pathlib,sys,time\ntry:\n print('READY', flush=True)\n time.sleep(5)\n"
                      "finally:\n pathlib.Path(sys.argv[1]).write_text('cleaned')\n")
            def interrupt_when_ready():
                for _ in range(100):
                    if path.exists() and 'READY' in path.read_text():
                        os.kill(os.getpid(), signal.SIGINT)
                        return
                    time.sleep(.02)
            interrupter = threading.Thread(target=interrupt_when_ready)
            interrupter.start()
            try:
                with redirect_stdout(io.StringIO()), self.assertRaises(KeyboardInterrupt):
                    run_logged([sys.executable, '-c', script, str(cleaned)], path)
            finally:
                interrupter.join(timeout=3)
            self.assertEqual(cleaned.read_text(), 'cleaned')

    def test_matrix_failure_reports_the_specific_reason_and_nonzero_exit(self):
        for code, expected, message in [
                (75, 'busy', '本机仿真被占用：stairs，PID 123'),
                (2, 'startup_error', '缺少本机仿真文件'),
                (0, 'startup_error', 'child exited without a result')]:
            with self.subTest(code=code), tempfile.TemporaryDirectory() as directory:
                def child(command, log_path):
                    log_path.write_text(message+'\n')
                    return code
                output = io.StringIO()
                with patch.object(sys, 'argv', ['run_matrix.py', '--output', directory]), \
                        patch.object(run_matrix, 'run_logged', side_effect=child), \
                        redirect_stdout(output), self.assertRaises(SystemExit) as error:
                    run_matrix.main()
                self.assertNotEqual(error.exception.code, 0)
                result = json.loads(output.getvalue().splitlines()[-1])
                self.assertEqual(result['result'], expected)
                self.assertEqual(result['error'], message if code else '仿真进程未生成 result.json')
                self.assertNotIn('harness_error', output.getvalue())


if __name__ == '__main__':
    unittest.main()
