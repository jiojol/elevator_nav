#!/usr/bin/env python3
"""Sequential cases share one isolated master, never concurrent controllers."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import signal
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
CASES = {
    'straight': 30, 'final_yaw': 45, 'large_obstacle': 50,
    'door_065': 35, 'door_055': 35, 'dynamic_crossing': 40,
    'blocking_then_clear': 45, 'stairs': 65,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'simulation/artifacts'/datetime.now().strftime('%Y%m%d_%H%M%S'))
    parser.add_argument('--motion',choices=['hover','dynamic'],default='hover')
    parser.add_argument('--hover-height',type=float,default=.4)
    parser.add_argument('--wait-for-slot', type=float, default=180., help='Wait for a running local simulation before starting (seconds)')
    args = parser.parse_args()
    args.output = args.output.resolve()
    if args.wait_for_slot < 0. or not args.wait_for_slot < float('inf'):
        parser.error('--wait-for-slot must be finite and nonnegative')
    cases = [(case, 'native', duration) for case, duration in CASES.items()]
    for index, (case, mode, duration) in enumerate(cases, 1):
        folder = args.output/(mode+'_'+case)
        folder.mkdir(parents=True, exist_ok=False)
        command = [sys.executable, str(ROOT/'simulation/run_case.py'), '--case', case,
                   '--mode', mode, '--duration', str(duration), '--output', str(folder),
                   '--wait-for-slot', str(args.wait_for_slot), '--quiet-result']
        command+=['--motion',args.motion,'--hover-height',str(args.hover_height)]
        print(f'[{index}/{len(cases)}] {case} / {mode}', flush=True)
        returncode = run_logged(command, folder/'console.log')
        missing_result = returncode == 0 and not (folder/'result.json').exists()
        if missing_result:
            returncode = 1
        report = json.loads((folder/'result.json').read_text()) if (folder/'result.json').exists() else {
            'result': 'busy' if returncode == 75 else 'startup_error'}
        summary = {'case': case, 'mode': mode, 'returncode': returncode,
                          'result': report['result'], 'duration': report.get('duration_sim'),
                          'contacts': report.get('counts', {}), 'log': str(folder/'console.log')}
        if returncode:
            lines = (folder/'console.log').read_text().splitlines()
            summary['error'] = ('仿真进程未生成 result.json' if missing_result else
                                next((line for line in reversed(lines) if line.strip()), '子进程未输出错误详情'))
        print(json.dumps(summary, ensure_ascii=False), flush=True)
        if returncode:
            raise SystemExit(returncode)


def run_logged(command, log_path):
    """Preserve the log and show progress/failure details in the terminal."""
    with log_path.open('w') as log:
        process = subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True, start_new_session=True)
        try:
            for line in process.stdout:
                log.write(line)
                log.flush()
                print(line, end='', flush=True)
            return process.wait()
        except BaseException:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=60)
            raise
        finally:
            process.stdout.close()


if __name__ == '__main__':
    main()
