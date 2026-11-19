#!/bin/bash
# Apply the compiled bridge only when the operator explicitly runs this script.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
STAGE_DIR="${PROJECT_DIR}/.runtime/robot_bridge_update"
python3 - "$STAGE_DIR" <<'PY'
from pathlib import Path
import hashlib,json,sys
stage=Path(sys.argv[1]);m=json.loads((stage/'manifest.json').read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
if str(Path('/home/nvidia/3dNav').resolve()) != m['release']:
    raise SystemExit('底盘发布目录已改变，请重新编译更新包。')
if sha(m['candidate']) != m['binary_sha256']:
    raise SystemExit('待启用程序与编译记录不一致。')
for path,expected in m['sources'].items():
    if sha(path) != expected:raise SystemExit('源码已改变，请先重新编译：'+path)
for process in Path('/proc').glob('[0-9]*/cmdline'):
    try:args=process.read_bytes().split(b'\0')
    except OSError:continue
    if any(Path(arg.decode(errors='replace')).name in ('scan_planner_node','closed_loop_controller','scan_command_gate.py','fastlio_map_localizer') for arg in args if arg):
        raise SystemExit('请先在原终端关闭导航，再启用底盘桥更新。')
print('已确认更新包来源；保留现有底盘限速。')
PY
sudo -v
python3 - "$STAGE_DIR" <<'PY'
from pathlib import Path
from datetime import datetime,timezone,timedelta
import json,os,shutil,sys
stage=Path(sys.argv[1]);m=json.loads((stage/'manifest.json').read_text());target=Path(m['target'])
backup=stage/('backup_'+datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S'));backup.mkdir()
shutil.copy2(target,backup/target.name)
shutil.copy2(stage/'manifest.json',backup/'manifest.json')
temporary=target.with_name(target.name+'.next')
shutil.copy2(m['candidate'],temporary);temporary.chmod(0o755);os.replace(temporary,target)
print('旧程序备份：'+str(backup))
PY
sudo systemctl restart modern-visual-state.service
printf '%s\n' '底盘桥程序已更新并重启。现在可按原命令启动导航，由你进行现场测试。'
