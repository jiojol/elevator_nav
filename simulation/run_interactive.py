#!/usr/bin/env python3
"""Open the local live Go2 simulation and choose navigation goals yourself."""
from datetime import datetime
import os
from pathlib import Path
import sys

root=Path(__file__).resolve().parents[1]
output=root/'simulation/artifacts'/('interactive_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
os.execv(sys.executable,[sys.executable,str(root/'simulation/run_case.py'),
    '--interactive','--case','large_obstacle','--output',str(output),*sys.argv[1:]])
