#!/usr/bin/env python3
"""Run the calibrated SLAM workspace binary without mixing catkin overlays."""
import os
from pathlib import Path
import sys

import rospkg


def main():
    root = Path(rospkg.RosPack().get_path("dog_nav_bringup")).resolve().parents[2]
    lib = root / "slam/devel/lib"
    executable = lib / "fast_lio/fastlio_mapping"
    if not os.access(str(executable), os.X_OK):
        raise SystemExit("FAST-LIO executable not found: %s" % executable)
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = str(lib) + os.pathsep + env.get("LD_LIBRARY_PATH", "")
    os.execve(str(executable), [str(executable)] + sys.argv[1:], env)


if __name__ == "__main__":
    main()
