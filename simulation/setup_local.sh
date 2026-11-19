#!/bin/bash
# Prepare this computer only. No SSH or robot deployment.
set -euo pipefail
SIMULATION_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_DIR="$(dirname "$SIMULATION_DIR")"
PHYSICS_ENV="${SIMULATION_DIR}/artifacts/physics_env"
if [ ! -x "${PHYSICS_ENV}/bin/python" ] || [ ! -x "${PHYSICS_ENV}/bin/uv" ]; then
  BOOTSTRAP_UV="${GO2_SIM_UV:-$(command -v uv || true)}"
  if [ -z "$BOOTSTRAP_UV" ] && [ -x /tmp/go2_graph_sim_env/bin/uv ]; then
    BOOTSTRAP_UV=/tmp/go2_graph_sim_env/bin/uv
  fi
  if [ -z "$BOOTSTRAP_UV" ]; then
    echo '请先安装 uv，或设置 GO2_SIM_UV 指向已安装的 uv。' >&2
    exit 1
  fi
  RUNTIME_PYTHON="${GO2_SIM_BASE_PYTHON:-${HOME}/miniconda3/envs/mujoco-lidar/bin/python}"
  if [ ! -x "$RUNTIME_PYTHON" ]; then
    echo '缺少现有 MuJoCo Python 环境；可用 GO2_SIM_BASE_PYTHON 指定。' >&2
    exit 1
  fi
  if [ ! -x "${PHYSICS_ENV}/bin/python" ]; then
    "$BOOTSTRAP_UV" venv --python "$RUNTIME_PYTHON" --system-site-packages "$PHYSICS_ENV"
  fi
  "$BOOTSTRAP_UV" pip install --python "${PHYSICS_ENV}/bin/python" uv pytest
fi
cd "$REPOSITORY_DIR"
/usr/bin/python3 simulation/build_local.py "$@"
