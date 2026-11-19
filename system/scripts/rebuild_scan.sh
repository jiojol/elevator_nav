#!/bin/bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.sh"
dog_nav_source_nav
exec python3 "${SCRIPT_DIR}/verify_scan_build.py" --rebuild
