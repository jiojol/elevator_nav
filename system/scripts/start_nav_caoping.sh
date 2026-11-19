#!/bin/bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export DOG_NAV_HOME="${DOG_NAV_HOME:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
export DOG_NAV_BUILDING_PCD="${DOG_NAV_HOME}/system/map/caoping/go2.pcd"
export DOG_NAV_PLAN_META="${DOG_NAV_HOME}/system/map/caoping/caoping_plan.json"
exec bash "${SCRIPT_DIR}/start_nav.sh" "$@"
