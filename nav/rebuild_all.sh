#!/bin/bash
set -euo pipefail
exec bash "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/system/scripts/build_current.sh" nav "$@"
