#!/usr/bin/env bash
# Run a mission against the firmware emulator. Usage: scripts/run_sim.sh [mission]
set -euo pipefail
cd "$(dirname "$0")/.."
[ -f .venv/bin/activate ] && source .venv/bin/activate
pluto-run --profile sim --mission "${1:-hover_test}"
