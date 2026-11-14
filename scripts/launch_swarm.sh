#!/usr/bin/env bash
# REAL DRONES. Usage: scripts/launch_swarm.sh <airlink|final_demo> [mission]
# Ctrl+C once = land all; twice = disarm all.
set -euo pipefail
cd "$(dirname "$0")/.."
[ -f .venv/bin/activate ] && source .venv/bin/activate
PROFILE="${1:?profile required: airlink or final_demo}"
read -r -p "Props clear, area clear, operator ready? Type YES: " ok
[ "$ok" = "YES" ] || { echo "Aborted."; exit 1; }
pluto-run --profile "$PROFILE" --mission "${2:-hover_test}"
