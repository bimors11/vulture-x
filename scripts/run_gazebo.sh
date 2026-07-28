#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}"
WORLD="${1:-$REPO_ROOT/simulation/worlds/vulture_x_test.sdf}"

if ! command -v gz >/dev/null 2>&1; then
  echo "Gazebo Sim command 'gz' was not found. Run scripts/setup_environment.sh first." >&2
  exit 2
fi

if ! gz sim -h >/dev/null 2>&1; then
  echo "'gz sim' is not available. Gazebo Classic alone is not enough for the current ArduPilot plugin." >&2
  exit 2
fi

if [[ ! -d "$ARDUPILOT_GAZEBO_DIR/build" ]]; then
  echo "ArduPilot Gazebo plugin build not found at $ARDUPILOT_GAZEBO_DIR/build." >&2
  echo "Run scripts/setup_environment.sh first." >&2
  exit 2
fi

export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_DIR/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$REPO_ROOT/simulation/worlds:$REPO_ROOT/simulation/models:$ARDUPILOT_GAZEBO_DIR/models:$ARDUPILOT_GAZEBO_DIR/worlds:${GZ_SIM_RESOURCE_PATH:-}"

exec gz sim -v4 -r "$WORLD"
