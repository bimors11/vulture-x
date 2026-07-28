#!/usr/bin/env bash
set -euo pipefail

if [[ "$(uname -s)" != "Linux" ]]; then
  echo "ArduPilot SITL installation is supported by this script only on Linux." >&2
  exit 2
fi

ARDUPILOT_PARENT="${ARDUPILOT_PARENT:-$PWD/..}"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$ARDUPILOT_PARENT/ardupilot}"

if [[ -e "$ARDUPILOT_DIR" ]]; then
  echo "Refusing to overwrite existing path: $ARDUPILOT_DIR" >&2
  exit 2
fi

echo "Review https://ardupilot.org/dev/docs/building-setup-linux.html first."
read -r -p "Clone ArduPilot and run its Ubuntu prerequisite installer? [y/N] " answer
if [[ "$answer" != "y" && "$answer" != "Y" ]]; then
  exit 0
fi

git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git "$ARDUPILOT_DIR"
"$ARDUPILOT_DIR/Tools/environment_install/install-prereqs-ubuntu.sh" -y
echo "ArduPilot checkout prepared at $ARDUPILOT_DIR. Reload your shell profile."

