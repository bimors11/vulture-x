#!/usr/bin/env bash
set -euo pipefail

CONFIG_PATH="${VULTURE_X_CONFIG:-configs/sitl.yaml}"
exec python -m vulture_x.main --config "$CONFIG_PATH" "$@"

