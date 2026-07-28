#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

if [[ -x ".venv/bin/python" ]]; then
  exec ".venv/bin/python" tools/vulture_x_ui.py "$@"
fi

exec python3 tools/vulture_x_ui.py "$@"
