#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMMAND="cd '$REPO_ROOT' && export VULTURE_X_SITL_INTERACTIVE=1 && exec '$REPO_ROOT/scripts/run_sitl.sh'"

if command -v x-terminal-emulator >/dev/null 2>&1; then
  exec x-terminal-emulator -e bash -lc "$COMMAND"
elif command -v gnome-terminal >/dev/null 2>&1; then
  exec gnome-terminal -- bash -lc "$COMMAND"
elif command -v konsole >/dev/null 2>&1; then
  exec konsole -e bash -lc "$COMMAND"
elif command -v xfce4-terminal >/dev/null 2>&1; then
  exec xfce4-terminal --command "bash -lc \"$COMMAND\""
else
  echo "No supported terminal emulator found." >&2
  echo "Run this manually in another terminal:" >&2
  echo "  VULTURE_X_SITL_INTERACTIVE=1 scripts/run_sitl.sh" >&2
  exit 2
fi
