#!/usr/bin/env bash
set -euo pipefail

echo "The first milestone has no real SITL MAVLink client." >&2
echo "run_demo.sh will be enabled only after guarded SITL integration passes." >&2
exit 2

