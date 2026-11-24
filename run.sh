#!/usr/bin/env bash
# Start the warehouse with whatever Isaac Sim is installed (5.1 binary or 6.x pip, x86_64 or aarch64).
#   ./run.sh                                   # GUI
#   ./run.sh --headless --steps 9000           # headless, 150 s of sim time
# Set ISAAC_PY to pick a specific interpreter (an Isaac Sim python.sh or a venv python).
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

py="${ISAAC_PY:-}"
if [[ -z "$py" ]]; then
  for c in "${ISAAC_SIM_DIR:-}/python.sh" "$HOME/isaacsim/python.sh" "$HOME/isaac-sim/python.sh" \
           "$HOME"/*/.venv-isaacsim/bin/python; do
    [[ -x "$c" ]] && { py="$c"; break; }
  done
fi
if [[ -z "$py" ]] && python3 -c "import isaacsim" 2>/dev/null; then py="$(command -v python3)"; fi
[[ -n "$py" ]] || { echo "No Isaac Sim python found: set ISAAC_PY=/path/to/python.sh (or a venv python)" >&2; exit 1; }

export OMNI_KIT_ACCEPT_EULA=YES
# pip Isaac Sim on aarch64 (DGX Spark / GB10) needs libgomp preloaded
if [[ "$(uname -m)" == "aarch64" && -e /lib/aarch64-linux-gnu/libgomp.so.1 ]]; then
  export LD_PRELOAD="/lib/aarch64-linux-gnu/libgomp.so.1${LD_PRELOAD:+:$LD_PRELOAD}"
fi
echo "[run] $py"
exec "$py" "$here/step3_autonomy.py" "$@"
