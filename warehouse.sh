#!/usr/bin/env bash
# The whole stack in one command: Isaac Sim warehouse cell (ROS 2 node /warehouse_sim) + WMS node
# + rosbridge websocket + web dashboard. Everything talks ROS 2 Jazzy on ROS_DOMAIN_ID (default 42).
#
#   ./warehouse.sh start [--gui] [extra step3 flags]   e.g. --seed 3, --no-cams
#   ./warehouse.sh stop | restart | status | logs [sim|wms|rosbridge|web]
#
# Dashboard: http://<this host>:${WEB_PORT:-8090}/   rosbridge: ws://<this host>:${WS_PORT:-9090}
set -uo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
logs="$here/logs"; run="$here/.run"
mkdir -p "$logs" "$run"
DOMAIN="${ROS_DOMAIN_ID:-42}"; WEB_PORT="${WEB_PORT:-8090}"; WS_PORT="${WS_PORT:-9090}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

ros_env() {
  # shellcheck disable=SC1090
  set +u; source "$ROS_SETUP"; set -u
  export ROS_DOMAIN_ID="$DOMAIN"
}

alive() { [[ -f "$run/$1.pid" ]] && kill -0 "$(cat "$run/$1.pid")" 2>/dev/null; }

launch() {            # name, command...   (own session so stop can take down the whole tree)
  local name="$1"; shift
  if alive "$name"; then echo "  $name already running (pid $(cat "$run/$name.pid"))"; return; fi
  setsid "$@" > "$logs/$name.log" 2>&1 < /dev/null &
  echo $! > "$run/$name.pid"
  echo "  $name started (pid $!, log logs/$name.log)"
}

start() {
  local gui=0 extra=()
  for a in "$@"; do [[ "$a" == "--gui" ]] && gui=1 || extra+=("$a"); done
  ros_env
  echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  launch rosbridge ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:="$WS_PORT" address:=0.0.0.0
  launch wms python3 "$here/wms_node.py"
  launch web python3 "$here/tools/web_server.py" "$WEB_PORT" "$here/web"
  if (( gui )); then
    export DISPLAY="${DISPLAY:-:0}"
    launch sim "$here/run.sh" --ros --orders --realtime "${extra[@]}"
  else
    launch sim "$here/run.sh" --headless --ros --orders --realtime "${extra[@]}"
  fi
  local ip; ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
  echo "dashboard: http://${ip:-localhost}:$WEB_PORT/   (the cell takes ~3-5 min to load assets and come up)"
}

stop() {
  for name in sim wms web rosbridge; do
    if alive "$name"; then
      local pid; pid="$(cat "$run/$name.pid")"
      kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null
      for _ in $(seq 1 30); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
      kill -KILL -- "-$pid" 2>/dev/null
      echo "  $name stopped"
    fi
    rm -f "$run/$name.pid"
  done
}

status() {
  for name in sim wms rosbridge web; do
    if alive "$name"; then echo "  $name: running (pid $(cat "$run/$name.pid"))"; else echo "  $name: stopped"; fi
  done
  if alive sim; then
    grep -E "^\[status|^\[ros\]|Traceback" "$logs/sim.log" | tail -3 | sed 's/^/    /'
  fi
}

case "${1:-}" in
  start) shift; start "$@" ;;
  stop) stop ;;
  restart) shift; stop; start "$@" ;;
  status) status ;;
  logs) tail -n 60 -f "$logs/${2:-sim}.log" ;;
  *) sed -n 2,9p "$0"; exit 1 ;;
esac
