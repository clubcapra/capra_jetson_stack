#!/usr/bin/env bash
# Launch the IK engine and the Rust control interface together.
#
# The engine bootstraps its own venv on first run (see run.py inside the
# engine bundle). The Rust crate is built --release by default; pass
# --debug to skip that. The engine's debug GUI is on by default — pass
# --no-gui to start it headless.
#
# Usage:
#   ./run.sh                 # release wrapper + engine + GUI at :9504
#   ./run.sh --debug         # debug cargo build
#   ./run.sh --no-gui        # don't start the engine's HTTP GUI
#   CARGO_FLAGS="..."  ./run.sh
#
# Ctrl-C cleanly stops both processes.

set -Eeuo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Engine moved into a project-named subdir after the latest re-export.
# Pick whichever layout is present so older checkouts still work.
if [[ -d "$HERE/rove_mvp_engine/rove_cursed_aaaah_goofy_design_ik_engine" ]]; then
  ENGINE_DIR="$HERE/rove_mvp_engine/rove_cursed_aaaah_goofy_design_ik_engine"
else
  ENGINE_DIR="$HERE/rove_mvp_engine"
fi
WRAPPER_DIR="$HERE/rove_control_interface"

# Resolve the cargo build profile. Default --release; --debug opts out.
WRAPPER_PROFILE="--release"
EXTRA_FLAGS=""
ENGINE_GUI_FLAG="--gui"
ENGINE_GUI_PORT="9504"
for arg in "$@"; do
  case "$arg" in
    --debug) WRAPPER_PROFILE="" ;;
    --release) WRAPPER_PROFILE="--release" ;;
    --no-gui) ENGINE_GUI_FLAG="" ;;
    --gui) ENGINE_GUI_FLAG="--gui" ;;
    *) EXTRA_FLAGS="$EXTRA_FLAGS $arg" ;;
  esac
done
CARGO_FLAGS="${CARGO_FLAGS:-$WRAPPER_PROFILE}"

ENGINE_PID=""
WRAPPER_PID=""

stop_pid() {
  # Send SIGINT, then SIGTERM, then SIGKILL with short grace windows so a
  # child that doesn't honour Ctrl-C (e.g. asyncio holding udp sockets in
  # a finally: block) still goes away.
  local pid="$1"
  [[ -n "$pid" ]] || return 0
  kill -0 "$pid" 2>/dev/null || return 0

  for sig in INT TERM KILL; do
    kill -"$sig" "$pid" 2>/dev/null || true
    for _ in $(seq 1 15); do
      kill -0 "$pid" 2>/dev/null || return 0
      sleep 0.1
    done
  done
}

cleanup() {
  local code=$?
  echo
  echo "[run.sh] stopping…"
  stop_pid "$WRAPPER_PID"
  stop_pid "$ENGINE_PID"
  exit $code
}
trap cleanup INT TERM EXIT

# Refuse to start if our ports are already held by something else; we'd
# otherwise see opaque "Address already in use" errors deep inside python.
preflight_ports() {
  local udp_conflicts=()
  for port in 9501 9502 9503 7000 7001; do
    if ss -lnup "sport = :$port" 2>/dev/null | grep -q ":$port "; then
      udp_conflicts+=("$port")
    fi
  done
  local tcp_conflicts=()
  if [[ -n "$ENGINE_GUI_FLAG" ]]; then
    if ss -lntp "sport = :$ENGINE_GUI_PORT" 2>/dev/null | grep -q ":$ENGINE_GUI_PORT "; then
      tcp_conflicts+=("$ENGINE_GUI_PORT (TCP, engine GUI)")
    fi
  fi
  if (( ${#udp_conflicts[@]} > 0 )) || (( ${#tcp_conflicts[@]} > 0 )); then
    echo "[run.sh] ports already in use: UDP[${udp_conflicts[*]}] TCP[${tcp_conflicts[*]}]"
    echo "[run.sh] previous engine/wrapper still running? try:"
    echo "         pkill -f 'engine.server|rove_control_interface|run.py'"
    exit 1
  fi
}

# Spin until 9501 is bound (preflight already cleared it, so the new
# binding is ours) and the engine process is still alive. Give up at
# ~30 s — first-run bootstrap may need to install numpy.
wait_for_engine_ready() {
  local port=9501
  for _ in $(seq 1 150); do
    if ! kill -0 "$ENGINE_PID" 2>/dev/null; then
      echo "[run.sh] engine process exited during startup"
      return 1
    fi
    if ss -lnup "sport = :$port" 2>/dev/null | grep -q ":$port "; then
      return 0
    fi
    sleep 0.2
  done
  return 1
}

preflight_ports

if [[ -n "$ENGINE_GUI_FLAG" ]]; then
  echo "[run.sh] starting IK engine (GUI at http://127.0.0.1:$ENGINE_GUI_PORT)"
else
  echo "[run.sh] starting IK engine (headless)"
fi
(
  cd "$ENGINE_DIR"
  # shellcheck disable=SC2086
  exec python3 run.py $ENGINE_GUI_FLAG
) &
ENGINE_PID=$!

if ! wait_for_engine_ready; then
  echo "[run.sh] engine never bound UDP 9501 — see log above"
  exit 1
fi
echo "[run.sh] engine ready (pid $ENGINE_PID)"

echo "[run.sh] starting rove_control_interface ($CARGO_FLAGS$EXTRA_FLAGS)"
(
  cd "$WRAPPER_DIR"
  # shellcheck disable=SC2086
  exec cargo run $CARGO_FLAGS $EXTRA_FLAGS -- --config config.toml
) &
WRAPPER_PID=$!

# Wait on whichever child exits first; cleanup() will tear down the other.
wait -n "$ENGINE_PID" "$WRAPPER_PID"
