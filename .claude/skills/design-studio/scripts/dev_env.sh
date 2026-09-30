#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Bring the rux GUI up for a screenshot pass, or tear it down.
#
# The frontend is a pure client of `rux gui`, and the Vite dev server MUST proxy
# to it (rux gui / Crow 1.3 cannot answer a CORS preflight, so a bare
# cross-origin call fails — see apps/rux/frontend/README.md). This script starts
# both halves against a fixture project and prints the URL to screenshot.
#
# Usage:
#   dev_env.sh start [project.rux] [gui_port] [vite_port]
#   dev_env.sh stop
#   dev_env.sh status
#
# Defaults: project=tests/fixtures/scans/office_corridor.rux, gui_port=8420,
# vite_port=5173. Run from inside `nix develop` (provides node + the rux binary).
# Override the rux binary with RUX_BIN=... (defaults to `rux` on PATH, then
# ./build/apps/rux/rux).
set -euo pipefail

# Repo root = two dirs above this script's skill dir (.claude/skills/design-studio).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
FRONTEND="$REPO_ROOT/apps/rux/frontend"
RUN_DIR="${TMPDIR:-/tmp}/rux-design-studio"
mkdir -p "$RUN_DIR"

cmd="${1:-start}"

resolve_rux() {
  if [[ -n "${RUX_BIN:-}" ]]; then echo "$RUX_BIN"; return; fi
  if command -v rux >/dev/null 2>&1; then command -v rux; return; fi
  if [[ -x "$REPO_ROOT/build/apps/rux/rux" ]]; then echo "$REPO_ROOT/build/apps/rux/rux"; return; fi
  echo ""; return
}

kill_pidfile() {
  local f="$1"
  [[ -f "$f" ]] || return 0
  local pid; pid="$(cat "$f" 2>/dev/null || true)"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill "$pid" 2>/dev/null || true
    # give it a moment, then hard-kill the group
    sleep 1
    kill -9 "$pid" 2>/dev/null || true
  fi
  rm -f "$f"
}

case "$cmd" in
  start)
    project="${2:-$REPO_ROOT/tests/fixtures/scans/office_corridor.rux}"
    gui_port="${3:-8420}"
    vite_port="${4:-5173}"

    rux_bin="$(resolve_rux)"
    if [[ -z "$rux_bin" ]]; then
      echo "error: no rux binary found. Build it (cmake --build build) or set RUX_BIN=." >&2
      exit 1
    fi
    if [[ ! -f "$project" ]]; then
      echo "error: project not found: $project" >&2
      exit 1
    fi
    if [[ ! -d "$FRONTEND/node_modules" ]]; then
      echo "note: installing frontend deps (first run)…" >&2
      npm --prefix "$FRONTEND" install
    fi

    echo "Starting rux gui  ($rux_bin) on :$gui_port against $(basename "$project")…"
    RUX_GUI_LOG="$RUN_DIR/gui.log"
    nohup "$rux_bin" -p "$project" gui --port "$gui_port" --no-browser \
      >"$RUX_GUI_LOG" 2>&1 &
    echo $! > "$RUN_DIR/gui.pid"

    echo "Starting vite dev on :$vite_port (proxying /api -> :$gui_port)…"
    VITE_LOG="$RUN_DIR/vite.log"
    RUX_GUI_URL="http://localhost:$gui_port" \
      nohup npm --prefix "$FRONTEND" run dev -- --port "$vite_port" --strictPort \
      >"$VITE_LOG" 2>&1 &
    echo $! > "$RUN_DIR/vite.pid"

    # Wait for Vite to answer before handing back to the caller.
    url="http://localhost:$vite_port"
    for _ in $(seq 1 60); do
      if curl -sf -o /dev/null "$url" 2>/dev/null; then
        echo
        echo "  UP:  $url        (screenshot this, never :$gui_port directly)"
        echo "  api: proxied to  http://localhost:$gui_port"
        echo "  logs: $RUX_GUI_LOG , $VITE_LOG"
        echo "  stop: bash $SCRIPT_DIR/dev_env.sh stop"
        exit 0
      fi
      sleep 1
    done
    echo "error: vite did not come up on $url in 60s. Check $VITE_LOG" >&2
    exit 1
    ;;

  stop)
    kill_pidfile "$RUN_DIR/vite.pid"
    kill_pidfile "$RUN_DIR/gui.pid"
    echo "Stopped."
    ;;

  status)
    for name in gui vite; do
      f="$RUN_DIR/$name.pid"
      if [[ -f "$f" ]] && kill -0 "$(cat "$f")" 2>/dev/null; then
        echo "$name: running (pid $(cat "$f"))"
      else
        echo "$name: not running"
      fi
    done
    ;;

  *)
    echo "usage: dev_env.sh {start [project.rux] [gui_port] [vite_port] | stop | status}" >&2
    exit 2
    ;;
esac
