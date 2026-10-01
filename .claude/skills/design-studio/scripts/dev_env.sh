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
# both halves against a project and prints the URL to screenshot.
#
# The project is never served in place. `start` copies it into the run
# directory and serves the copy, because `rux gui` migrates the schema and
# leaves -wal/-shm files beside whatever it opens — a git-tracked fixture
# included. Every `start` serves a fresh copy, so a flow that mutates the
# project can simply be re-run.
#
# Run state (pidfiles, logs, the served copy) lives in
# <repo>/.superpowers/dev-env/: gitignored, and the same path from every shell.
# It used to live under $TMPDIR, which `nix develop` points somewhere new each
# time, so `stop` from another shell missed the servers.
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

# Repo root = four dirs above this script (scripts/ → design-studio/ → skills/ → .claude/ → root).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
FRONTEND="$REPO_ROOT/apps/rux/frontend"
RUN_DIR="$REPO_ROOT/.superpowers/dev-env"
mkdir -p "$RUN_DIR"

cmd="${1:-start}"

resolve_rux() {
  if [[ -n "${RUX_BIN:-}" ]]; then echo "$RUX_BIN"; return; fi
  if command -v rux >/dev/null 2>&1; then command -v rux; return; fi
  if [[ -x "$REPO_ROOT/build/apps/rux/rux" ]]; then echo "$REPO_ROOT/build/apps/rux/rux"; return; fi
  echo ""; return
}

# Who each pidfile's process must be: its executable name (`ps -o comm=`) and
# a substring of its command line (`ps -o args=`). `rux gui` is the rux binary
# serving the copy under $RUN_DIR/project/; the dev server is node running
# vite with the flags `start` gives it. Both are checked: a command line alone
# is not enough, since any shell whose script merely mentions "vite" would
# match it. A live pid that fails the check is a stale pidfile whose pid the
# system has reused — it is never signalled.
expected_comm() {
  case "$1" in
    gui) echo "rux" ;;
    vite) echo "node" ;;
  esac
}
expected_args() {
  case "$1" in
    gui) echo " -p $RUN_DIR/project/" ;;
    vite) echo "node_modules/.bin/vite --port " ;;
  esac
}

# Succeeds when <name>.pid holds a live process that is ours. A stale pidfile
# (a live pid that is someone else) is removed with a warning; one whose
# process has exited is left for the caller to clear.
alive() {
  local name="$1" f="$RUN_DIR/$1.pid"
  [[ -f "$f" ]] || return 1
  local pid; pid="$(cat "$f" 2>/dev/null || true)"
  [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null || return 1
  local comm args
  comm="$(ps -o comm= -p "$pid" 2>/dev/null || true)"
  args="$(ps -ww -o args= -p "$pid" 2>/dev/null || true)"
  local want_comm want_args
  want_comm="$(expected_comm "$name")"
  want_args="$(expected_args "$name")"
  if [[ "$comm" != "$want_comm" || "$args" != *"$want_args"* ]]; then
    echo "warning: stale $name.pid: pid $pid is '${comm:-?}', not $name ($want_comm …$want_args…); removed the pidfile, killed nothing" >&2
    rm -f "$f"
    return 1
  fi
  return 0
}

# Each server runs in its own session (setsid), so its pid is also its process
# group id: signalling the group stops the server *and* anything it spawned.
# Only a pid that `alive` vouches for is ever signalled.
kill_pidfile() {
  local name="$1" f="$RUN_DIR/$1.pid"
  [[ -f "$f" ]] || return 0
  if alive "$name"; then
    local pid; pid="$(cat "$f")"
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.25
    done
    if kill -0 "$pid" 2>/dev/null; then
      kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
    fi
  fi
  rm -f "$f"
}

case "$cmd" in
  start)
    command -v setsid >/dev/null || { echo "dev_env: setsid not found (util-linux)"; exit 1; }
    project="${2:-$REPO_ROOT/tests/fixtures/scans/office_corridor.rux}"
    gui_port="${3:-8420}"
    vite_port="${4:-5173}"

    gui_up=0; vite_up=0
    alive gui && gui_up=1
    alive vite && vite_up=1
    if (( gui_up || vite_up )); then
      echo "error: already running (bash $SCRIPT_DIR/dev_env.sh status); stop it first" >&2
      exit 1
    fi
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

    # Serve a throwaway copy, never the original (see the header).
    rm -rf "$RUN_DIR/project"
    mkdir -p "$RUN_DIR/project"
    served="$RUN_DIR/project/$(basename "$project")"
    cp "$project" "$served"
    if [[ -f "$project-wal" ]]; then cp "$project-wal" "$served-wal"; fi

    echo "Starting rux gui  ($rux_bin) on :$gui_port against a copy of $(basename "$project")…"
    RUX_GUI_LOG="$RUN_DIR/gui.log"
    setsid nohup "$rux_bin" -p "$served" gui --port "$gui_port" --no-browser \
      >"$RUX_GUI_LOG" 2>&1 &
    echo $! > "$RUN_DIR/gui.pid"

    echo "Starting vite dev on :$vite_port (proxying /api -> :$gui_port)…"
    VITE_LOG="$RUN_DIR/vite.log"
    # vite itself, not `npm run dev` (which is just `vite`): the recorded pid's
    # command line then names vite, which is what `alive` checks.
    (cd "$FRONTEND" && RUX_GUI_URL="http://localhost:$gui_port" \
      exec setsid nohup ./node_modules/.bin/vite --port "$vite_port" --strictPort) \
      >"$VITE_LOG" 2>&1 &
    echo $! > "$RUN_DIR/vite.pid"

    # Wait for Vite to answer before handing back to the caller.
    url="http://localhost:$vite_port"
    for _ in $(seq 1 60); do
      if curl -sf -o /dev/null "$url" 2>/dev/null; then
        echo
        echo "  UP:      $url        (screenshot this, never :$gui_port directly)"
        echo "  api:     proxied to  http://localhost:$gui_port"
        echo "  project: $served   (a copy; refreshed by every start)"
        echo "  logs:    $RUX_GUI_LOG , $VITE_LOG"
        echo "  stop:    bash $SCRIPT_DIR/dev_env.sh stop"
        exit 0
      fi
      sleep 1
    done
    echo "error: vite did not come up on $url in 60s. Check $VITE_LOG" >&2
    exit 1
    ;;

  stop)
    kill_pidfile vite
    kill_pidfile gui
    rm -rf "$RUN_DIR/project"
    echo "Stopped."
    ;;

  status)
    for name in gui vite; do
      f="$RUN_DIR/$name.pid"
      if alive "$name"; then
        echo "$name: running (pid $(cat "$f"))"
      else
        echo "$name: not running"
      fi
    done
    if [[ -d "$RUN_DIR/project" ]]; then
      echo "project: $(ls "$RUN_DIR/project"/*.rux 2>/dev/null | head -1)"
    fi
    ;;

  *)
    echo "usage: dev_env.sh {start [project.rux] [gui_port] [vite_port] | stop | status}" >&2
    exit 2
    ;;
esac
