#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Build the heavy ReUseX flake dependencies one at a time, each guarded by a
# memory watchdog that aborts the running build before the host runs out of RAM.
#
# Why one at a time: a couple of packages (opencv+cuda, libtorch, cuOpt,
# trtsam3) dominate build time and peak RAM. Building them serially — instead of
# letting `max-jobs = auto` fire dozens of heavy compiles at once — keeps memory
# bounded while still letting each build use as many cores as you allow.
#
# Aborting is cheap here: overlays/ccache.nix routes these compiles through the
# shared /var/cache/ccache, so object files produced before a kill survive and a
# re-run resumes from them. The daemon aborts the build when the `nix build`
# client is interrupted, which frees the nixbld compiler processes.
#
# Usage:
#   scripts/build-deps.sh                 # build the default heavy set, in order
#   scripts/build-deps.sh opencv gtsam    # build only these attrs
#   CORES=8 scripts/build-deps.sh libtorch        # cap threads for one run
#   MAX_JOBS=2 scripts/build-deps.sh              # allow 2 concurrent derivations
#   MIN_FREE_MIB=12000 scripts/build-deps.sh      # abort earlier (more headroom)
#
# Tuning (all optional env knobs):
#   The peak number of concurrent compilers is roughly MAX_JOBS * CORES. On this
#   host (48 cores, 125 GB) the template-heavy TUs (CGAL/PCL/Eigen/GTSAM) can use
#   2-3 GB each, so ~32 concurrent keeps memory well under budget while still
#   filling the machine (serial configure/link phases of one job overlap the
#   compile phases of another). max-jobs=auto x cores=0 is what caused the OOM.
#
#   CORES         nix --cores per derivation      (default 8;  0 = all cores)
#   MAX_JOBS      nix --max-jobs (concurrent drvs)(default 4)
#   MIN_FREE_MIB  abort a build below this MemAvailable, in MiB (default 8192)
#   POLL          watchdog poll interval, seconds  (default 3)
#
# Examples:
#   scripts/build-deps.sh                          # bounded-parallel, memory-safe
#   MAX_JOBS=8 CORES=6 scripts/build-deps.sh       # push harder (~48 compilers)
#   MAX_JOBS=1 CORES=0 scripts/build-deps.sh libtorch opencv   # one giant at a time
set -uo pipefail

CORES=${CORES:-8}
MAX_JOBS=${MAX_JOBS:-4}
MIN_FREE_MIB=${MIN_FREE_MIB:-8192}
# Abort a build if free space on the nix store drops below this (GiB). Heavy CUDA
# builds (opencv, libtorch) churn tens of GB of transient build dirs; hitting a
# full store wedges the whole system, so we stop well before that.
MIN_FREE_DISK_GIB=${MIN_FREE_DISK_GIB:-15}
POLL=${POLL:-3}

# With no args, build the whole CUDA ReUseX closure in ONE nix invocation: nix
# schedules the entire dependency graph with --max-jobs concurrency, which uses
# the machine far better than building attr-by-attr (which would block the next
# package until the current one fully finishes). Completed derivations persist if
# the build is interrupted, and ccache keeps objects from in-flight ones.
#
# Pass explicit attrs to isolate them instead — useful to give a giant its own
# resource limits, e.g.:  MAX_JOBS=1 CORES=0 scripts/build-deps.sh opencv default
DEFAULT_TARGETS=(default)

targets=("$@")
if [ "${#targets[@]}" -eq 0 ]; then
  targets=("${DEFAULT_TARGETS[@]}")
fi

ccache_stats() {
  command -v nix-ccache >/dev/null 2>&1 || return 0
  nix-ccache -s 2>/dev/null | grep -E "Hits:|Misses:|Cache size" | head -3 | tr '\n' ' '
}

mem_avail_mib() {
  awk '/^MemAvailable:/ {print int($2 / 1024)}' /proc/meminfo
}

disk_avail_gib() {
  df -BG --output=avail /nix 2>/dev/null | tail -1 | tr -dc '0-9'
}

# Abort the nix build client $1 (SIGINT so the daemon tears down the build and
# frees its resources), escalating to TERM/KILL.
abort_build() {
  local pid="$1" why="$2"
  echo "" >&2
  echo "[watchdog] $why — aborting build (pid $pid)" >&2
  kill -INT "$pid" 2>/dev/null
  sleep 5
  kill -TERM "$pid" 2>/dev/null
  sleep 3
  kill -KILL "$pid" 2>/dev/null
}

# Watch RAM and disk while $1 (the nix build client) runs; abort if either drops
# below its floor so the daemon releases resources before the box wedges. Also
# prints a periodic utilization line so you can see it's busy.
watch_mem() {
  local pid="$1"
  local ticks=0
  while kill -0 "$pid" 2>/dev/null; do
    local avail disk
    avail=$(mem_avail_mib)
    disk=$(disk_avail_gib)
    if [ "${avail:-0}" -lt "$MIN_FREE_MIB" ]; then
      abort_build "$pid" "MemAvailable ${avail}MiB < ${MIN_FREE_MIB}MiB"
      return 1
    fi
    if [ "${disk:-999}" -lt "$MIN_FREE_DISK_GIB" ]; then
      abort_build "$pid" "disk ${disk}GiB < ${MIN_FREE_DISK_GIB}GiB on /nix"
      return 1
    fi
    # Every ~10 polls, report load / running compilers / free memory / free disk.
    ticks=$((ticks + 1))
    if [ $((ticks % 10)) -eq 1 ]; then
      local ncc load
      # pgrep -c prints 0 AND exits 1 when there are no matches; the `|| true`
      # keeps that single "0" instead of appending a second one.
      ncc=$(pgrep -c "cc1plus|cc1|nvcc" 2>/dev/null || true)
      load=$(awk '{print $1}' /proc/loadavg)
      echo "    [status] load=${load} compilers=${ncc} memAvail=${avail}MiB diskAvail=${disk}GiB" >&2
    fi
    sleep "$POLL"
  done
  return 0
}

echo "build-deps: cores=$CORES max-jobs=$MAX_JOBS watchdog=ram:${MIN_FREE_MIB}MiB,disk:${MIN_FREE_DISK_GIB}GiB poll=${POLL}s"
echo "targets: ${targets[*]}"

failed=()
for t in "${targets[@]}"; do
  echo "==================================================================="
  echo ">>> .#$t   ($(date '+%H:%M:%S'))"
  echo "    ccache: $(ccache_stats)"

  nix build ".#$t" --cores "$CORES" --max-jobs "$MAX_JOBS" \
    --print-build-logs --no-link &
  build_pid=$!

  watch_mem "$build_pid" &
  watch_pid=$!

  wait "$build_pid"
  status=$?

  kill "$watch_pid" 2>/dev/null
  wait "$watch_pid" 2>/dev/null

  if [ "$status" -eq 0 ]; then
    echo "    [ok] .#$t   ccache: $(ccache_stats)"
  else
    echo "    [FAIL/aborted] .#$t (exit $status)" >&2
    failed+=("$t")
    # Stop so a memory-abort doesn't cascade into the next heavy build. Re-run
    # this script to resume — cached objects and finished derivations are kept.
    echo "Stopping. Re-run to resume: scripts/build-deps.sh ${t} ${targets[*]##${t}}" >&2
    break
  fi
done

if [ "${#failed[@]}" -ne 0 ]; then
  echo "FAILED: ${failed[*]}" >&2
  exit 1
fi
echo "All targets built."
