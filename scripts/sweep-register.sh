#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Parameter sweep for `rux register` (JPR pose refinement) scored by
# `rux analyze quality` (issue #221).
#
# Usage: scripts/sweep-register.sh <source-project.rux> <output-dir>
#
# For each configuration: copy the source project, refine poses, regenerate
# clouds+planes with default parameters, compute quality metrics, then delete
# the (large) project copy — only the JSON report and log are kept.
# Configurations are defined at the bottom; edit there to change the sweep.

set -euo pipefail

src="$1"
out="$2"
rux="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/build/apps/rux/rux"

mkdir -p "$out"

run() {
  local name="$1"
  shift
  local proj="$out/sweep-$name.rux"
  local log="$out/sweep-$name.log"
  echo "==> $name: $*"
  cp -f "$src" "$proj"
  {
    "$rux" -v -p "$proj" register "$@"
    "$rux" -v -p "$proj" create clouds
    "$rux" -v -p "$proj" create planes
    "$rux" -v -p "$proj" analyze quality -o "$out/sweep-$name.json"
  } >> "$log" 2>&1
  rm -f "$proj" "$proj-shm" "$proj-wal"
  grep "Quality:" "$log" | tail -1
}

# ── Sweep configurations ─────────────────────────────────────────────────
# Reference results on scan afb3234950 (see issue #221):
#   no refinement:               flatness_rms 21.6mm  p90 33.8mm
#   temporal-only defaults:      flatness_rms 18.0mm  p90 28.7mm
#   best knob combo (phase 1):   flatness_rms 17.7mm  p90 27.4mm
# Phase 2: scene-centroid spatial pairing (feat/jpr-spatial-pairing).
run sp0        --spatial-radius 0
run spdef      # spatial defaults: radius 1.5, max 8 pairs/frame
run sp25       --spatial-radius 2.5
run sp15combo  --prior-weight 0.1 --neighbor-window 10 --iterations 50
run sp25loose  --spatial-radius 2.5 --prior-weight 0.1 --neighbor-window 10 --iterations 50

echo "SWEEP-DONE"
