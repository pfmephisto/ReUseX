#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Loop-closure matcher benchmark (issue #221/#225 P2). Measures the effect of
# externally-computed wide-baseline loop edges (from tools/loop_edges/, produced
# by ORB / XFeat / MASt3R / MapAnything) on pose quality, all fed through the
# license-clean `rux optimize --loop-edges` bridge.
#
# For a given scan it builds a fresh copy per variant, runs the identical
# reconstruction pipeline (deterministic), and scores it with `rux analyze
# quality` (GT-free flatness/thickness) and, if a ground-truth PLY is given,
# `rux analyze accuracy` (F-score @ threshold). Variants:
#   base                 stored poses, no pose stage
#   optimize             plane-graph optimize (current default)
#   optimize+<edges>     optimize --loop-edges <file> --loop-trust  (one per file)
#
# Usage:
#   scripts/bench-loop-edges.sh <project.rux> <out_dir> <edges_dir> [gt.ply]
# where <edges_dir> holds *.json edge files (schema reusex.loop_edges.v1).
# Re-runnable; each variant is isolated in its own .rux copy under <out_dir>.

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
rux="$repo_root/build/apps/rux/rux"

project="${1:?usage: bench-loop-edges.sh <project.rux> <out_dir> <edges_dir> [gt.ply]}"
out_dir="${2:?missing out_dir}"
edges_dir="${3:?missing edges_dir}"
gt="${4:-}"

mkdir -p "$out_dir"
summary="$out_dir/summary.tsv"
printf "variant\tflatness_rms_mm\tthickness_p90_mm\tGT_Fscore\tedges\tmax_shift_m\n" > "$summary"

# extract a scalar from a rux JSON report (key at top level)
jget() { python3 -c "import json,sys;print(json.load(open(sys.argv[1])).get(sys.argv[2],''))" "$1" "$2" 2>/dev/null || echo ""; }

run_variant() {
  local name="$1"; shift
  local edge_file="$1"; shift   # "" for none
  local rux_copy="$out_dir/$name.rux"
  echo "==================== variant: $name ===================="
  rm -f "$rux_copy" "$rux_copy"-shm "$rux_copy"-wal
  cp -f "$project" "$rux_copy"

  local opt_log="$out_dir/$name.optimize.log"
  local edges=0 shift_m=""
  if [[ "$name" != "base" ]]; then
    if [[ -n "$edge_file" ]]; then
      "$rux" -v -p "$rux_copy" optimize --loop-edges "$edge_file" --loop-trust \
        --odometry-sigma-trans 0.05 "$@" > "$opt_log" 2>&1 || true
    else
      "$rux" -v -p "$rux_copy" optimize "$@" > "$opt_log" 2>&1 || true
    fi
    edges=$(grep -oP 'Loop closure: \K[0-9]+' "$opt_log" | tail -1 || echo 0)
    shift_m=$(grep -oP 'max pose shift \K[0-9.]+' "$opt_log" | tail -1 || echo "")
  fi

  "$rux" -p "$rux_copy" create clouds  > "$out_dir/$name.clouds.log" 2>&1
  "$rux" -p "$rux_copy" create planes  > "$out_dir/$name.planes.log" 2>&1
  "$rux" -p "$rux_copy" analyze quality -o "$out_dir/$name.quality.json" \
      > "$out_dir/$name.quality.log" 2>&1

  local fl=$(jget "$out_dir/$name.quality.json" flatness_rms)
  local tp=$(jget "$out_dir/$name.quality.json" thickness_p90)
  # metres -> mm if the report is in metres
  fl=$(python3 -c "v='$fl';print(round(float(v)*1000,2) if v else '')" 2>/dev/null || echo "$fl")
  tp=$(python3 -c "v='$tp';print(round(float(v)*1000,2) if v else '')" 2>/dev/null || echo "$tp")

  local fs=""
  if [[ -n "$gt" ]]; then
    "$rux" -p "$rux_copy" analyze accuracy --gt "$gt" -o "$out_dir/$name.accuracy.json" \
        > "$out_dir/$name.accuracy.log" 2>&1 || true
    fs=$(jget "$out_dir/$name.accuracy.json" fscore)
  fi

  printf "%s\t%s\t%s\t%s\t%s\t%s\n" "$name" "$fl" "$tp" "$fs" "$edges" "$shift_m" >> "$summary"
}

run_variant base ""
run_variant optimize ""
for ef in "$edges_dir"/*.json; do
  [[ -e "$ef" ]] || continue
  bn="$(basename "$ef" .json)"
  run_variant "optimize+$bn" "$ef"
done

echo
echo "==================== SUMMARY ===================="
column -t -s $'\t' "$summary"
echo "(reports + logs in $out_dir)"
