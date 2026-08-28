#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Run the benchmark suite and store a timestamped XML report under
# bench-results/. Compare two reports to judge a change (STANDARDS.md §8):
#
#   scripts/bench.sh                       # run all benchmarks
#   scripts/bench.sh "[geometry]"          # run a tag subset
#   BENCH_SAMPLES=10 scripts/bench.sh      # more samples (default 5)
#
# Workflow for a change:
#   git stash / checkout main  -> scripts/bench.sh   (baseline)
#   apply change               -> scripts/bench.sh   (candidate)
#   compare the two newest files in bench-results/
#
# Must be run inside the dev shell (`nix develop`).

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${CHECK_BUILD_DIR:-$repo_root/build}"
out_dir="$repo_root/bench-results"
samples="${BENCH_SAMPLES:-5}"
filter="${1:-}"

mkdir -p "$out_dir"

echo "==> building benchmarks"
cmake --build "$build_dir" --parallel --target reusex_benchmarks

stamp="$(date +%Y%m%d-%H%M%S)-$(git -C "$repo_root" rev-parse --short HEAD)"
out_file="$out_dir/bench-$stamp.xml"

echo "==> running benchmarks (samples=$samples)"
"$build_dir/tests/reusex_benchmarks" ${filter:+"$filter"} \
  --benchmark-samples "$samples" \
  --reporter console::out=- \
  --reporter XML::out="$out_file"

echo "==> report: $out_file"
