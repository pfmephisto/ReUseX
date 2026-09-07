#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later
#
# One-command code coverage report (#206): configure + build a dedicated
# Debug+ENABLE_COVERAGE tree, run the unit tests, and render an HTML report
# plus a per-module summary table (grouped by libs/reusex/src/<module>).
#
#   nix develop --command scripts/coverage.sh
#
# Output:
#   coverage/html/index.html   - browsable HTML report (gcovr --html-details)
#   coverage/gcovr.json        - full gcovr JSON report (machine-readable)
#   stdout                     - per-module summary table
#
# Uses its own build-coverage/ tree, never the shared build/ (main build/ is
# a shared benchmark resource - see MEMORY.md - and coverage instrumentation
# changes codegen, so it must not be mixed with normal Release/Debug builds).
#
# Env overrides:
#   COVERAGE_BUILD_DIR   build tree location (default: build-coverage/)
#   COVERAGE_JOBS        parallel *build* jobs (default: min(nproc, 12))
#   COVERAGE_CTEST_JOBS  parallel *ctest* jobs (default: 1, i.e. serial)
#   COVERAGE_CMAKE_ARGS  extra args appended to the cmake configure step
#
# ctest defaults to serial (COVERAGE_CTEST_JOBS=1) rather than -j$(nproc):
# the suite has known flaky ProjectDB tests under high parallelism (#262).
# Serial is fine for coverage purposes - line/branch counts don't depend on
# wall-clock time - so this trades a slower run for a reproducible report.
#
# CI hookup (issue #206's third task, "report coverage in CI") is deferred
# until hosted CI is enabled - see .github/workflows/ci.yml's header comment
# for why hosted CI is currently manual-trigger-only. Once that lands, this
# script is the thing to invoke from a workflow step; it already fails (via
# set -e) on configure/build/ctest errors.
#
# Must be run inside the dev shell (`nix develop`): needs cmake, ninja (or
# make), a GCC/Clang toolchain, and gcovr (all provided by shell.nix).

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${COVERAGE_BUILD_DIR:-$repo_root/build-coverage}"
report_dir="$repo_root/coverage"
html_dir="$report_dir/html"
json_report="$report_dir/gcovr.json"

nproc_val="$(nproc 2>/dev/null || echo 4)"
build_jobs="${COVERAGE_JOBS:-$((nproc_val < 12 ? nproc_val : 12))}"
ctest_jobs="${COVERAGE_CTEST_JOBS:-1}"

for tool in cmake gcovr; do
  if ! command -v "$tool" >/dev/null; then
    echo "error: $tool not found — run inside 'nix develop'" >&2
    exit 1
  fi
done

echo "==> configuring coverage build ($build_dir)"
# shellcheck disable=SC2086
cmake -B "$build_dir" -S "$repo_root" \
  -DCMAKE_BUILD_TYPE=Debug \
  -DENABLE_COVERAGE=ON \
  -DBUILD_TESTS=ON \
  ${COVERAGE_CMAKE_ARGS:-}

echo "==> building (jobs=$build_jobs)"
cmake --build "$build_dir" --parallel "$build_jobs"

echo "==> running tests serially (ctest jobs=$ctest_jobs) — see #262 re: flaky ProjectDB tests under parallel ctest"
# Coverage counters accumulate even if a test fails, so don't let a single
# flaky test abort report generation; still surface the failure at the end.
ctest_status=0
ctest --test-dir "$build_dir" --output-on-failure --parallel "$ctest_jobs" || ctest_status=$?

echo "==> generating coverage report"
mkdir -p "$html_dir"
rm -f "$json_report"

# Defensive cleanup: cmake/Coverage.cmake deliberately does NOT instrument
# CUDA (.cu) sources (see that file for why - mixing gcc-15-compiled .cpp
# objects with nvcc's gcc-14-compiled .cu objects corrupts gcov's runtime
# registration). If an older build ever *did* instrument .cu files (e.g. a
# tree built before that fix), their stale .gcno/.gcda crash this gcovr
# version outright (a .gcno with no matching execution data fails its
# sanity check instead of just reporting 0%). Belt-and-braces on top of the
# --exclude below: there should never be any of these to find.
find "$build_dir" \( -name "*.cu.gcno" -o -name "*.cu.gcda" \) -delete

gcovr \
  --root "$repo_root" \
  --filter 'libs/reusex/' \
  --filter 'apps/rux/' \
  --filter 'apps/ruxd/' \
  --exclude 'extern/' \
  --exclude '/tests/' \
  --exclude 'build[^/]*/' \
  --exclude 'bindings/' \
  --exclude '\.cu$' \
  --exclude-unreachable-branches \
  --exclude-throw-branches \
  --gcov-ignore-parse-errors \
  --html-details "$html_dir/index.html" \
  --json-pretty --json "$json_report" \
  --print-summary \
  -j "$build_jobs" \
  "$build_dir"

echo
echo "==> per-module coverage summary"
python3 "$repo_root/scripts/coverage_report.py" "$json_report"

echo
echo "HTML report: $html_dir/index.html"
echo "JSON report: $json_report"

if [[ $ctest_status -ne 0 ]]; then
  echo
  echo "warning: ctest exited with status $ctest_status (coverage report above is still valid" \
       "for whatever ran; re-run scripts/check.sh to investigate the failing test)." >&2
fi

exit "$ctest_status"
