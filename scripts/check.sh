#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Fast incremental verify loop: configure (once) + build + ctest.
# This is the check every change must pass before commit (see
# docs/STANDARDS.md §9 "Definition of Done").
#
# Usage:
#   scripts/check.sh                 # build + run all tests
#   scripts/check.sh -R <regex>      # build + run matching tests only
#   CHECK_BUILD_DIR=build-ci scripts/check.sh   # use a different build dir
#
# Must be run inside the dev shell (`nix develop`). Extra arguments are
# forwarded to ctest.

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${CHECK_BUILD_DIR:-$repo_root/build}"

if ! command -v cmake >/dev/null; then
  echo "error: cmake not found — run inside 'nix develop'" >&2
  exit 1
fi

# Configure only when needed (missing or stale cache).
if [[ ! -f "$build_dir/CMakeCache.txt" ]]; then
  echo "==> configuring ($build_dir)"
  cmake -B "$build_dir" -S "$repo_root" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTS=ON
fi

echo "==> building"
cmake --build "$build_dir" --parallel

echo "==> testing"
ctest --test-dir "$build_dir" --output-on-failure \
  --parallel "$(nproc)" "$@"

echo "==> OK"
