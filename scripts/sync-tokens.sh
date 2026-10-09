#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Vendor the rux-frontend repo's tokens.css into the Qt client
# (apps/rux/qt/theme/tokens.css), then run the Qt token ctests so a design
# change that breaks the Qt client's token/colour expectations is caught
# here, not in a screenshot nobody looked at.
#
# tokens.css is owned by rux-frontend (Claude Design's /design-sync writes
# it there); this repo keeps a vendored copy because the Qt client has no
# npm/node dependency and reads the file at build/run time via CMake
# (apps/rux/qt/CMakeLists.txt: RUX_QT_TOKENS_CSS).
#
# Usage:
#   scripts/sync-tokens.sh <path-to-tokens.css>
#   scripts/sync-tokens.sh <https://... raw tokens.css URL>
#   scripts/sync-tokens.sh ../rux-frontend/src/tokens.css
#
# Options:
#   --build-dir DIR   CMake build dir (default: <repo>/build)
#   --no-test         copy only, skip the ctest run
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel)"
DEST="$ROOT/apps/rux/qt/theme/tokens.css"

BUILD_DIR="$ROOT/build"
RUN_TESTS=1
SRC=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-dir) BUILD_DIR="$2"; shift 2 ;;
    --no-test) RUN_TESTS=0; shift ;;
    -h|--help) sed -n '2,/^set -euo/p' "$0" | sed 's/^# \{0,1\}//; /^set -euo/d'; exit 0 ;;
    -*) echo "sync-tokens.sh: unknown option $1" >&2; exit 2 ;;
    *)
      if [[ -n "$SRC" ]]; then
        echo "sync-tokens.sh: unexpected extra argument $1" >&2
        exit 2
      fi
      SRC="$1"
      shift
      ;;
  esac
done

if [[ -z "$SRC" ]]; then
  echo "sync-tokens.sh: usage: scripts/sync-tokens.sh <path-or-url> [--build-dir DIR] [--no-test]" >&2
  exit 2
fi

TMP=""
cleanup() { [[ -n "$TMP" ]] && rm -f "$TMP"; }
trap cleanup EXIT

if [[ "$SRC" =~ ^https?:// ]]; then
  TMP="$(mktemp "${TMPDIR:-/tmp}/sync-tokens.XXXXXX.css")"
  echo "sync-tokens.sh: fetching $SRC" >&2
  curl -fsSL "$SRC" -o "$TMP"
  SRC_FILE="$TMP"
else
  SRC_FILE="$SRC"
fi

[[ -f "$SRC_FILE" ]] || { echo "sync-tokens.sh: $SRC_FILE does not exist" >&2; exit 1; }
[[ -s "$SRC_FILE" ]] || { echo "sync-tokens.sh: $SRC_FILE is empty" >&2; exit 1; }

if ! grep -q -- '--color-canvas' "$SRC_FILE"; then
  echo "sync-tokens.sh: $SRC_FILE does not look like tokens.css (no --color-canvas)" >&2
  exit 1
fi

cp "$SRC_FILE" "$DEST"
echo "sync-tokens.sh: wrote $DEST ($(wc -l < "$DEST") lines)" >&2

if [[ $RUN_TESTS -eq 0 ]]; then
  exit 0
fi

in_dev() {
  if [[ -n "${IN_NIX_SHELL:-}" ]]; then "$@"; else nix develop "$ROOT" -c "$@"; fi
}

if [[ ! -d "$BUILD_DIR" ]]; then
  echo "sync-tokens.sh: no build dir at $BUILD_DIR; skipping the ctest run (copy only)" >&2
  exit 0
fi

echo "sync-tokens.sh: building reusex_unit_tests (light binary, carries rux_qt_core) …" >&2
in_dev cmake --build "$BUILD_DIR" --target reusex_unit_tests

echo "sync-tokens.sh: running the Qt token ctests …" >&2
# ctest -R matches the registered test NAME (the Catch2 TEST_CASE name, not
# its tags), so filter by the test_tokens.cpp case-name prefixes rather than
# the [rux_qt][tokens] tag.
in_dev ctest --test-dir "$BUILD_DIR" \
  -R 'ParseTokensCss|ResolveVars|NormaliseValue|ParseLengthPx|ParseLengthEm|ParseColor|AppQss_|QtSources_' \
  --output-on-failure
