#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Run screenshot.py with a working Playwright, whatever the host has.
#
# Uses the ambient python if it can import playwright; otherwise borrows
# nixpkgs' python3.withPackages([playwright]) plus the matching
# playwright-driver.browsers bundle (built once, then cached in the store), so
# screenshots work on a bare NixOS box with no pip install.
#
# Usage: shot.sh <screenshot.py args…>
#   shot.sh http://localhost:5173/ --out shots/x --theme light --viewports desktop
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="$SCRIPT_DIR/screenshot.py"

if python3 -c 'import playwright' >/dev/null 2>&1; then
  exec python3 "$PY" "$@"
fi

BROWSERS="$(nix build --no-link --print-out-paths nixpkgs#playwright-driver.browsers)"
export PLAYWRIGHT_BROWSERS_PATH="$BROWSERS"
exec nix shell --impure \
  --expr 'let p = import (builtins.getFlake "nixpkgs") {}; in p.python3.withPackages (ps: [ ps.playwright ])' \
  --command python3 "$PY" "$@"
