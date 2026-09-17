#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Launch the GT Curator Gradio app.
#
# Usage:
#   ./tools/gt_curator/launch.sh -p /path/to/project.rux [--port 7860]
#
# This script sets the necessary environment variables to isolate the
# venv from Nix's site-packages (for typing_extensions, tomlkit, etc.)
# while still pulling in Nix's opencv and numpy.

set -euo pipefail

VENV="${HOME}/gt-curator-venv"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Nix packages that the venv needs to reach (opencv, numpy)
NIX_NUMPY="/nix/store/l59n6vzkswz23y6s4pr6cmv2p4dpd5f0-python3.13-numpy-2.4.4/lib/python3.13/site-packages"
NIX_CV2="/nix/store/kd9y1ckp87vh5c341rnngrqkhjdibxf1-opencv-4.13.0/lib/python3.13/site-packages"
GCC_LIB="/nix/store/xm08aqdd7pxcdhm0ak6aqb1v7hw5q6ri-gcc-14.3.0-lib/lib"
# libz needed by pip numpy wheels (used in the solve subprocess)
ZLIB="/nix/store/l7xwm1f6f3zj2x8jwdbi8gdyfbx07sh7-zlib-1.3.1/lib"

# Gradio UI path: venv packages first (for gradio/scipy/etc), then Nix numpy+cv2
export PYTHONPATH="${VENV}/lib/python3.13/site-packages:${NIX_NUMPY}:${NIX_CV2}"
export LD_LIBRARY_PATH="${GCC_LIB}:${ZLIB}:${LD_LIBRARY_PATH:-}"

exec "${VENV}/bin/python" "${SCRIPT_DIR}/app.py" "$@"
