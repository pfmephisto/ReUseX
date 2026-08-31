#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later
#
# One-shot MuSHRoom benchmark run (issue #221/#224): download the honka room
# (CC-BY-4.0, Zenodo), import it via `rux import mushroom`, run the
# reconstruction pipeline, and score it with `rux analyze quality` — both
# with original poses and with JPR refinement (best-known parameters).
#
# Usage: scripts/bench-mushroom.sh [dataset_dir]   (default ~/datasets/mushroom)
# Safe to re-run: downloads resume, projects are recreated from scratch.

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
rux="$repo_root/build/apps/rux/rux"
data="${1:-$HOME/datasets/mushroom}"

mkdir -p "$data"
cd "$data"

echo "==> [1/6] downloading honka (resumable; ~3.6 GB total)"
curl -LC - --retry 5 \
  "https://zenodo.org/records/10151161/files/honka_iphone_our.tar.gz?download=1" \
  -o honka_iphone_our.tar.gz
curl -LC - --retry 5 \
  "https://zenodo.org/records/10222321/files/honka_mesh_pd.tar.gz?download=1" \
  -o honka_mesh_pd.tar.gz

echo "==> [2/6] extracting"
if [[ ! -d honka ]]; then
  tar xzf honka_iphone_our.tar.gz
  tar xzf honka_mesh_pd.tar.gz
fi
find "$data" -maxdepth 3 -type d | sort

# Locate the iPhone long capture (layout may nest under honka/iphone/...).
capture="$(find "$data" -maxdepth 4 -type d -name long_capture -path "*iphone*" | head -1)"
if [[ -z "$capture" ]]; then
  echo "error: could not find iphone long_capture directory under $data" >&2
  exit 1
fi
echo "capture dir: $capture"

echo "==> [3/6] format diagnostics (for importer verification)"
ls "$capture" | head
echo "--- transformations.json header + first frame ---"
python3 - "$capture/transformations.json" <<'PYEOF'
import json, sys
meta = json.load(open(sys.argv[1]))
hdr = {k: v for k, v in meta.items() if k != "frames"}
print(json.dumps(hdr, indent=1)[:1200])
print("frame count:", len(meta.get("frames", [])))
print(json.dumps(meta["frames"][0], indent=1)[:900])
PYEOF
echo "--- one depth file ---"
depth_sample="$(find "$capture" -maxdepth 2 -path "*depth*" -name "*.png" | head -1)"
file "$depth_sample"

echo "==> [4/6] import + reconstruct (baseline poses)"
rm -f honka.rux honka.rux-shm honka.rux-wal
"$rux" -v -p honka.rux import mushroom "$capture"
"$rux" -v -p honka.rux create clouds
"$rux" -v -p honka.rux create planes
"$rux" -v -p honka.rux analyze quality -o honka-quality-baseline.json

echo "==> [5/6] JPR-refined variant"
cp -f honka.rux honka-jpr.rux
"$rux" -v -p honka-jpr.rux register --prior-weight 0.1 --neighbor-window 10 --iterations 50
"$rux" -v -p honka-jpr.rux create clouds
"$rux" -v -p honka-jpr.rux create planes
"$rux" -v -p honka-jpr.rux analyze quality -o honka-quality-jpr.json

echo "==> [6/6] results"
echo "--- baseline ---"
head -8 honka-quality-baseline.json
echo "--- jpr ---"
head -8 honka-quality-jpr.json
echo "BENCH-MUSHROOM-DONE"
