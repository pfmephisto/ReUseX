#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later
#
# One-shot ARKitScenes benchmark run (issue #224 / #221 Tier 2): download a few
# Validation scenes (Apple ARKitScenes, research-only license), import each via
# `rux import arkitscenes`, run the reconstruction pipeline, and score it with
# `rux analyze quality` (internal metrics) and `rux analyze accuracy` against the
# per-scene ARKit ground-truth mesh (<video_id>_3dod_mesh.ply) — which is in the
# same trajectory frame as the imported poses, so no registration is needed.
#
# ARKitScenes is iPad-Pro-LiDAR capture: the same sensor class as ReUseX's own
# scans, making it an external ground-truth benchmark for reconstruction quality.
#
# Usage: scripts/bench-arkitscenes.sh [dataset_dir] [video_id ...]
#   dataset_dir default: ~/datasets/arkitscenes
#   video_id    default: 41069050 41069048 41069051 (small Validation scenes)
#
# Data is downloaded outside the repo and is NOT committed (research-only license,
# see Apple's ARKitScenes terms). Safe to re-run: downloads/extractions resume.

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
rux="$repo_root/build/apps/rux/rux"
data="${1:-$HOME/datasets/arkitscenes}"
shift || true
video_ids=("${@:-}")
if [[ -z "${video_ids[*]}" ]]; then
  video_ids=(41069050 41069048 41069051)
fi

if [[ ! -x "$rux" ]]; then
  echo "error: rux binary not found at $rux — build first (cmake --build build)" >&2
  exit 1
fi

mkdir -p "$data"

# ── [1/4] tooling: apple/ARKitScenes downloader + a venv with pandas ──────────
tools="$HOME/tools/ARKitScenes"
if [[ ! -f "$tools/download_data.py" ]]; then
  echo "==> [1/4] cloning apple/ARKitScenes downloader"
  git clone --depth 1 https://github.com/apple/ARKitScenes.git "$tools"
fi
venv="$data/.venv"
if [[ ! -x "$venv/bin/python3" ]]; then
  echo "==> [1/4] creating download venv (needs pandas)"
  python3 -m venv "$venv"
  "$venv/bin/pip" install -q --upgrade pip pandas
fi

# ── [2/4] download the requested scenes (lowres streams + GT mesh only) ───────
echo "==> [2/4] downloading ${#video_ids[@]} Validation scene(s): ${video_ids[*]}"
( cd "$tools" && "$venv/bin/python3" download_data.py raw \
    --split Validation \
    --video_id "${video_ids[@]}" \
    --download_dir "$data" \
    --raw_dataset_assets lowres_wide lowres_depth confidence \
      lowres_wide.traj lowres_wide_intrinsics mesh )

# ── [3/4] per-scene pipeline: import -> clouds -> planes -> quality/accuracy ──
summary="$data/arkitscenes-bench-summary.txt"
: > "$summary"
for vid in "${video_ids[@]}"; do
  scene="$data/raw/Validation/$vid"
  gt="$scene/${vid}_3dod_mesh.ply"
  proj="$data/${vid}.rux"
  echo "==> [3/4] scene $vid"
  if [[ ! -f "$scene/lowres_wide.traj" ]]; then
    echo "  !! missing lowres_wide.traj under $scene — skipping" | tee -a "$summary"
    continue
  fi
  rm -f "$proj" "$proj-shm" "$proj-wal"
  "$rux" -p "$proj" import arkitscenes "$scene"
  "$rux" -p "$proj" create clouds
  "$rux" -p "$proj" create planes
  "$rux" -p "$proj" analyze quality  -o "$data/${vid}-quality.json"
  if [[ -f "$gt" ]]; then
    "$rux" -p "$proj" analyze accuracy "$gt" -o "$data/${vid}-accuracy.json"
  else
    echo "  !! no GT mesh $gt — skipping accuracy" >&2
  fi

  # Compact one-line-per-scene summary (flatness / thickness / F-score).
  python3 - "$vid" "$data/${vid}-quality.json" "$data/${vid}-accuracy.json" >> "$summary" <<'PY'
import json, sys, os
vid, qpath, apath = sys.argv[1], sys.argv[2], sys.argv[3]
q = json.load(open(qpath))
line = (f"{vid}: flatness_rms={q['flatness_rms']*1000:.1f}mm "
        f"thickness_p90={q['thickness_p90']*1000:.1f}mm "
        f"planes={q.get('plane_count','?')}")
if os.path.exists(apath):
    a = json.load(open(apath))
    line += (f" | fscore@5cm={a['fscore']:.3f} "
             f"acc_med={a['accuracy_median']*1000:.1f}mm "
             f"comp_med={a['completeness_median']*1000:.1f}mm")
print(line)
PY
done

# ── [4/4] results ─────────────────────────────────────────────────────────────
echo "==> [4/4] results"
cat "$summary"
echo "BENCH-ARKITSCENES-DONE"
