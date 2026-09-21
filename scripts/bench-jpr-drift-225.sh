#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later
#
# GT-based JPR benchmark (issue #225 sub-task).
#
# Measures rux register on drifted ARKitScenes vs the #342 protocol.
# Key configs:
#   none         no pose stage — drifted baseline
#   optimize     plane-landmark pose graph (sanity check vs reference)
#   register-3   JPR --spatial-radius 0.5 --iterations 3 (~2h for 41069050)
#   register-t   temporal-only JPR (--spatial-radius 0.001, ~5min), no spatial loop closure
#
# Usage:
#   scripts/bench-jpr-drift-225.sh [-d dataset_dir] [-o out_dir] [-s scales]
#                                   [-S seeds] [-v "vid ..."] [-c "cfg ..."]

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
rux="$repo_root/build/apps/rux/rux"

data="$HOME/datasets/arkitscenes"
out="$repo_root/scratch/jpr-drift-bench"
scales="0.25"
seeds="1"
video_ids="41069050"
configs="none optimize register-3 register-t"

while getopts "d:o:s:S:v:c:" o; do
  case "$o" in
    d) data="$OPTARG" ;;
    o) out="$OPTARG" ;;
    s) scales="$OPTARG" ;;
    S) seeds="$OPTARG" ;;
    v) video_ids="$OPTARG" ;;
    c) configs="$OPTARG" ;;
    *) echo "usage: see header of $0" >&2; exit 2 ;;
  esac
done

if [[ ! -x "$rux" ]]; then
  echo "error: rux binary not found at $rux — build first" >&2
  exit 1
fi
mkdir -p "$out"

summary="$out/jpr-drift-bench-summary.txt"

score_line() {
  local tag="$1" q="$2" a="$3" t="$4"
  python3 - "$tag" "$q" "$a" "$t" >> "$summary" <<'PY'
import json, os, sys
tag, qpath, apath, elapsed = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
line = tag
if os.path.exists(qpath):
    q = json.load(open(qpath))
    line += (f" | flatness_rms={q['flatness_rms']*1000:.2f}mm "
             f"thickness_p90={q['thickness_p90']*1000:.2f}mm "
             f"planes={q.get('plane_count','?')}")
if os.path.exists(apath):
    a = json.load(open(apath))
    line += (f" | F@50mm={a['fscore']:.4f} "
             f"chamfer={a['chamfer']*1000:.2f}mm "
             f"acc_med={a['accuracy_median']*1000:.2f}mm "
             f"comp_med={a['completeness_median']*1000:.2f}mm")
line += f" | wall={elapsed}"
print(line)
PY
}

for vid in $video_ids; do
  src="$data/${vid}.rux"
  gt="$data/raw/Validation/$vid/${vid}_3dod_mesh.ply"
  if [[ ! -f "$src" ]]; then
    echo "!! missing $src" | tee -a "$summary"; continue
  fi

  for scale in $scales; do
    for seed in $seeds; do
      master="$out/${vid}-d${scale}-s${seed}.rux"
      # Reuse existing master if it already exists (supports interrupted restarts)
      if [[ ! -f "$master" ]]; then
        rm -f "$master" "$master-shm" "$master-wal"
        cp "$src" "$master"
        echo "==> $vid drift-scale $scale seed $seed: perturbing poses"
        "$rux" -v -p "$master" edit perturb-poses \
            --seed "$seed" --drift-scale "$scale" --yes \
            2>&1 | tee "$out/${vid}-d${scale}-s${seed}-perturb.log" \
          | grep -E "Frames:|Drift ratio:|Pose error:" || true
      else
        echo "==> $vid drift-scale $scale seed $seed: reusing existing master"
      fi

      for cfg in $configs; do
        qout="$out/${vid}-d${scale}-s${seed}-${cfg}.quality.json"
        aout="$out/${vid}-d${scale}-s${seed}-${cfg}.accuracy.json"
        # Skip already-scored configs (supports interrupted restarts)
        if [[ -f "$qout" && -f "$aout" ]]; then
          echo "==> $vid d=$scale seed=$seed $cfg: already scored, skipping"
          continue
        fi

        tag="$vid d=$scale seed=$seed $cfg"
        work="$out/${vid}-d${scale}-s${seed}-${cfg}.rux"
        rm -f "$work" "$work-shm" "$work-wal"
        cp "$master" "$work"
        echo "==> $tag"

        t_start=$(date +%s)

        case "$cfg" in
          none)         : ;;
          optimize)     "$rux" -p "$work" optimize ;;
          register-3)   "$rux" -v -p "$work" register --spatial-radius 0.5 --iterations 3
                        echo "NOTE: register-3 uses only 3 iterations (budget constraint); not fully converged" ;;
          register-20)  "$rux" -v -p "$work" register --spatial-radius 0.5 ;;
          register-t)   "$rux" -v -p "$work" register --spatial-radius 0.001
                        echo "NOTE: register-t uses r≈0 (temporal-only), no spatial loop closure" ;;
          opt-register) "$rux" -v -p "$work" optimize &&
                        "$rux" -v -p "$work" register --spatial-radius 0.5 ;;
          *) echo "unknown config '$cfg'" >&2; exit 2 ;;
        esac

        "$rux" -p "$work" create clouds -g 0.05
        "$rux" -p "$work" create planes
        "$rux" -p "$work" analyze quality -o "$qout"
        if [[ -f "$gt" ]]; then
          "$rux" -p "$work" analyze accuracy "$gt" -o "$aout"
        fi

        t_end=$(date +%s)
        elapsed="$(( (t_end - t_start) / 60 ))m$(( (t_end - t_start) % 60 ))s"

        score_line "$tag" "$qout" "$aout" "$elapsed"
        rm -f "$work" "$work-shm" "$work-wal"
      done
    done
  done
done

echo "==> results"
cat "$summary"
echo "BENCH-JPR-DRIFT-225-DONE"
