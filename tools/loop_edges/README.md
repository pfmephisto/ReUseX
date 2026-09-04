<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# `tools/loop_edges` — external wide-baseline loop-edge exporter

Out-of-process producer for `rux optimize --loop-edges` (issues #221 / #225 P2).
Runs an image matcher in Python, lifts the 2D matches to metric 3D using the
scan's own stored depth, estimates a robust relative pose per candidate pair,
and writes a JSON file (schema `reusex.loop_edges.v1`) that the C++ optimizer
ingests as **data**. See
[`docs/research/loop-closure-learned-matchers.md`](../../docs/research/loop-closure-learned-matchers.md).

## Why out-of-process

Two problems with putting a learned matcher directly into the GPL C++ binary:

1. **Integration** — the best matchers are Python-only research code.
2. **Licensing** — MASt3R and MapAnything's 13-dataset checkpoint are
   **CC-BY-NC** (non-commercial); ReUseX is a commercial product.

Both vanish if the matcher runs out-of-process and hands the C++ a plain JSON of
relative-pose constraints. A non-commercial model can then act as an offline
accuracy-**ceiling oracle** without ever entering the shipped binary, while a
commercial-safe matcher writes the identical file for production.

## Matcher backends

| backend | licence | use |
|---|---|---|
| `orb` | BSD (OpenCV) | zero-dependency baseline / parity check; ships as the C++ default |
| `xfeat` | Apache-2.0 | commercial-safe learned features |
| `lightglue` | Apache-2.0 | commercial-safe (LightGlue + ALIKED/DISK — **not** SuperPoint, which is NC) |
| `mast3r` | **CC-BY-NC-SA** | **oracle only**; needs `--allow-noncommercial` |
| `mapanything` | Apache-2.0 (`--variant apache`) or CC-BY-NC | apache checkpoint is shippable; NC checkpoint is oracle-only |

**Commercial rule:** output from `mast3r` or `mapanything --variant nc` is an
evaluation artefact. It must not be bundled into or shipped with a commercial
deliverable. The tool prints a banner and gates these behind
`--allow-noncommercial`.

## Environment

Each learned backend has its own venv (dependency isolation). `orb` needs only
`numpy` + `opencv-python` (already present in the dev shell). Setup for
`xfeat` / `mast3r` / `mapanything` is documented in the PR; venvs live under
`~/loop-edges-work/<backend>/` (outside the repo, not committed).

## Usage

```bash
# ORB baseline (no GPU needed)
python3 tools/loop_edges/export_loop_edges.py PROJECT.rux \
    -o edges-orb.json --matcher orb --proposal exhaustive --min-frame-gap 50

# XFeat, commercial-safe (GPU)
~/loop-edges-work/xfeat/.venv/bin/python tools/loop_edges/export_loop_edges.py \
    PROJECT.rux -o edges-xfeat.json --matcher xfeat

# Target a start<->end drift loop the spatial proposer is blind to
... --proposal endcap --band-frac 0.15

# MASt3R oracle (research/eval only)
~/loop-edges-work/mast3r/.venv/bin/python tools/loop_edges/export_loop_edges.py \
    PROJECT.rux -o edges-mast3r.json --matcher mast3r --allow-noncommercial

# Feed edges into the pose graph (license-clean; C++ never links a matcher)
rux -p PROJECT.rux optimize --loop-edges edges-xfeat.json --loop-trust \
    --odometry-sigma-trans 0.05
rux -p PROJECT.rux create clouds
```

## Edge file schema (`reusex.loop_edges.v1`)

```json
{
  "schema": "reusex.loop_edges.v1",
  "producer": "xfeat",
  "edges": [
    { "node_i": 12, "node_j": 230,
      "T_ij": [ 16 doubles, row-major 4x4 = pose(i)^-1 * pose(j) ],
      "sigma_rot": 0.03, "sigma_trans": 0.05, "inliers": 120 }
  ]
}
```

`node_i` / `node_j` are `sensor_frames.node_id`. `T_ij` maps a point from frame
`j`'s optical frame into frame `i`'s optical frame (optical→world convention).
Edges referencing unknown node ids, self-loops, duplicates, or malformed entries
are skipped with a warning; an unreadable/invalid file fails loudly.
