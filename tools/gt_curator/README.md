<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# GT Curator — Gradio tool for manual anchor curation (issue #221)

A browser-based GUI for manually curating trusted frame-pair correspondences
to build a one-off ground-truth pose dataset for ReUseX issue #221.

## Dependencies

The tool uses two separate Python environments to keep conflicting torch/CUDA
versions isolated:

| Component | Python env | Notes |
|---|---|---|
| Gradio UI, OpenCV matchers, ICP | `~/gt-curator-venv` | Created by the setup below |
| XFeat matcher (subprocess) | `~/loop-edges-work/xfeat/.venv` | Pre-existing |
| MASt3R matcher (subprocess) | `~/loop-edges-work/mast3r/.venv` | Pre-existing |
| GT solve (GTSAM, subprocess) | `~/gt-curator-venv` | Same venv; gtsam + scipy 1.14 |

### Creating the curator venv

```bash
python3 -m venv ~/gt-curator-venv
VENV=~/gt-curator-venv

# Core deps (numpy 2.x is fine; gtsam 4.2.2 is compatible)
$VENV/bin/pip install gradio scipy==1.14.1 gtsam opencv-python
$VENV/bin/pip install packaging pyyaml Pillow markupsafe jinja2 starlette
$VENV/bin/pip install six click certifi idna httpx tomlkit

# opencv-python wheels need X11; instead, we use Nix's opencv at runtime via PYTHONPATH.
# Remove the pip opencv-python to avoid conflicts:
$VENV/bin/pip uninstall -y opencv-python
```

The launch script sets the required PYTHONPATH and LD_LIBRARY_PATH automatically.

## How to launch

From the worktree root:

```bash
./tools/gt_curator/launch.sh -p /path/to/project.rux [--port 7860]
```

Or equivalently:

```bash
VENV=~/gt-curator-venv
NIX_NUMPY=/nix/store/l59n6vzkswz23y6s4pr6cmv2p4dpd5f0-python3.13-numpy-2.4.4/lib/python3.13/site-packages
NIX_CV2=/nix/store/kd9y1ckp87vh5c341rnngrqkhjdibxf1-opencv-4.13.0/lib/python3.13/site-packages
GCC_LIB=/nix/store/xm08aqdd7pxcdhm0ak6aqb1v7hw5q6ri-gcc-14.3.0-lib/lib
ZLIB=/nix/store/l7xwm1f6f3zj2x8jwdbi8gdyfbx07sh7-zlib-1.3.1/lib

PYTHONPATH="${VENV}/lib/python3.13/site-packages:${NIX_NUMPY}:${NIX_CV2}" \
LD_LIBRARY_PATH="${GCC_LIB}:${ZLIB}" \
${VENV}/bin/python tools/gt_curator/app.py -p /path/to/project.rux
```

Then open http://localhost:7860 in your browser.

## Workflow

1. **Load a project.** Pass `-p project.rux` at launch. The top-down scatter
   shows all frame positions from the seed poses as an **interactive Plotly
   figure** — zoom with scroll, pan with drag.

2. **Pick a pair.** Three ways to select frames:
   - **Sliders** — Frame A and Frame B scrubbers (original, always works).
   - **Node-ID box** — hover over the graph to read a node_id from the tooltip,
     then type it in the "Set A/B by node_id" box and click Set.  Toggle the
     "Selecting A/B" radio to control which frame gets set.
   - **Row select in tables** — selecting a row in the loaded-edges table,
     the match-history table, or the anchor table loads that pair into A/B.
   - Click "Propose candidates for A" to find spatially nearby frames by seed-pose
     proximity.

3. **Load prior-computed edges (optional).** Paste a path to an edge JSON file
   or directory (e.g. `pseudo-gt/edges/xfeat_edges.json` or the whole
   `pseudo-gt/edges/` directory) and click "Load". The edges appear as a
   browsable table and as faint lines on the graph.  Select any row to open
   that pair in the verification panel.

4. **Run a match.** Choose a method from the dropdown:
   - `orb`, `sift`, `akaze` — OpenCV in-process matchers
   - `xfeat` — XFeat learned features (subprocess, `~/loop-edges-work/xfeat/.venv`)
   - `mast3r` — MASt3R (subprocess, `~/loop-edges-work/mast3r/.venv`, CC-BY-NC)
   - `icp` — Manual + ICP mode (use the sliders to coarse-align, then Run ICP)

   Click "Run match". The correspondence image shows green/red inlier lines.
   Every match is recorded in the **Session match history** table.

5. **Accept or redo.** Click "Accept last match" to append to the anchor list.
   Accepted anchors are immediately drawn as solid green lines on the graph.
   The anchor table is always visible; selecting a row re-opens that pair.

6. **Export anchors.** Click "Export anchors" to write a JSON file in the
   `reusex.gt_anchors.v1` schema that `solve_gt_poses_cli.py` consumes.

7. **Live GT preview (optional).** Fill in the seed `.rux` path and optionally
   the anchor-zone max sequential index (e.g. 567 for NewOffice), then click
   "Run GT preview". This runs the full GTSAM solve on a copy, then
   `rux create clouds` and `rux render --view top` to show before/after images.
   Expect ~2 minutes for 3,876 frames.

## T_ij convention

`T_ij = pose(i)^-1 · pose(j)` (world-from-camera convention).

This is identical to the convention in `solve_gt_poses.py` and
`export_loop_edges.py`. In the optical frame:
`p_i ≈ T_ij · p_j` (maps 3D points from frame j into frame i).

Verified by `tests/test_convention.py::TestTijConvention::test_seed_relative_round_trip`
(translation diff: 0.00mm, rotation diff: 0.0000°).

## Module layout

| File | Purpose |
|---|---|
| `app.py` | Gradio UI — all callbacks and server startup |
| `db_reader.py` | Read frame positions, thumbnails, poses from `.rux` |
| `graph_view.py` | Plotly interactive top-down figure builder (zoom/pan, overlays) |
| `edge_io.py` | Load prior-computed edge JSON files (xfeat/mast3r/spatial-filter) |
| `history.py` | Session match history (one row per cb_match invocation) |
| `opencv_features.py` | In-process ORB/SIFT/AKAZE matching + T_ij extraction |
| `icp_align.py` | Point-to-point ICP (scipy/numpy, no open3d) + slider→T helper |
| `anchors.py` | Anchor list management + JSON export in `gt_anchors.v1` schema |
| `match_worker.py` | Subprocess worker for XFeat/MASt3R (runs inside matcher venvs) |
| `solve_gt_poses_cli.py` | GTSAM solve CLI (generalised from `solve_gt_poses.py`) |
| `launch.sh` | Shell wrapper that sets the required env vars |
| `tests/test_convention.py` | Unit tests: T_ij convention, anchor round-trip, ICP |
| `tests/test_new_features.py` | Unit tests: edge_io, history, graph_view, callback logic |

## What is built and working

- **Interactive graph** (`gr.Plot` with Plotly): zoom/pan natively; A/B markers;
  accepted anchor overlays (solid green lines); loaded prior-edge overlays (faint
  blue lines); graph refreshes on every selection/accept/import/load.
- **Node-ID direct select**: type a node_id (read from graph hover tooltip) + radio
  to choose A/B + "Set" button; sets the matching slider and loads the thumbnail.
- **Load prior-computed edges**: file or directory path; parses all known schema
  variants (`inliers`, `n_inliers`, `num_inliers`); browsable Dataframe; row-select
  loads the pair into A/B and shows the pre-computed T_ij.
- **Session match history**: one row per `cb_match` invocation; accepted? column;
  row-select re-opens the pair in A/B.
- Frame panel: A/B scrubbers (retained), thumbnails, candidate proposal.
- Verification panel: ORB/SIFT/AKAZE (in-process), XFeat/MASt3R (subprocess).
- Manual + ICP mode: x/y/z + yaw sliders, Run ICP button.
- Anchor list: add, remove, export JSON, import JSON; anchor table row-select re-opens pair.
- solve_gt_poses_cli.py: generalised from solve_gt_poses.py with `--edges`/`--out`.
- Live GT preview: wired end-to-end (solve → clouds → render → before/after images).
- **46 unit tests passing** (11 original + 35 new).

## Plotly click-to-select: status and fallback

`gr.Plot` in Gradio 6.27 exposes only a `.change` event (no `.select` or `.click`).
Plotly's `clickData` cannot be bridged to Python callbacks without custom JavaScript.

Fallback implemented: the "Set A/B by node_id" input + "Set" button.  Workflow:
hover over a node in the Plotly graph to see its tooltip (`node_id=NNN`), type that
NNN into the box, toggle the A/B radio, click Set.  This is reliable and works
headlessly.  The sliders also remain fully functional as before.

## What is stubbed / not fully verified

- **Plotly click-to-select**: not available in Gradio 6.27 (see above).  The
  node-ID input is the supported workaround.
- **MapAnything backend**: hook present in `match_worker.py` CLI (arg accepted)
  but returns `"mapanything not implemented"`.
- **Live preview timing**: ~2 min expected from the existing solve_gt_poses.py
  experience; not re-measured in this task.
- **rux render before/after**: requires a fused cloud; handles gracefully if absent.
- **open3d**: not available on this Nix system; ICP is pure numpy+scipy.
- **MASt3R tested**: subprocess bridge is wired but not run (requires ~30GB VRAM).
