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

# Core deps
$VENV/bin/pip install gradio numpy==1.26.4 scipy==1.14.1 gtsam opencv-python
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
   shows all frame positions from the seed poses.

2. **Pick a pair.** Use the Frame A and Frame B scrubbers to choose frames.
   Click "Propose candidates for A" to find spatially nearby frames by seed-pose
   proximity.

3. **Run a match.** Choose a method from the dropdown:
   - `orb`, `sift`, `akaze` — OpenCV in-process matchers
   - `xfeat` — XFeat learned features (subprocess, `~/loop-edges-work/xfeat/.venv`)
   - `mast3r` — MASt3R (subprocess, `~/loop-edges-work/mast3r/.venv`, CC-BY-NC)
   - `icp` — Manual + ICP mode (use the sliders to coarse-align, then Run ICP)

   Click "Run match". The correspondence image shows green/red inlier lines.

4. **Accept or redo.** Click "Accept last match" to append to the anchor list.
   Repeat for more pairs.

5. **Export anchors.** Click "Export anchors" to write a JSON file in the
   `reusex.gt_anchors.v1` schema that `solve_gt_poses_cli.py` consumes.

6. **Live GT preview (optional).** Fill in the seed `.rux` path and optionally
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
| `opencv_features.py` | In-process ORB/SIFT/AKAZE matching + T_ij extraction |
| `icp_align.py` | Point-to-point ICP (scipy/numpy, no open3d) + slider→T helper |
| `anchors.py` | Anchor list management + JSON export in `gt_anchors.v1` schema |
| `match_worker.py` | Subprocess worker for XFeat/MASt3R (runs inside matcher venvs) |
| `solve_gt_poses_cli.py` | GTSAM solve CLI (generalised from `solve_gt_poses.py`) |
| `launch.sh` | Shell wrapper that sets the required env vars |
| `tests/test_convention.py` | Unit tests: T_ij convention, anchor round-trip, ICP |

## What is built and working

- Frame panel: top-down scatter, A/B scrubbers, thumbnails, candidate proposal
- Verification panel: ORB/SIFT/AKAZE (in-process), XFeat/MASt3R (subprocess)
- Manual + ICP mode: x/y/z + yaw sliders, Run ICP button
- Anchor list: add, remove, export JSON, import JSON
- solve_gt_poses_cli.py: generalised from solve_gt_poses.py with `--edges`/`--out`
- Live GT preview: wired end-to-end (solve → clouds → render → before/after images)
- 11 unit tests passing

## What is stubbed / not fully verified

- **MapAnything backend**: hook present in `match_worker.py` CLI (arg accepted)
  but returns `"mapanything not implemented"`. The matchers/__init__.py code is
  already there — only the `match_worker.py` dispatch needs `elif backend == "mapanything"`.
- **Live preview timing**: the full 3,876-frame solve + clouds + render was wired
  and the solve path is verified (0.2s with 1 anchor). Full pipeline timing on
  NewOffice was not measured in this task (~2 min expected from the existing
  solve_gt_poses.py experience).
- **rux render before/after**: `rux render` requires a fused cloud to be present;
  for the seed project a cloud may not exist, so before-image may be blank/error.
  The code handles this gracefully (reads None from cv2.imread on failure).
- **open3d**: pip wheels for open3d don't work on this Nix system (missing X11/EGL/usb
  libs). ICP is implemented in pure numpy+scipy instead — this is equally correct.
  Point-to-plane ICP (open3d's advantage) is not available; point-to-point is used.
- **MASt3R tested**: XFeat subprocess was verified to return 117 inliers on node
  1↔1370. MASt3R subprocess bridge is wired identically but was not run in this
  task (it requires ~30GB VRAM or a long CPU run).
