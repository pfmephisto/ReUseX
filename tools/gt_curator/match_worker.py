#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""
Subprocess worker for learned matchers (MASt3R, XFeat).

This script is meant to be invoked INSIDE one of the matcher venvs:
  ~/loop-edges-work/xfeat/.venv/bin/python match_worker.py ...
  ~/loop-edges-work/mast3r/.venv/bin/python match_worker.py ...

It reads two frames from a .rux database, runs the requested matcher,
back-projects matches to 3D via the stored depth, and runs RANSAC-Kabsch.
Output is a single JSON line on stdout.

Convention (identical to export_loop_edges.py):
  T_ij maps a 3D point from frame j's optical frame into frame i's optical frame
  <=> T_ij = pose(i)^-1 @ pose(j)   (world-from-camera convention)

MapAnything: hook is present (CLI arg accepted) but NOT wired — returns an
error with the message "mapanything not implemented" so the app can surface it.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

# Inject loop_edges into the path — same trick used by visualize_matches.py.
# The path below is relative to this file's location inside the worktree.
_this = Path(__file__).resolve()
_loop_edges = _this.parent.parent / "loop_edges"
sys.path.insert(0, str(_loop_edges))

from export_loop_edges import backproject, ransac_pose, read_frames  # noqa: E402
from matchers import load_matcher  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description="Subprocess matcher worker for gt_curator")
    ap.add_argument("project", help="path to .rux database")
    ap.add_argument("node_i", type=int)
    ap.add_argument("node_j", type=int)
    ap.add_argument("backend", choices=["xfeat", "mast3r", "mapanything"])
    ap.add_argument("--ransac-thresh", type=float, default=0.10)
    ap.add_argument("--ransac-iters", type=int, default=500)
    ap.add_argument("--min-depth", type=float, default=0.3)
    ap.add_argument("--max-depth", type=float, default=5.0)
    ap.add_argument("--max-matches", type=int, default=4000)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--weights", default=None)
    ap.add_argument("--variant", default=None)
    ap.add_argument("--allow-noncommercial", action="store_true")
    args = ap.parse_args()

    def fail(msg: str):
        print(json.dumps({"error": msg, "T_ij": None, "n_inliers": 0,
                          "rms": None, "xy_i": [], "xy_j": [], "inlier_mask": []}))
        sys.exit(0)

    # MapAnything hook — not yet wired
    if args.backend == "mapanything":
        fail("mapanything not implemented in match_worker (hook only)")

    frames = read_frames(args.project)
    by_id = {f.node_id: f for f in frames}
    fi = by_id.get(args.node_i)
    fj = by_id.get(args.node_j)
    if fi is None:
        fail(f"node_id {args.node_i} not found")
    if fj is None:
        fail(f"node_id {args.node_j} not found")

    # Load matcher using args namespace (load_matcher reads .matcher, .device, etc.)
    args.matcher = args.backend
    try:
        matcher = load_matcher(args)
    except Exception as exc:
        fail(f"load_matcher failed: {exc}")

    result = matcher.match(fi, fj, max_matches=args.max_matches)
    if result is None:
        fail("matcher returned no matches")

    xy_i, xy_j = result
    p_i, vi = backproject(xy_i, fi.depth_m, fi.K, args.min_depth, args.max_depth)
    p_j, vj = backproject(xy_j, fj.depth_m, fj.K, args.min_depth, args.max_depth)
    both = vi & vj

    if int(both.sum()) < 3:
        fail("too few valid depth pixels after backprojection")

    rng = np.random.default_rng(args.seed)
    T_ij, inl = ransac_pose(p_j[both], p_i[both], args.ransac_thresh, args.ransac_iters, rng)

    inlier_mask = np.zeros(len(xy_i), dtype=bool)
    if T_ij is not None and inl is not None:
        inlier_mask[np.flatnonzero(both)[inl]] = True

    n_inliers = int(inlier_mask.sum())
    rms = None
    if T_ij is not None and n_inliers >= 3:
        p_j_inl = p_j[inlier_mask]
        p_i_inl = p_i[inlier_mask]
        pred = (p_j_inl @ T_ij[:3, :3].T) + T_ij[:3, 3]
        rms = float(np.sqrt(np.mean(np.sum((pred - p_i_inl) ** 2, axis=1))))

    out = {
        "error": None,
        "xy_i": xy_i.tolist(),
        "xy_j": xy_j.tolist(),
        "inlier_mask": inlier_mask.tolist(),
        "T_ij": T_ij.tolist() if T_ij is not None else None,
        "n_inliers": n_inliers,
        "rms": rms,
    }
    print(json.dumps(out))


if __name__ == "__main__":
    main()
