#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Out-of-process wide-baseline loop-edge exporter for `rux optimize --loop-edges`
# (issue #221 / #225 P2).
#
# WHY THIS EXISTS (license + integration boundary)
# ------------------------------------------------
# The strongest wide-baseline image correspondences come from learned matchers
# and pointmap foundation models. Two of the best (MASt3R, MapAnything's
# 13-dataset checkpoint) are CC-BY-NC — they must NOT enter a commercial binary,
# and several are Python-only research code that is painful to link into GPL C++.
#
# This script keeps them entirely OUT of the shipped product: it runs a matcher
# in Python, lifts the 2D matches to metric 3D using the scan's own stored depth,
# estimates a robust relative pose per pair, and writes a plain-DATA JSON file
# (schema "reusex.loop_edges.v1") that `rux optimize --loop-edges` ingests. The
# C++ never links the matcher. A non-commercial model can therefore serve as an
# offline accuracy-CEILING ORACLE, while a commercial-safe matcher writes the
# identical file for the production path.
#
# Matcher backends (see matchers/):
#   COMMERCIAL-SAFE (Apache-2.0 / MIT — shippable path):
#     orb          OpenCV ORB (BSD). Zero extra deps. Baseline / parity check.
#     xfeat        XFeat accelerated features (Apache-2.0).
#     lightglue    LightGlue + ALIKED/DISK (Apache-2.0). NOT SuperPoint (NC).
#   RESEARCH-ONLY ORACLES (CC-BY-NC — never ship; --allow-noncommercial gate):
#     mast3r       MASt3R metric pointmaps (CC-BY-NC-SA).
#     mapanything  MapAnything (use the *apache* checkpoint for a ship-safe run;
#                  the CC-BY-NC checkpoint is an oracle only).
#
# The T_ij convention matches reusex::geometry::LoopEdge exactly: T_ij maps a
# point from frame j's optical frame into frame i's optical frame
# (= pose(i)^-1 * pose(j) in the optical->world convention the graph uses). It is
# recovered purely from backprojected optical-frame 3D correspondences, so it is
# independent of the (possibly drifted) stored world poses — the metric scale
# comes from the RGB-D depth, which is exactly why we do NOT need MASt3R's
# metric-from-RGB capability here.

import argparse
import json
import sqlite3
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from matchers import load_matcher  # noqa: E402

SCHEMA = "reusex.loop_edges.v1"


# --------------------------------------------------------------------------- #
# Frame I/O                                                                    #
# --------------------------------------------------------------------------- #
class Frame:
    __slots__ = ("node_id", "gray", "color", "depth_m", "K")

    def __init__(self, node_id, gray, color, depth_m, K):
        self.node_id = node_id
        self.gray = gray
        self.color = color
        self.depth_m = depth_m  # float32 metres, 0 = invalid
        self.K = K  # 3x3


def read_frames(db_path, stride=1, max_frames=None):
    con = sqlite3.connect(db_path)
    cur = con.cursor()
    cur.execute(
        "SELECT node_id, color, depth, camera_model FROM sensor_frames "
        "WHERE color IS NOT NULL AND depth IS NOT NULL ORDER BY node_id"
    )
    rows = cur.fetchall()
    con.close()

    frames = []
    for idx, (node_id, color_blob, depth_blob, cam_json) in enumerate(rows):
        if idx % stride != 0:
            continue
        color = cv2.imdecode(np.frombuffer(color_blob, np.uint8), cv2.IMREAD_COLOR)
        depth = cv2.imdecode(np.frombuffer(depth_blob, np.uint8), cv2.IMREAD_UNCHANGED)
        if color is None or depth is None:
            continue
        cam = json.loads(cam_json)
        K = np.array(
            [[cam["fx"], 0, cam["cx"]], [0, cam["fy"], cam["cy"]], [0, 0, 1]],
            dtype=np.float64,
        )
        # Depth may be stored at a different resolution than the intrinsics
        # reference; scale K to the actual depth grid so backprojection is exact.
        dh, dw = depth.shape[:2]
        if dw != cam["width"] or dh != cam["height"]:
            sx, sy = dw / cam["width"], dh / cam["height"]
            K = K.copy()
            K[0, 0] *= sx
            K[0, 2] *= sx
            K[1, 1] *= sy
            K[1, 2] *= sy
        depth_m = depth.astype(np.float32) / 1000.0  # mm -> m
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        # Match gray to depth resolution for pixel-aligned backprojection.
        if gray.shape[:2] != depth_m.shape[:2]:
            gray = cv2.resize(gray, (dw, dh), interpolation=cv2.INTER_AREA)
            color = cv2.resize(color, (dw, dh), interpolation=cv2.INTER_AREA)
        frames.append(Frame(node_id, gray, color, depth_m, K))
        if max_frames and len(frames) >= max_frames:
            break
    return frames


# --------------------------------------------------------------------------- #
# Geometry                                                                     #
# --------------------------------------------------------------------------- #
def backproject(pts_xy, depth_m, K, min_depth, max_depth):
    """Backproject Nx2 pixel coords to Nx3 optical-frame points; returns the 3D
    points and a boolean mask of pixels with valid depth in [min,max]."""
    u = pts_xy[:, 0]
    v = pts_xy[:, 1]
    ui = np.round(u).astype(int)
    vi = np.round(v).astype(int)
    h, w = depth_m.shape[:2]
    inb = (ui >= 0) & (ui < w) & (vi >= 0) & (vi < h)
    z = np.zeros(len(u), np.float32)
    z[inb] = depth_m[vi[inb], ui[inb]]
    valid = inb & (z >= min_depth) & (z <= max_depth) & np.isfinite(z)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.stack([x, y, z], axis=1).astype(np.float64), valid


def umeyama_rigid(src, dst):
    """Least-squares rigid transform T (4x4) with dst ~= T @ src (no scale)."""
    cs = src.mean(0)
    cd = dst.mean(0)
    H = (src - cs).T @ (dst - cd)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1, 1, d])
    R = Vt.T @ D @ U.T
    t = cd - R @ cs
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def ransac_pose(p_src, p_dst, thresh, iters, rng):
    """RANSAC rigid pose with p_dst ~= T @ p_src. Returns (T, inlier_mask)."""
    n = len(p_src)
    if n < 3:
        return None, None
    best_inl = None
    best_cnt = 0
    for _ in range(iters):
        idx = rng.choice(n, 3, replace=False)
        if np.linalg.matrix_rank(p_src[idx] - p_src[idx][0]) < 2:
            continue
        T = umeyama_rigid(p_src[idx], p_dst[idx])
        res = np.linalg.norm((p_src @ T[:3, :3].T + T[:3, 3]) - p_dst, axis=1)
        inl = res < thresh
        c = int(inl.sum())
        if c > best_cnt:
            best_cnt = c
            best_inl = inl
    if best_inl is None or best_cnt < 3:
        return None, None
    T = umeyama_rigid(p_src[best_inl], p_dst[best_inl])  # refit on inliers
    res = np.linalg.norm((p_src @ T[:3, :3].T + T[:3, 3]) - p_dst, axis=1)
    inl = res < thresh
    if int(inl.sum()) >= 3:
        T = umeyama_rigid(p_src[inl], p_dst[inl])
    return T, inl


def edge_sigmas(inliers, ref, base_t, base_r, floor_t, floor_r):
    s = np.sqrt(ref / max(inliers, 1))
    return (
        float(max(base_t * s, floor_t)),
        float(max(base_r * s, floor_r)),
    )


# --------------------------------------------------------------------------- #
# Candidate proposal                                                          #
# --------------------------------------------------------------------------- #
def propose_pairs(n, min_gap, max_pairs, rng, mode="exhaustive", band_frac=0.15):
    """Candidate loop pairs.
    exhaustive: every (i,j) with j-i>min_gap.
    endcap:     first band x last band only — targets the start<->end revisit a
                drifting scan cannot close. Spatial (seed-pose) proposal is blind
                to it because the drift pulls the true partners far apart in the
                stored poses, so we pair by frame INDEX position instead."""
    if mode == "endcap":
        b = max(1, int(n * band_frac))
        pairs = [(i, j) for i in range(b) for j in range(n - b, n) if j - i > min_gap]
    else:
        pairs = [(i, j) for i in range(n) for j in range(i + 1, n) if j - i > min_gap]
    if max_pairs and len(pairs) > max_pairs:
        sel = rng.choice(len(pairs), max_pairs, replace=False)
        pairs = [pairs[k] for k in sorted(sel)]
    return pairs


# --------------------------------------------------------------------------- #
# Main                                                                        #
# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(
        description="Export wide-baseline loop-closure edges for "
        "`rux optimize --loop-edges` (issue #221/#225 P2)."
    )
    ap.add_argument("project", help="path to the .rux project database")
    ap.add_argument("-o", "--output", required=True, help="output edge JSON")
    ap.add_argument(
        "--matcher",
        default="orb",
        choices=["orb", "xfeat", "lightglue", "mast3r", "mapanything"],
    )
    ap.add_argument(
        "--proposal", default="exhaustive", choices=["exhaustive", "endcap"]
    )
    ap.add_argument("--band-frac", type=float, default=0.15, help="endcap band size")
    ap.add_argument("--min-frame-gap", type=int, default=50)
    ap.add_argument("--min-inliers", type=int, default=40)
    ap.add_argument("--ransac-thresh", type=float, default=0.10, help="3D-3D (m)")
    ap.add_argument("--ransac-iters", type=int, default=500)
    ap.add_argument("--min-depth", type=float, default=0.3)
    ap.add_argument("--max-depth", type=float, default=5.0)
    ap.add_argument("--max-matches", type=int, default=4000)
    ap.add_argument("--stride", type=int, default=1, help="subsample frames")
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--max-pairs", type=int, default=None)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--device", default="cuda")
    ap.add_argument(
        "--allow-noncommercial",
        action="store_true",
        help="acknowledge that mast3r / mapanything-NC weights are research-only "
        "and their edges must not enter a commercial deliverable",
    )
    # matcher-specific extras (weights path, checkpoint variant, etc.)
    ap.add_argument("--weights", default=None)
    ap.add_argument("--variant", default=None)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    matcher = load_matcher(args)

    t0 = time.time()
    frames = read_frames(args.project, stride=args.stride, max_frames=args.max_frames)
    print(f"[read] {len(frames)} frames from {args.project} in {time.time()-t0:.1f}s")
    if len(frames) < 2:
        sys.exit("need >= 2 frames")

    pairs = propose_pairs(
        len(frames),
        args.min_frame_gap,
        args.max_pairs,
        rng,
        mode=args.proposal,
        band_frac=args.band_frac,
    )
    print(
        f"[propose] {len(pairs)} candidate pairs "
        f"(mode={args.proposal}, min_frame_gap={args.min_frame_gap})"
    )

    edges = []
    t1 = time.time()
    for k, (i, j) in enumerate(pairs):
        fi, fj = frames[i], frames[j]
        m = matcher.match(fi, fj, max_matches=args.max_matches)
        if m is None or len(m[0]) < 3:
            continue
        xy_i, xy_j = m  # matched pixel coords in frames i and j
        p_i, vi = backproject(xy_i, fi.depth_m, fi.K, args.min_depth, args.max_depth)
        p_j, vj = backproject(xy_j, fj.depth_m, fj.K, args.min_depth, args.max_depth)
        both = vi & vj
        if int(both.sum()) < 3:
            continue
        # p_i ~= T_ij @ p_j  =>  src=p_j, dst=p_i
        T_ij, inl = ransac_pose(
            p_j[both], p_i[both], args.ransac_thresh, args.ransac_iters, rng
        )
        if T_ij is None:
            continue
        ninl = int(inl.sum())
        if ninl < args.min_inliers:
            continue
        st, sr = edge_sigmas(ninl, args.min_inliers, 0.10, 0.05, 0.04, 0.02)
        edges.append(
            {
                "node_i": int(fi.node_id),
                "node_j": int(fj.node_id),
                "T_ij": [float(x) for x in T_ij.reshape(-1)],
                "sigma_rot": sr,
                "sigma_trans": st,
                "inliers": ninl,
            }
        )
        if (k + 1) % 200 == 0:
            print(
                f"[match] {k+1}/{len(pairs)} pairs, {len(edges)} edges, "
                f"{(k+1)/(time.time()-t1):.1f} pairs/s"
            )

    out = {
        "schema": SCHEMA,
        "producer": f"{args.matcher}",
        "project": str(args.project),
        "convention": "T_ij = pose(i)^-1 * pose(j), optical->world; node ids are "
        "sensor_frames.node_id",
        "params": {
            "matcher": args.matcher,
            "min_frame_gap": args.min_frame_gap,
            "min_inliers": args.min_inliers,
            "ransac_thresh_m": args.ransac_thresh,
            "stride": args.stride,
        },
        "edges": edges,
    }
    Path(args.output).write_text(json.dumps(out, indent=1))
    print(
        f"[done] {len(edges)} edges -> {args.output} "
        f"({time.time()-t0:.1f}s total, {len(pairs)} pairs)"
    )


if __name__ == "__main__":
    main()
