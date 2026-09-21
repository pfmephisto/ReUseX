#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""
CLI wrapper for solve_gt_poses GTSAM backend — generalised for gt_curator.

Generalises the original one-off solve_gt_poses.py by accepting:
  --edges <json>  — anchor edges JSON in reusex.gt_anchors.v1 schema
  --out   <rux>   — output .rux file (copy of seed with GT poses written)

The T_ij convention is identical to solve_gt_poses.py:
  T_ij = pose(i)^-1 * pose(j)  (optical→world convention)
  pose(j)_pred = pose(i) @ T_ij

This script runs in ~/gt-curator-venv (which has gtsam 4.2.2 + scipy 1.14 +
numpy 2.x). The app shells out to it via subprocess using SOLVE_VENV defined
in app.py.  Do NOT run it from the mast3r or xfeat venvs.

Usage:
  LD_LIBRARY_PATH=<gcc-lib> \
  ~/gt-curator-venv/bin/python solve_gt_poses_cli.py \
    --seed <rux> --edges <json> --out <rux> [--anchor-max-idx N]
"""

import argparse
import json
import math
import shutil
import sqlite3
import struct
import sys
import time
from pathlib import Path

import gtsam
import numpy as np
from scipy.spatial.transform import Rotation


# --------------------------------------------------------------------------- #
# Helpers (identical to solve_gt_poses.py)                                     #
# --------------------------------------------------------------------------- #

def decode_pose(blob: bytes) -> np.ndarray:
    return np.array(struct.unpack("16d", blob)).reshape(4, 4)


def encode_pose(mat: np.ndarray) -> bytes:
    return struct.pack("16d", *mat.flatten().tolist())


def mat_to_pose3(mat: np.ndarray) -> gtsam.Pose3:
    R = mat[:3, :3]
    t = mat[:3, 3]
    q = Rotation.from_matrix(R).as_quat()  # xyzw
    w, x, y, z = float(q[3]), float(q[0]), float(q[1]), float(q[2])
    rot = gtsam.Rot3(w, x, y, z)
    tv = np.array([float(t[0]), float(t[1]), float(t[2])], dtype=np.float64)
    return gtsam.Pose3(rot, tv)


def pose3_to_mat(p: gtsam.Pose3) -> np.ndarray:
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = p.rotation().matrix()
    mat[:3, 3] = np.asarray(p.translation(), dtype=np.float64)
    return mat


def diag_noise6(sigmas: list) -> gtsam.noiseModel.Diagonal:
    sv = np.array([float(s) for s in sigmas], dtype=np.float64)
    return gtsam.noiseModel.Diagonal.Sigmas(sv)


def load_seed_poses(db_path: Path):
    conn = sqlite3.connect(str(db_path))
    cur = conn.cursor()
    cur.execute("SELECT node_id, transform FROM sensor_frames ORDER BY node_id ASC")
    rows = cur.fetchall()
    conn.close()
    poses = {}
    ids = []
    for nid, blob in rows:
        if blob and len(blob) == 128:
            poses[nid] = decode_pose(blob)
            ids.append(nid)
    return poses, ids


# --------------------------------------------------------------------------- #
# Main                                                                          #
# --------------------------------------------------------------------------- #

def main():
    ap = argparse.ArgumentParser(
        description="GTSAM GT-pose solver driven by gt_curator anchor edges"
    )
    ap.add_argument("--seed", required=True, help="seed .rux (read-only)")
    ap.add_argument("--edges", required=True, help="anchor edges JSON")
    ap.add_argument("--out", required=True, help="output .rux with GT poses")
    ap.add_argument(
        "--anchor-max-idx", type=int, default=None,
        help="last sequential index (0-based) to treat as a tight prior anchor; "
             "if omitted, no PriorFactors are added (loop edges alone constrain "
             "the graph, which may leave gauge freedom)"
    )
    # Noise hyperparams
    ap.add_argument("--odom-sigma-t", type=float, default=0.10)
    ap.add_argument("--odom-sigma-r", type=float, default=0.05)
    ap.add_argument("--anchor-sigma-t", type=float, default=0.02)
    ap.add_argument("--anchor-sigma-r", type=float, default=0.01)
    ap.add_argument("--huber-k", type=float, default=1.345)
    ap.add_argument("--max-iters", type=int, default=300)
    args = ap.parse_args()

    seed_path = Path(args.seed)
    out_path = Path(args.out)
    edges_doc = json.loads(Path(args.edges).read_text())
    loop_edges = edges_doc.get("edges", [])

    print(f"[solve] seed={seed_path.name}, edges={len(loop_edges)}, out={out_path.name}")

    seed_poses, node_ids_sorted = load_seed_poses(seed_path)
    N = len(node_ids_sorted)
    print(f"[solve] {N} frames loaded from seed")

    # Anchor set (tight priors)
    anchor_nids = set()
    if args.anchor_max_idx is not None:
        k = min(args.anchor_max_idx + 1, N)
        anchor_nids = set(node_ids_sorted[:k])
        print(f"[solve] anchor prior zone: {len(anchor_nids)} frames "
              f"(nids {node_ids_sorted[0]}..{node_ids_sorted[k-1]})")

    # Build factor graph
    graph = gtsam.NonlinearFactorGraph()
    values = gtsam.Values()
    for nid, mat in seed_poses.items():
        values.insert(nid, mat_to_pose3(mat))

    # Odometry (consecutive seed relatives, loose)
    odom_noise = diag_noise6([
        args.odom_sigma_r, args.odom_sigma_r, args.odom_sigma_r,
        args.odom_sigma_t, args.odom_sigma_t, args.odom_sigma_t,
    ])
    n_odom = 0
    for i in range(N - 1):
        ni, nj = node_ids_sorted[i], node_ids_sorted[i + 1]
        T_ij = np.linalg.inv(seed_poses[ni]) @ seed_poses[nj]
        graph.add(gtsam.BetweenFactorPose3(ni, nj, mat_to_pose3(T_ij), odom_noise))
        n_odom += 1
    print(f"[solve] {n_odom} odometry BetweenFactors")

    # Anchor priors (tight)
    if anchor_nids:
        anchor_noise = diag_noise6([
            args.anchor_sigma_r, args.anchor_sigma_r, args.anchor_sigma_r,
            args.anchor_sigma_t, args.anchor_sigma_t, args.anchor_sigma_t,
        ])
        for nid in anchor_nids:
            graph.add(gtsam.PriorFactorPose3(nid, mat_to_pose3(seed_poses[nid]), anchor_noise))
        print(f"[solve] {len(anchor_nids)} anchor PriorFactors "
              f"(sigma_t={args.anchor_sigma_t}m)")
    else:
        # No anchor zone — add a single prior on the first frame to fix gauge
        nid0 = node_ids_sorted[0]
        fix_noise = diag_noise6([1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6])
        graph.add(gtsam.PriorFactorPose3(nid0, mat_to_pose3(seed_poses[nid0]), fix_noise))
        print("[solve] no anchor zone — fixing frame 0 for gauge")

    # Loop closure anchors (Huber-robust)
    n_loop = 0
    n_skip = 0
    for e in loop_edges:
        ni, nj = e["node_i"], e["node_j"]
        if ni not in seed_poses or nj not in seed_poses:
            n_skip += 1
            continue
        sig_t = float(e.get("sigma_trans", 0.05))
        sig_r = float(e.get("sigma_rot", 0.02))
        base_n = diag_noise6([sig_r, sig_r, sig_r, sig_t, sig_t, sig_t])
        rob_n = gtsam.noiseModel.Robust.Create(
            gtsam.noiseModel.mEstimator.Huber.Create(args.huber_k), base_n
        )
        T_ij = np.array(e["T_ij"]).reshape(4, 4)
        graph.add(gtsam.BetweenFactorPose3(ni, nj, mat_to_pose3(T_ij), rob_n))
        n_loop += 1

    print(f"[solve] {n_loop} loop BetweenFactors (skipped {n_skip})")

    # Solve
    params = gtsam.LevenbergMarquardtParams()
    params.setMaxIterations(args.max_iters)
    params.setRelativeErrorTol(1e-8)
    params.setAbsoluteErrorTol(1e-8)
    params.setVerbosity("SILENT")

    print("[solve] running Levenberg-Marquardt …")
    t0 = time.time()
    opt = gtsam.LevenbergMarquardtOptimizer(graph, values, params)
    result = opt.optimize()
    elapsed = time.time() - t0

    init_err = graph.error(values)
    final_err = graph.error(result)
    print(f"[solve] elapsed={elapsed:.1f}s  error {init_err:.4f} → {final_err:.4f} "
          f"(ratio {final_err/init_err:.5f})")

    # Collect GT poses
    gt_poses = {}
    for nid in seed_poses:
        try:
            gt_poses[nid] = pose3_to_mat(result.atPose3(nid))
        except Exception:
            gt_poses[nid] = seed_poses[nid]

    # Pose shift stats
    shifts = np.array([
        np.linalg.norm(gt_poses[nid][:3, 3] - seed_poses[nid][:3, 3])
        for nid in node_ids_sorted
    ])
    print(f"[solve] pose shifts — max={shifts.max():.4f}m, mean={shifts.mean():.4f}m, "
          f"p95={np.percentile(shifts, 95):.4f}m")

    # Write output
    out_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(str(seed_path), str(out_path))
    conn = sqlite3.connect(str(out_path))
    cur = conn.cursor()
    for nid, mat in gt_poses.items():
        cur.execute("UPDATE sensor_frames SET transform=? WHERE node_id=?",
                    (encode_pose(mat), nid))
    conn.commit()
    conn.close()
    print(f"[solve] wrote {len(gt_poses)} poses to {out_path}")

    # Summary JSON to stdout for the app to parse
    summary = {
        "n_frames": N,
        "n_odom": n_odom,
        "n_anchors": len(anchor_nids),
        "n_loop": n_loop,
        "init_err": init_err,
        "final_err": final_err,
        "elapsed_s": elapsed,
        "max_shift_m": float(shifts.max()),
        "mean_shift_m": float(shifts.mean()),
        "p95_shift_m": float(np.percentile(shifts, 95)),
        "out_rux": str(out_path),
    }
    print("SOLVE_SUMMARY:" + json.dumps(summary))


if __name__ == "__main__":
    main()
