# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""
Point-to-point ICP for the manual-adjust mode of gt_curator.

Uses only numpy + scipy — no open3d dependency. Implements a simple ICP loop:
  1. Find nearest neighbours (cKDTree)
  2. Reject pairs beyond a distance threshold
  3. Solve for the rigid transform (Umeyama)
  4. Repeat until convergence or max_iter

The depth clouds for frames i and j are first trimmed to the valid-depth
region, then ICP registers j's cloud onto i's cloud.

Returns: T_refine (4x4), rms (float), overlap_frac (float)
"""

import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "loop_edges"))
from export_loop_edges import backproject  # noqa: E402

MIN_DEPTH = 0.3
MAX_DEPTH = 5.0
ICP_MAX_ITER = 50
ICP_TOL = 1e-5
ICP_MAX_CORR_M = 0.5   # initial correspondence distance cap (metres)
ICP_TRIM_FRAC = 0.80   # keep this fraction of closest pairs each iter


def _sample_cloud(frame, min_depth=MIN_DEPTH, max_depth=MAX_DEPTH, stride=4):
    """Return an Nx3 array of optical-frame 3D points sampled at stride."""
    h, w = frame.depth_m.shape[:2]
    uu, vv = np.meshgrid(np.arange(0, w, stride), np.arange(0, h, stride))
    pts_xy = np.stack([uu.ravel(), vv.ravel()], axis=1).astype(np.float64)
    pts3, valid = backproject(pts_xy, frame.depth_m, frame.K, min_depth, max_depth)
    return pts3[valid]


def _umeyama_rigid(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """Least-squares rigid T (4x4) with dst ≈ T @ src."""
    cs = src.mean(0)
    cd = dst.mean(0)
    H = (src - cs).T @ (dst - cd)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    t = cd - R @ cs
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def icp(
    src_cloud: np.ndarray,
    dst_cloud: np.ndarray,
    T_init: np.ndarray | None = None,
    max_iter: int = ICP_MAX_ITER,
    tol: float = ICP_TOL,
    max_corr_m: float = ICP_MAX_CORR_M,
    trim_frac: float = ICP_TRIM_FRAC,
) -> tuple[np.ndarray, float, float]:
    """Register src_cloud onto dst_cloud.

    T_init: initial 4x4 guess (identity if None).
    Returns (T_final 4x4, rms_m, overlap_fraction).
    """
    if T_init is None:
        T_init = np.eye(4)

    T = T_init.copy()
    best_T = T.copy()
    best_rms = float("inf")
    dst_tree = cKDTree(dst_cloud)

    def _eval(T_candidate):
        src_t = (src_cloud @ T_candidate[:3, :3].T) + T_candidate[:3, 3]
        d, ix = dst_tree.query(src_t, workers=-1)
        return src_t, d, ix

    prev_rms = float("inf")

    for _ in range(max_iter):
        src_t, dists, idx = _eval(T)

        # Trim: keep closest trim_frac pairs within max_corr_m
        valid = dists < max_corr_m
        if not valid.any():
            break
        dist_cap = float(np.percentile(dists[valid], trim_frac * 100))
        keep = valid & (dists <= dist_cap)
        if keep.sum() < 6:
            break

        rms = float(np.sqrt(np.mean(dists[keep] ** 2)))
        if rms < best_rms:
            best_rms = rms
            best_T = T.copy()

        if abs(prev_rms - rms) < tol:
            break
        prev_rms = rms

        # Umeyama step: refine T
        src_kept = src_cloud[keep]
        dst_kept = dst_cloud[idx[keep]]
        T_step = _umeyama_rigid(src_kept, dst_kept)
        T = T_step @ T

    # Use best T seen (guards against late divergence)
    _, dists_final, _ = _eval(best_T)
    in_corr = dists_final < max_corr_m
    rms = float(np.sqrt(np.mean(dists_final[in_corr] ** 2))) if in_corr.any() else float("inf")
    overlap = float(in_corr.mean())

    return best_T, rms, overlap


def run_icp_on_frames(
    db_path: str,
    node_i: int,
    node_j: int,
    T_init: np.ndarray | None = None,
    stride: int = 4,
) -> dict:
    """Load frames from DB and run ICP of frame j's cloud onto frame i's cloud.

    T_init: coarse initial transform (from sliders or a previous match).

    Returns dict with T_ij (4x4), rms (m), overlap, error.
    """
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "loop_edges"))
    from export_loop_edges import read_frames

    frames = read_frames(db_path)
    by_id = {f.node_id: f for f in frames}
    fi = by_id.get(node_i)
    fj = by_id.get(node_j)

    if fi is None or fj is None:
        missing = node_i if fi is None else node_j
        return {"error": f"frame {missing} not found", "T_ij": None, "rms": None, "overlap": None}

    cloud_i = _sample_cloud(fi, stride=stride)
    cloud_j = _sample_cloud(fj, stride=stride)

    if len(cloud_i) < 20 or len(cloud_j) < 20:
        return {"error": "too few valid depth points", "T_ij": None, "rms": None, "overlap": None}

    T_ij, rms, overlap = icp(cloud_j, cloud_i, T_init=T_init)

    return {
        "error": None,
        "T_ij": T_ij.tolist(),
        "rms": rms,
        "overlap": overlap,
    }


def sliders_to_T(dx: float, dy: float, dz: float, dyaw_deg: float) -> np.ndarray:
    """Build a 4x4 transform from x/y/z translation + yaw (gravity-locked).

    The yaw rotation is around the Z axis (gravity-aligned up = +Z in the
    optical frame convention used here — matches the world-up assumption).
    """
    yaw = np.deg2rad(dyaw_deg)
    cy, sy = np.cos(yaw), np.sin(yaw)
    R = np.array([[cy, -sy, 0.0],
                  [sy,  cy, 0.0],
                  [0.0, 0.0, 1.0]])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [dx, dy, dz]
    return T
