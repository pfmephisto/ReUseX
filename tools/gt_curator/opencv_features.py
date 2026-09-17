# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""
In-process OpenCV feature matchers (ORB, SIFT, AKAZE) for the GT curator.

Each matcher returns:
  (xy_i, xy_j, inlier_mask, T_ij, rms, n_inliers)

where T_ij follows the loop_edges convention:
  T_ij maps a 3D point from frame j's optical frame into frame i's optical frame
  <=> T_ij = pose(i)^-1 @ pose(j)   (world-from-camera convention)

This is verified in tests/test_convention.py.
"""

import sys
from pathlib import Path
from typing import Literal

import cv2
import numpy as np

# Re-use backproject and ransac_pose from export_loop_edges to keep behaviour
# identical across the in-process and subprocess paths.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "loop_edges"))
from export_loop_edges import backproject, ransac_pose  # noqa: E402

RANSAC_THRESH = 0.10   # metres, same default as export_loop_edges
RANSAC_ITERS = 500
MIN_DEPTH = 0.3
MAX_DEPTH = 5.0
RATIO = 0.85
SEED = 42


def _load_frame(db_path: str, node_id: int):
    """Load a Frame object from the .rux database, reusing export_loop_edges.read_frames."""
    from export_loop_edges import read_frames  # noqa: F401
    frames = read_frames(db_path)
    by_id = {f.node_id: f for f in frames}
    return by_id.get(node_id)


# --------------------------------------------------------------------------- #
# Feature extraction helpers                                                   #
# --------------------------------------------------------------------------- #

def _orb_match(fi, fj, nfeatures: int = 3000, ratio: float = RATIO):
    orb = cv2.ORB_create(nfeatures=nfeatures)
    ki, di = orb.detectAndCompute(fi.gray, None)
    kj, dj = orb.detectAndCompute(fj.gray, None)
    if di is None or dj is None or len(ki) < 4 or len(kj) < 4:
        return None, None
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    knn = bf.knnMatch(di, dj, k=2)
    good = [m for pair in knn if len(pair) == 2 for m in [pair[0]]
            if m.distance < ratio * pair[1].distance]
    if len(good) < 3:
        return None, None
    xy_i = np.array([ki[m.queryIdx].pt for m in good], np.float64)
    xy_j = np.array([kj[m.trainIdx].pt for m in good], np.float64)
    return xy_i, xy_j


def _sift_match(fi, fj, ratio: float = RATIO):
    sift = cv2.SIFT_create()
    ki, di = sift.detectAndCompute(fi.gray, None)
    kj, dj = sift.detectAndCompute(fj.gray, None)
    if di is None or dj is None or len(ki) < 4 or len(kj) < 4:
        return None, None
    bf = cv2.BFMatcher(cv2.NORM_L2)
    knn = bf.knnMatch(di, dj, k=2)
    good = [m for pair in knn if len(pair) == 2 for m in [pair[0]]
            if m.distance < ratio * pair[1].distance]
    if len(good) < 3:
        return None, None
    xy_i = np.array([ki[m.queryIdx].pt for m in good], np.float64)
    xy_j = np.array([kj[m.trainIdx].pt for m in good], np.float64)
    return xy_i, xy_j


def _akaze_match(fi, fj, ratio: float = RATIO):
    akaze = cv2.AKAZE_create()
    ki, di = akaze.detectAndCompute(fi.gray, None)
    kj, dj = akaze.detectAndCompute(fj.gray, None)
    if di is None or dj is None or len(ki) < 4 or len(kj) < 4:
        return None, None
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    knn = bf.knnMatch(di, dj, k=2)
    good = [m for pair in knn if len(pair) == 2 for m in [pair[0]]
            if m.distance < ratio * pair[1].distance]
    if len(good) < 3:
        return None, None
    xy_i = np.array([ki[m.queryIdx].pt for m in good], np.float64)
    xy_j = np.array([kj[m.trainIdx].pt for m in good], np.float64)
    return xy_i, xy_j


# --------------------------------------------------------------------------- #
# Main entry point                                                              #
# --------------------------------------------------------------------------- #

def match_pair(
    db_path: str,
    node_i: int,
    node_j: int,
    method: Literal["orb", "sift", "akaze"] = "orb",
    ransac_thresh: float = RANSAC_THRESH,
    ransac_iters: int = RANSAC_ITERS,
    min_depth: float = MIN_DEPTH,
    max_depth: float = MAX_DEPTH,
    seed: int = SEED,
) -> dict:
    """Match a pair of frames with an OpenCV detector.

    Returns a dict with keys:
      xy_i, xy_j       – all matched pixel coords (Nx2)
      inlier_mask      – bool array of RANSAC inliers (N,)
      T_ij             – 4x4 rigid transform (j→i optical frame), None on failure
      n_inliers        – int
      rms              – float (metres), None on failure
      error            – str or None
    """
    frames = _read_frames_by_id(db_path)
    fi = frames.get(node_i)
    fj = frames.get(node_j)
    if fi is None or fj is None:
        return {"error": f"frame not found: {node_i if fi is None else node_j}",
                "T_ij": None, "n_inliers": 0, "rms": None,
                "xy_i": [], "xy_j": [], "inlier_mask": []}

    if method == "orb":
        xy_i, xy_j = _orb_match(fi, fj)
    elif method == "sift":
        xy_i, xy_j = _sift_match(fi, fj)
    elif method == "akaze":
        xy_i, xy_j = _akaze_match(fi, fj)
    else:
        return {"error": f"unknown method: {method}", "T_ij": None, "n_inliers": 0,
                "rms": None, "xy_i": [], "xy_j": [], "inlier_mask": []}

    if xy_i is None:
        return {"error": "no matches found", "T_ij": None, "n_inliers": 0,
                "rms": None, "xy_i": [], "xy_j": [], "inlier_mask": []}

    # Lift 2D matches → 3D optical-frame points
    p_i, vi = backproject(xy_i, fi.depth_m, fi.K, min_depth, max_depth)
    p_j, vj = backproject(xy_j, fj.depth_m, fj.K, min_depth, max_depth)
    both = vi & vj

    if int(both.sum()) < 3:
        return {"error": "too few valid depth pixels", "T_ij": None, "n_inliers": 0,
                "rms": None, "xy_i": xy_i.tolist(), "xy_j": xy_j.tolist(),
                "inlier_mask": [False] * len(xy_i)}

    rng = np.random.default_rng(seed)
    # Convention: T_ij maps p_j → p_i, i.e. p_i ≈ T_ij @ p_j
    # ransac_pose(src, dst) returns T such that dst ≈ T @ src
    # => src = p_j[both], dst = p_i[both]
    T_ij, inl = ransac_pose(p_j[both], p_i[both], ransac_thresh, ransac_iters, rng)

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

    return {
        "error": None,
        "xy_i": xy_i.tolist(),
        "xy_j": xy_j.tolist(),
        "inlier_mask": inlier_mask.tolist(),
        "T_ij": T_ij.tolist() if T_ij is not None else None,
        "n_inliers": n_inliers,
        "rms": rms,
    }


# Module-level cache so repeated calls on the same DB don't re-read all frames.
_frame_cache: dict[str, dict] = {}


def _read_frames_by_id(db_path: str) -> dict:
    global _frame_cache
    if db_path not in _frame_cache:
        from export_loop_edges import read_frames
        frames = read_frames(db_path)
        _frame_cache[db_path] = {f.node_id: f for f in frames}
    return _frame_cache[db_path]


def clear_cache():
    global _frame_cache
    _frame_cache = {}
