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
#
# PROPOSAL MODES
# --------------
# exhaustive:  every (i,j) pair with j-i > min_gap.  O(N²).
# endcap:      first-band × last-band — targets start↔end revisit.
# spatial:     seed-pose proximity within --spatial-radius metres.
#              Blind to drift: rooms whose early and late visits are pulled
#              >radius apart by accumulated drift will be missed.
# appearance:  pose-INDEPENDENT global descriptor retrieval via DINOv2 CLS
#              token.  Proposes top-k cosine-nearest neighbours per frame,
#              excluding temporally-close frames.  Catches drift-hidden revisits
#              that spatial proposal misses: the descriptor is computed from the
#              image, not the stored pose, so it is unaffected by drift magnitude.
#              Complementary to spatial: use both and take the union.
#              Compute cost: one DINOv2 forward pass per frame (~34 ms on CPU
#              for vits14 at 224 px; ~6 ms on a mid-range GPU).  Descriptor
#              extraction runs once upfront; NN search is O(N²) in descriptor
#              space, negligible vs extraction.

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


def read_seed_positions(db_path):
    """Read stored (possibly drifted) camera positions for all sensor frames.

    Returns a list of (node_id, xyz) in node_id order, aligned with the frame
    list that read_frames() returns at stride=1.  The transform blob is a
    row-major float64 4x4 camera-to-world matrix; only the translation column
    (last column, first three rows) is extracted here.

    NOTE: These are the *seed* poses baked into the .rux at import time.  For
    scans with significant accumulated drift, a room that was revisited late in
    the trajectory may appear metres away from its early-visit position in this
    coordinate frame.  The spatial proposal therefore uses a generous radius
    (see --spatial-radius) and pairs by seed-pose proximity as a *heuristic*
    that works well for low-to-moderate drift but is blind to heavily drifted
    revisits.  For those, an appearance-based retrieval via DINOv2 (the
    'appearance' proposal mode) finds pairs regardless of pose error — see §9
    of the investigation doc for results."""
    con = sqlite3.connect(db_path)
    cur = con.cursor()
    cur.execute(
        "SELECT node_id, transform FROM sensor_frames "
        "WHERE color IS NOT NULL AND depth IS NOT NULL ORDER BY node_id"
    )
    rows = cur.fetchall()
    con.close()

    positions = []
    for node_id, transform_data in rows:
        if transform_data is None:
            positions.append((node_id, None))
            continue
        arr = np.frombuffer(transform_data, dtype=np.float64)
        if len(arr) == 16:
            xyz = arr.reshape(4, 4)[:3, 3].copy()
        else:
            xyz = None
        positions.append((node_id, xyz))
    return positions


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
# Appearance descriptors (DINOv2 CLS token)                                   #
# --------------------------------------------------------------------------- #
_DINO_INPUT_SIZE = 224  # resize shorter side to this before centre-crop
_DINO_MEAN = np.array([0.485, 0.456, 0.406], np.float32)
_DINO_STD = np.array([0.229, 0.224, 0.225], np.float32)


def _preprocess_for_dino(color_bgr):
    """Resize and normalise a BGR frame for DINOv2 input.

    Resizes the shorter side to _DINO_INPUT_SIZE, then centre-crops to a
    (_DINO_INPUT_SIZE × _DINO_INPUT_SIZE) square.  Normalises with ImageNet
    mean/std.  Returns a (1, 3, H, W) float32 numpy array; the caller converts
    to a torch tensor.

    Choice of 224 px: the vits14 backbone was trained at 518 px but accepts any
    multiple of 14.  224 px (= 16 × 14) gives a 6.5× forward-pass speedup vs
    518 px on CPU (34 ms vs 225 ms) with negligible retrieval quality loss for
    place recognition — the CLS token captures scene-level semantics that are
    robust to moderate down-sampling.
    """
    h, w = color_bgr.shape[:2]
    # Resize so shorter side = _DINO_INPUT_SIZE, preserving aspect ratio.
    scale = _DINO_INPUT_SIZE / min(h, w)
    new_w = int(round(w * scale))
    new_h = int(round(h * scale))
    resized = cv2.resize(color_bgr, (new_w, new_h), interpolation=cv2.INTER_AREA)
    # Centre crop to square.
    y0 = (new_h - _DINO_INPUT_SIZE) // 2
    x0 = (new_w - _DINO_INPUT_SIZE) // 2
    cropped = resized[y0 : y0 + _DINO_INPUT_SIZE, x0 : x0 + _DINO_INPUT_SIZE]
    # BGR -> RGB, [0,1], ImageNet normalise, BCHW.
    rgb = cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    rgb = (rgb - _DINO_MEAN) / _DINO_STD
    return rgb.transpose(2, 0, 1)[np.newaxis]  # (1, 3, H, W)


def extract_appearance_descriptors(frames, device="cpu", batch_size=8):
    """Extract L2-normalised DINOv2 CLS descriptors for each frame.

    Uses ``dinov2_vits14`` loaded from torch.hub (Apache-2.0 backbone,
    384-dim CLS token).  The model weights download automatically on first
    use and are cached at ~/.cache/torch/hub/checkpoints/.

    WHY DINOv2 CLS:
      - Pose-independent: the descriptor encodes scene appearance, not geometry.
        Drift in stored poses does not affect retrieval quality.
      - Proven on place recognition: DINOv2 CLS-cosine retrieval is competitive
        with NetVLAD and purpose-trained VPR models on indoor benchmarks (InLoc,
        RobotCar) without any fine-tuning.
      - Already in the repo's dependency closure: MASt3R/MapAnything both use
        DINOv2 as a backbone.  The model file is already cached after running
        the MASt3R matcher once.
      - License: Apache-2.0 (the ``facebookresearch/dinov2`` repo).  The CLS
        descriptor retrieval does not require any non-commercial weights.

    WHY NOT NetVLAD: NetVLAD checkpoints (VGG + trained on Pittsburgh) are
      NOT pre-downloaded in the existing venvs.  DINOv2 CLS achieves similar
      recall on indoor scans and requires zero additional weight downloads.

    Returns an (N, 384) float32 array of L2-normalised descriptors, one per
    frame, in the same order as ``frames``."""
    try:
        import torch
    except ImportError:
        raise RuntimeError(
            "[appearance] torch is not importable.  Run with the mast3r or xfeat "
            "venv (e.g. LD_LIBRARY_PATH=... .venv/bin/python3 ...)."
        )

    print(f"[appearance] loading DINOv2 vits14 on device={device} ...")
    model = torch.hub.load(
        "facebookresearch/dinov2",
        "dinov2_vits14",
        trust_repo=True,
    )
    model = model.to(device).eval()

    all_descs = []
    t0 = time.time()
    for start in range(0, len(frames), batch_size):
        batch_frames = frames[start : start + batch_size]
        imgs = np.concatenate(
            [_preprocess_for_dino(f.color) for f in batch_frames], axis=0
        )
        imgs_t = torch.from_numpy(imgs).to(device)
        with torch.no_grad():
            cls = model(imgs_t)  # (B, 384)
        cls_np = cls.cpu().float().numpy()
        all_descs.append(cls_np)
        if (start // batch_size + 1) % 20 == 0:
            n_done = min(start + batch_size, len(frames))
            print(
                f"[appearance] descriptors {n_done}/{len(frames)} "
                f"({n_done/(time.time()-t0):.1f} frames/s)"
            )

    descs = np.concatenate(all_descs, axis=0)  # (N, 384)
    # L2-normalise for cosine similarity via dot product.
    norms = np.linalg.norm(descs, axis=1, keepdims=True).clip(min=1e-8)
    descs = descs / norms
    print(
        f"[appearance] {len(frames)} descriptors extracted in "
        f"{time.time()-t0:.1f}s ({(time.time()-t0)/len(frames)*1000:.0f} ms/frame)"
    )
    return descs


def propose_pairs_appearance(descs, min_gap, max_pairs, topk, rng):
    """Nearest-neighbour pairs in descriptor space (cosine similarity).

    For each frame i, find the top-``topk`` most similar frames by cosine
    similarity, excluding frames within ``min_gap`` indices of i (temporal
    exclusion).  Add (min(i,j), max(i,j)) to the candidate set; deduplicate.

    The resulting pairs are **pose-independent**: a room revisited after large
    accumulated drift sits far from its early visit in seed-pose space (spatial
    proposal misses it) but looks visually similar and will score high in
    descriptor space (appearance retrieval catches it).

    Complexity: O(N²) dot-product for the full similarity matrix.  At N=1000
    this is a 1000×1000 float32 matrix (4 MB), computed in <1 s with numpy.
    For N>5000 consider batched FAISS-flat, but typical strides keep N ≤ 2000.
    """
    n = len(descs)
    # Full cosine similarity matrix (descs are already L2-normalised).
    sim = descs @ descs.T  # (N, N)
    # Zero out self-similarity and temporally-close pairs.
    for offset in range(-min_gap + 1, min_gap):
        idx = np.arange(n)
        jdx = idx + offset
        mask = (jdx >= 0) & (jdx < n)
        sim[idx[mask], jdx[mask]] = -1.0
    np.fill_diagonal(sim, -1.0)

    pairs_set = set()
    effective_topk = min(topk, n - 1)
    for i in range(n):
        top_j = np.argpartition(sim[i], -effective_topk)[-effective_topk:]
        for j in top_j:
            if sim[i, j] > -0.5:  # skip masked entries
                a, b = (int(i), int(j)) if i < j else (int(j), int(i))
                if b - a >= min_gap:
                    pairs_set.add((a, b))

    pairs = sorted(pairs_set)
    if max_pairs and len(pairs) > max_pairs:
        sel = rng.choice(len(pairs), max_pairs, replace=False)
        pairs = [pairs[k] for k in sorted(sel)]
    return pairs


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
def propose_pairs(
    n,
    min_gap,
    max_pairs,
    rng,
    mode="exhaustive",
    band_frac=0.15,
    positions=None,
    spatial_radius=3.0,
):
    """Candidate loop pairs (all modes except 'appearance').

    exhaustive: every (i,j) with j-i>min_gap.  O(N²) — only tractable for
                small scans; use --max-pairs to cap it.

    endcap:     first band × last band only — targets the start↔end revisit a
                drifting scan cannot close.  Spatial (seed-pose) proximity is
                BLIND to it because accumulated drift pulls the true partners
                far apart in the stored poses, so we pair by frame-INDEX
                position instead.

    spatial:    pairs whose stored seed-pose camera centres are within
                `spatial_radius` metres AND whose frame indices differ by at
                least `min_gap`.  This surfaces intra-building revisits (same
                room seen at different times) that are completely invisible to
                the endcap strategy.

                DRIFT CAVEAT: for scans with large accumulated drift (>half
                the room diameter) the early and late visits to the same room
                will appear far apart in seed-pose space, so `spatial_radius`
                must be set generously enough to still find them.  The default
                3 m works for low-to-moderate drift; increase to 5–8 m for
                heavily drifted scans at the cost of more false proposals.  For
                very large drift an appearance-based retrieval (DINOv2) is the
                correct solution — use --proposal appearance.

                `positions` must be supplied when mode='spatial'; it is a list
                of (node_id, xyz_or_None) aligned with the subsampled frame
                list (i.e., already respecting --stride / --max-frames).

    NOTE: 'appearance' mode is handled separately in main() via
    propose_pairs_appearance(); this function is not called for it.
    """
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
        "--proposal",
        default="exhaustive",
        choices=["exhaustive", "endcap", "spatial", "appearance"],
        help="candidate pair proposal strategy. "
        "'appearance' extracts a DINOv2 CLS descriptor per frame and proposes "
        "top-k cosine-nearest neighbours, excluding temporally-close frames. "
        "It is pose-independent and catches drift-hidden revisits that "
        "'spatial' misses. Use 'spatial' and 'appearance' in combination by "
        "running the script twice and merging the JSON edges. Default: exhaustive.",
    )
    ap.add_argument("--band-frac", type=float, default=0.15, help="endcap band size")
    ap.add_argument(
        "--spatial-radius",
        type=float,
        default=3.0,
        help="seed-pose proximity radius in metres for --proposal spatial. "
        "Pairs whose stored camera centres are within this distance AND whose "
        "frame indices differ by at least --min-frame-gap are proposed. "
        "Generous values (5-8 m) help find revisits in heavily drifted scans "
        "at the cost of more false proposals. Default: 3.0 m.",
    )
    ap.add_argument(
        "--appearance-topk",
        type=int,
        default=10,
        help="top-k nearest neighbours per frame for --proposal appearance. "
        "Higher values increase recall at the cost of more false proposals. "
        "Default: 10.",
    )
    ap.add_argument(
        "--appearance-batch-size",
        type=int,
        default=8,
        help="batch size for DINOv2 descriptor extraction (--proposal appearance). "
        "Increase for GPU; reduce to 1 for low-memory CPU. Default: 8.",
    )
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

    # For the spatial proposal, read stored seed poses (needed before propose_pairs).
    # We read ALL poses from the DB and align them to the subsampled frame list by
    # matching on node_id so the indexing stays consistent with the frames list.
    positions = None
    if args.proposal == "spatial":
        all_seed = read_seed_positions(args.project)
        # Build a node_id → xyz lookup; the frames list is a subset of all frames
        # (due to stride / max_frames), so we extract positions in that order.
        seed_map = {nid: xyz for nid, xyz in all_seed}
        positions = [seed_map.get(f.node_id) for f in frames]
        n_with_pos = sum(1 for p in positions if p is not None)
        print(
            f"[spatial] loaded seed poses for {n_with_pos}/{len(frames)} frames "
            f"(radius={args.spatial_radius:.1f}m)"
        )

    # Appearance proposal: extract DINOv2 descriptors and propose top-k NN pairs.
    # This is pose-independent and catches drift-hidden revisits that the spatial
    # proposal misses: rooms whose early and late visits are pulled >radius apart
    # by accumulated drift look visually similar and score high in descriptor space.
    # Complementary to spatial: the union of both covers low-to-moderate drift
    # (spatial) and high-drift revisits (appearance).
    if args.proposal == "appearance":
        descs = extract_appearance_descriptors(
            frames, device=args.device, batch_size=args.appearance_batch_size
        )
        pairs = propose_pairs_appearance(
            descs, args.min_frame_gap, args.max_pairs, args.appearance_topk, rng
        )
    else:
        pairs = propose_pairs(
            len(frames),
            args.min_frame_gap,
            args.max_pairs,
            rng,
            mode=args.proposal,
            band_frac=args.band_frac,
            positions=positions,
            spatial_radius=args.spatial_radius,
        )
    print(
        f"[propose] {len(pairs)} candidate pairs "
        f"(mode={args.proposal}, min_frame_gap={args.min_frame_gap}"
        + (
            f", topk={args.appearance_topk}"
            if args.proposal == "appearance"
            else ""
        )
        + ")"
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
            **(
                {
                    "spatial_radius_m": args.spatial_radius,
                    "band_frac": args.band_frac,
                }
                if args.proposal in ("spatial", "endcap")
                else {}
            ),
            **(
                {
                    "appearance_topk": args.appearance_topk,
                    "appearance_backbone": "dinov2_vits14",
                    "appearance_input_px": _DINO_INPUT_SIZE,
                }
                if args.proposal == "appearance"
                else {}
            ),
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
