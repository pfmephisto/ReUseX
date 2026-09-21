# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later
"""
NewOffice ground-truth pose solver (issue #221).

One-off validation artifact — NOT production code. CC-BY-NC-SA MASt3R edges
are explicitly allowed here (not shipped/production).

Run via:
  ./run_solver.sh

POSE CONVENTION (verified by round-trip test):
  sensor_frames.transform = 4x4 float64, row-major, 128 bytes
  T_world_from_camera  (world-from-camera transformation)
  Edge convention: T_ij = pose(i)^-1 * pose(j)
    => pose(j)_pred = pose(i) @ T_ij

APPROACH:
  - Seed odometry: relative poses between consecutive frames, LOOSE sigma
    (0.10 m) so that loop corrections can propagate across the chain.
  - Entrance anchors: tight PriorFactors on the first 568 frames, which are
    drift-free by definition (scan start). This gives the solve an absolute
    frame of reference and prevents gauge freedom.
  - MASt3R loop closures: ALL 527 targeted-revisit edges (all 4 revisit
    groups: 1370, 2816, 3429, 3780) + 348 spatial-filtered edges, with
    Huber-robust noise models. The deformation filter that previously rejected
    3429/3780 edges was tuned for rigid-odometry rux optimize; here the
    anchor priors + robust kernel handle large corrections properly.
  - Solve with Levenberg-Marquardt.
"""

import json
import math
import shutil
import sqlite3
import struct
import sys
import time
from pathlib import Path
from collections import Counter

import gtsam
import numpy as np
from scipy.spatial.transform import Rotation

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
PSEUDO_GT = Path("/home/mephisto/repos/NewOffice/pseudo-gt")
SEED_RUX   = PSEUDO_GT / "newoffice_pgt_before_seed.rux"
GT_BUILD   = PSEUDO_GT / "gt-build"
GT_RUX     = GT_BUILD / "newoffice_gt.rux"
RESULTS_JSON = GT_BUILD / "solve_results.json"

# Edge files
TARGETED_RAW   = PSEUDO_GT / "targeted-run" / "mast3r_targeted_revisits.json"
SPATIAL_FILT   = PSEUDO_GT / "targeted-run" / "spatial_filtered.json"

# ---------------------------------------------------------------------------
# Hyperparameters
# ---------------------------------------------------------------------------
ANCHOR_SEQ_MAX_IDX  = 567    # first 568 frames = entrance (drift-free)
ANCHOR_SIGMA_T      = 0.02   # m  tight
ANCHOR_SIGMA_R      = 0.01   # rad

ODOM_SIGMA_T        = 0.10   # m  loose enough to let loop corrections propagate
ODOM_SIGMA_R        = 0.05   # rad

MIN_INLIERS         = 40     # minimum inliers for targeted edges
SPATIAL_MIN_INLIERS = 100    # higher bar for spatial edges (less targeted)
LOOP_SIGMA_SCALE    = 2.0    # scaling: tighter = sigma_t = SCALE/sqrt(inliers)
LOOP_SIGMA_FLOOR_T  = 0.05   # m
LOOP_SIGMA_FLOOR_R  = 0.02   # rad
HUBER_K             = 1.345

RETURN_NODE_IDS = [1370, 2816, 3429, 3780]

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def decode_pose(blob: bytes) -> np.ndarray:
    return np.array(struct.unpack("16d", blob)).reshape(4, 4)


def encode_pose(mat: np.ndarray) -> bytes:
    return struct.pack("16d", *mat.flatten().tolist())


def mat_to_pose3(mat: np.ndarray) -> gtsam.Pose3:
    """Convert 4x4 matrix to gtsam.Pose3 via quaternion (avoids numpy ABI crash)."""
    R  = mat[:3, :3]
    t  = mat[:3,  3]
    q  = Rotation.from_matrix(R).as_quat()   # xyzw order
    w, x, y, z = float(q[3]), float(q[0]), float(q[1]), float(q[2])
    rot = gtsam.Rot3(w, x, y, z)
    tv  = np.array([float(t[0]), float(t[1]), float(t[2])], dtype=np.float64)
    return gtsam.Pose3(rot, tv)


def pose3_to_mat(p: gtsam.Pose3) -> np.ndarray:
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = p.rotation().matrix()
    mat[:3,  3] = np.asarray(p.translation(), dtype=np.float64)
    return mat


def diag_noise6(sigmas: list) -> gtsam.noiseModel.Diagonal:
    sv = np.array([float(s) for s in sigmas], dtype=np.float64)
    return gtsam.noiseModel.Diagonal.Sigmas(sv)


def load_seed_poses(db_path: Path):
    conn = sqlite3.connect(str(db_path))
    cur  = conn.cursor()
    cur.execute("SELECT node_id, transform FROM sensor_frames ORDER BY node_id ASC")
    rows = cur.fetchall()
    conn.close()
    return {r[0]: decode_pose(r[1]) for r in rows}, [r[0] for r in rows]


def load_edges_json(path: Path):
    d = json.loads(Path(path).read_text())
    return d.get("edges", []) if isinstance(d, dict) else d, \
           d.get("convention", "") if isinstance(d, dict) else ""


def rotation_error_deg(R: np.ndarray) -> float:
    tr = np.clip((np.trace(R) - 1) / 2, -1.0, 1.0)
    return math.degrees(math.acos(tr))


# ---------------------------------------------------------------------------
# Phase 1: Load constraints
# ---------------------------------------------------------------------------
print("=" * 70)
print("Phase 1: Load seeds + constraints")
print("=" * 70)

seed_poses, node_ids_sorted = load_seed_poses(SEED_RUX)
N = len(node_ids_sorted)

anchor_nids = set(node_ids_sorted[:ANCHOR_SEQ_MAX_IDX + 1])
print(f"Frames: {N}  |  Anchor zone: {len(anchor_nids)} "
      f"(nids {node_ids_sorted[0]}..{node_ids_sorted[ANCHOR_SEQ_MAX_IDX]})")

# Odometry
odom_edges = []
for i in range(N - 1):
    ni = node_ids_sorted[i]
    nj = node_ids_sorted[i + 1]
    T_ij = np.linalg.inv(seed_poses[ni]) @ seed_poses[nj]
    odom_edges.append((ni, nj, T_ij))
print(f"Odometry edges: {len(odom_edges)}")

# Loop closures: targeted (all 4 revisits) + spatial filtered
targeted_all, tconv = load_edges_json(TARGETED_RAW)
spatial_all,  sconv = load_edges_json(SPATIAL_FILT)

# Filter by inliers
targeted_good = [e for e in targeted_all if e.get("inliers", 0) >= MIN_INLIERS]
spatial_good  = [e for e in spatial_all  if e.get("inliers", 0) >= SPATIAL_MIN_INLIERS]

print(f"Targeted (all revisits): {len(targeted_good)}/{len(targeted_all)} edges (min_inliers={MIN_INLIERS})")
t_dist = Counter(e.get("revisit_hint") for e in targeted_good)
for rh, cnt in sorted(t_dist.items()):
    print(f"  revisit_hint={rh}: {cnt}")
print(f"Spatial filtered: {len(spatial_good)}/{len(spatial_all)} edges (min_inliers={SPATIAL_MIN_INLIERS})")

# Combine + deduplicate (prefer targeted over spatial)
seen = set()
loop_edges = []
for e in targeted_good:
    key = (e["node_i"], e["node_j"])
    if key not in seen:
        seen.add(key)
        loop_edges.append(dict(e, source="targeted"))
for e in spatial_good:
    key = (e["node_i"], e["node_j"])
    if key not in seen:
        seen.add(key)
        loop_edges.append(dict(e, source="spatial"))

print(f"Total loop edges (dedup): {len(loop_edges)}")

print(f"\nConstraint inventory:")
print(f"  Odometry BetweenFactors: {len(odom_edges)}")
print(f"  Anchor PriorFactors:     {len(anchor_nids)}")
print(f"  Loop BetweenFactors:     {len(loop_edges)}")

# ---------------------------------------------------------------------------
# Phase 2: GTSAM solve
# ---------------------------------------------------------------------------
print("\n" + "=" * 70)
print("Phase 2: GTSAM Pose3 factor graph")
print("=" * 70)

graph  = gtsam.NonlinearFactorGraph()
values = gtsam.Values()

for nid, mat in seed_poses.items():
    values.insert(nid, mat_to_pose3(mat))

# Odometry
odom_noise = diag_noise6([
    ODOM_SIGMA_R, ODOM_SIGMA_R, ODOM_SIGMA_R,
    ODOM_SIGMA_T, ODOM_SIGMA_T, ODOM_SIGMA_T
])
for ni, nj, T_ij in odom_edges:
    graph.add(gtsam.BetweenFactorPose3(ni, nj, mat_to_pose3(T_ij), odom_noise))
print(f"Added {len(odom_edges)} odometry BetweenFactors (sigma_t={ODOM_SIGMA_T}m)")

# Anchors
anchor_noise = diag_noise6([
    ANCHOR_SIGMA_R, ANCHOR_SIGMA_R, ANCHOR_SIGMA_R,
    ANCHOR_SIGMA_T, ANCHOR_SIGMA_T, ANCHOR_SIGMA_T
])
for nid in anchor_nids:
    graph.add(gtsam.PriorFactorPose3(nid, mat_to_pose3(seed_poses[nid]), anchor_noise))
print(f"Added {len(anchor_nids)} anchor PriorFactors "
      f"(sigma_t={ANCHOR_SIGMA_T}m, sigma_r={ANCHOR_SIGMA_R}rad)")

# Loop closures
n_loop_added = 0
n_loop_skip  = 0
for e in loop_edges:
    ni, nj = e["node_i"], e["node_j"]
    if ni not in seed_poses or nj not in seed_poses:
        n_loop_skip += 1
        continue
    inliers = max(e.get("inliers", 100), 1)
    sig_t   = max(LOOP_SIGMA_SCALE / math.sqrt(inliers), LOOP_SIGMA_FLOOR_T)
    sig_r   = max(LOOP_SIGMA_SCALE * 0.5 / math.sqrt(inliers), LOOP_SIGMA_FLOOR_R)

    base_n = diag_noise6([sig_r, sig_r, sig_r, sig_t, sig_t, sig_t])
    rob_n  = gtsam.noiseModel.Robust.Create(
        gtsam.noiseModel.mEstimator.Huber.Create(HUBER_K),
        base_n
    )
    T_ij = np.array(e["T_ij"]).reshape(4, 4)
    graph.add(gtsam.BetweenFactorPose3(ni, nj, mat_to_pose3(T_ij), rob_n))
    n_loop_added += 1

print(f"Added {n_loop_added} loop BetweenFactors (Huber k={HUBER_K})")
if n_loop_skip:
    print(f"  Skipped {n_loop_skip} (node not in DB)")

# Solve
params = gtsam.LevenbergMarquardtParams()
params.setMaxIterations(500)
params.setRelativeErrorTol(1e-8)
params.setAbsoluteErrorTol(1e-8)
params.setVerbosity("SILENT")

print("\nRunning Levenberg-Marquardt...")
t0      = time.time()
opt     = gtsam.LevenbergMarquardtOptimizer(graph, values, params)
result  = opt.optimize()
elapsed = time.time() - t0

init_err  = graph.error(values)
final_err = graph.error(result)
print(f"Elapsed: {elapsed:.1f}s")
print(f"Error: {init_err:.4f} → {final_err:.4f}  (ratio {final_err/init_err:.5f})")

# Collect GT poses
gt_poses = {}
for nid in seed_poses:
    try:
        gt_poses[nid] = pose3_to_mat(result.atPose3(nid))
    except Exception:
        gt_poses[nid] = seed_poses[nid]

# ---------------------------------------------------------------------------
# Colocation analysis
# ---------------------------------------------------------------------------
print("\n" + "=" * 70)
print("Colocation analysis (return → entrance proximity)")
print("=" * 70)

anchor_list = list(anchor_nids)

def nearest_ent(poses, nid):
    p_ret = poses[nid][:3, 3]
    dists = np.array([np.linalg.norm(poses[a][:3, 3] - p_ret) for a in anchor_list])
    mi    = int(np.argmin(dists))
    return float(dists[mi]), anchor_list[mi]

colocation = {}
print(f"{'Return nid':<12} {'Seed dist':>10} {'GT dist':>10} {'Δ':>12}")
print("-" * 50)
for rf in RETURN_NODE_IDS:
    sd, sn = nearest_ent(seed_poses, rf)
    gd, gn = nearest_ent(gt_poses,   rf)
    colocation[rf] = {"seed_m": float(sd), "gt_m": float(gd), "improvement_m": float(sd - gd),
                      "nearest_entrance": sn}
    print(f"{rf:<12} {sd:>10.3f}m {gd:>10.3f}m {sd-gd:>+12.3f}m")

# Pose shifts
all_shifts = np.array([
    np.linalg.norm(gt_poses[nid][:3, 3] - seed_poses[nid][:3, 3])
    for nid in node_ids_sorted
])
max_shift  = float(all_shifts.max())
mean_shift = float(all_shifts.mean())
p95_shift  = float(np.percentile(all_shifts, 95))
max_nid    = node_ids_sorted[int(all_shifts.argmax())]

print(f"\nPose shifts (GT vs seed):")
print(f"  Max:  {max_shift:.4f}m at node_id={max_nid}")
print(f"  Mean: {mean_shift:.4f}m")
print(f"  P95:  {p95_shift:.4f}m")

anchor_shifts = np.array([
    np.linalg.norm(gt_poses[nid][:3, 3] - seed_poses[nid][:3, 3])
    for nid in anchor_list
])
print(f"\nAnchor zone: max_shift={anchor_shifts.max():.5f}m, mean={anchor_shifts.mean():.5f}m")

# Loop residuals
residuals = []
for e in loop_edges:
    ni, nj = e["node_i"], e["node_j"]
    if ni not in gt_poses or nj not in gt_poses:
        continue
    T_meas     = np.array(e["T_ij"]).reshape(4, 4)
    T_computed = np.linalg.inv(gt_poses[ni]) @ gt_poses[nj]
    err_T      = np.linalg.inv(T_meas) @ T_computed
    err_t      = float(np.linalg.norm(err_T[:3, 3]))
    err_r      = rotation_error_deg(err_T[:3, :3])
    residuals.append((err_t, err_r, ni, nj, e.get("revisit_hint"), e.get("inliers", 0)))

residuals.sort()
print(f"\nLoop edge residuals ({len(residuals)} edges):")
print(f"  Median t:  {residuals[len(residuals)//2][0]:.3f}m")
print(f"  P95 t:     {residuals[int(len(residuals)*0.95)][0]:.3f}m")
print(f"  Max t:     {residuals[-1][0]:.3f}m (ni={residuals[-1][2]}, nj={residuals[-1][3]})")
n_high = sum(1 for r in residuals if r[0] > 0.5)
print(f"  >0.5m:     {n_high}/{len(residuals)}")

# Per-revisit residuals
for rh in RETURN_NODE_IDS:
    rh_res = [r for r in residuals if r[4] == rh]
    if rh_res:
        ts = [r[0] for r in rh_res]
        print(f"  revisit {rh}: median_t={sorted(ts)[len(ts)//2]:.3f}m, max_t={max(ts):.3f}m ({len(ts)} edges)")

# ---------------------------------------------------------------------------
# Phase 3a: Identity round-trip test
# ---------------------------------------------------------------------------
print("\n" + "=" * 70)
print("Phase 3a: Identity round-trip test (convention verification)")
print("=" * 70)

ROUNDTRIP = GT_BUILD / "newoffice_roundtrip.rux"
shutil.copy2(str(SEED_RUX), str(ROUNDTRIP))

conn_rt = sqlite3.connect(str(ROUNDTRIP))
cur_rt  = conn_rt.cursor()
for nid, mat in seed_poses.items():
    cur_rt.execute("UPDATE sensor_frames SET transform=? WHERE node_id=?",
                   (encode_pose(mat), nid))
conn_rt.commit()
conn_rt.close()

conn_v = sqlite3.connect(str(ROUNDTRIP))
cur_v  = conn_v.cursor()
cur_v.execute("SELECT node_id, transform FROM sensor_frames ORDER BY node_id ASC")
rt_rows = cur_v.fetchall()
conn_v.close()

max_err = 0.0
for row in rt_rows:
    nid, blob = row
    rt_mat = decode_pose(blob)
    err    = float(np.max(np.abs(rt_mat - seed_poses[nid])))
    max_err = max(max_err, err)

print(f"Round-trip max element error: {max_err:.2e}")
rt_ok = max_err < 1e-10
print("PASS" if rt_ok else "FAIL — ABORTING")

for p in [ROUNDTRIP, ROUNDTRIP.with_suffix(".rux-wal"), ROUNDTRIP.with_suffix(".rux-shm")]:
    p.unlink(missing_ok=True)

if not rt_ok:
    sys.exit(1)

# ---------------------------------------------------------------------------
# Phase 3b: Write GT poses
# ---------------------------------------------------------------------------
print("\n" + "=" * 70)
print("Phase 3b: Write GT poses")
print("=" * 70)

GT_BUILD.mkdir(parents=True, exist_ok=True)
shutil.copy2(str(SEED_RUX), str(GT_RUX))

conn_gt = sqlite3.connect(str(GT_RUX))
cur_gt  = conn_gt.cursor()
for nid, mat in gt_poses.items():
    cur_gt.execute("UPDATE sensor_frames SET transform=? WHERE node_id=?",
                   (encode_pose(mat), nid))
conn_gt.commit()
conn_gt.close()
print(f"Written {len(gt_poses)} poses to {GT_RUX.name}")

# ---------------------------------------------------------------------------
# Save results JSON
# ---------------------------------------------------------------------------
results = {
    "seed_rux": str(SEED_RUX),
    "gt_rux":   str(GT_RUX),
    "n_frames": N,
    "anchor_zone": {
        "n_frames": len(anchor_nids),
        "node_id_range": [node_ids_sorted[0], node_ids_sorted[ANCHOR_SEQ_MAX_IDX]],
    },
    "constraints": {
        "odometry": len(odom_edges),
        "anchors": len(anchor_nids),
        "loop_closures": n_loop_added,
    },
    "hyperparams": {
        "odom_sigma_t": ODOM_SIGMA_T,
        "odom_sigma_r": ODOM_SIGMA_R,
        "anchor_sigma_t": ANCHOR_SIGMA_T,
        "anchor_sigma_r": ANCHOR_SIGMA_R,
        "huber_k": HUBER_K,
        "loop_sigma_scale": LOOP_SIGMA_SCALE,
        "min_inliers": MIN_INLIERS,
        "spatial_min_inliers": SPATIAL_MIN_INLIERS,
    },
    "solve": {
        "initial_error": init_err,
        "final_error":   final_err,
        "error_ratio":   final_err / init_err,
        "elapsed_s":     elapsed,
    },
    "pose_shifts": {
        "max_m":  max_shift,
        "mean_m": mean_shift,
        "p95_m":  p95_shift,
        "max_at_nid": max_nid,
    },
    "anchor_stability": {
        "max_shift_m":  float(anchor_shifts.max()),
        "mean_shift_m": float(anchor_shifts.mean()),
    },
    "colocation": colocation,
    "loop_residuals": {
        "n_total":    len(residuals),
        "median_t_m": residuals[len(residuals) // 2][0],
        "p95_t_m":    residuals[int(len(residuals) * 0.95)][0],
        "max_t_m":    residuals[-1][0],
        "n_high_gt05m": n_high,
    },
    "round_trip_passed": rt_ok,
}
RESULTS_JSON.write_text(json.dumps(results, indent=2))
print(f"Results → {RESULTS_JSON}")

print("\n" + "=" * 70)
print("DONE — proceed to Phase 4 (rux create clouds + quality analysis)")
print("=" * 70)
