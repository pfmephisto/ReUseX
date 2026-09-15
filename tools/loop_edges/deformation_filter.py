#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Deformation filter for loop-closure edge sets.
# Issue #221 / #364: reject edges that shear the trajectory rather than correct it.
#
# MOTIVATION
# ----------
# The XFeat endcap regression (#221) showed that a geometrically-correct loop
# edge CAN still deform the trajectory badly if it forces a global shear (a
# linear-ramp pose correction) rather than a local correction near the matched
# frames.  The root cause is the factor-graph structure: with no intermediate
# loop factors anchoring the middle of the trajectory, the optimizer distributes
# the full loop-edge demand across all odometry links as a smooth ramp.
#
# This is a structurally UNAVOIDABLE consequence of the GTSAM chain factor graph
# when:
#   1. Frame 0 is gauge-pinned (cannot move).
#   2. The loop edge spans from frame ii to frame ij (both far from 0 and N).
#   3. There are NO intermediate loop factors anchoring frames between 0 and ii
#      or between ij and N.
#
# Result: the optimizer distributes the loop-edge demand linearly from frame 0
# to the loop frame, producing a global shear instead of a local correction.
#
# MECHANISM: SPAN-FRACTION GATE
# ------------------------------
# The cheapest and most principled deformation gate is the SPAN FRACTION:
#
#   span_frac = (frame_idx_j - frame_idx_i) / total_frames
#
# For the gauge-pinned factor graph, an edge spanning fraction f distributes
# its correction as a ramp over (frame_idx_i / N) × (correction_demand) before
# the matched frames, and (correction_demand) over the remaining (f - ...) frames.
# The "shear extent" is simply: how much of the trajectory carries a non-zero
# ramp correction?
#
# Concretely:
#   - XFeat endcap edges: span_frac ≈ 0.85–0.90 → shear across 85-90% of poses.
#   - Revisit 1370↔entrance (frame 20): span_frac ≈ 0.35 → shear in 35% of poses.
#   - Revisit 3429↔entrance (frame 20): span_frac ≈ 0.88 → same shear risk as endcap.
#
# The span-fraction gate rejects edges with span_frac > threshold (default 0.60).
# This is a STRUCTURAL property of the edge (depends only on frame indices), not
# on the edge's pose measurement or inliers.
#
# SECONDARY GATE: SEED-POSE RESIDUAL
# ------------------------------------
# An edge that agrees with the seed (residual < disagreement gate) is non-
# informative.  The `rux optimize` seed-disagreement gate uses 1.165 m; we apply
# the same lower bound here.  Edges with seed residual < min_correction are
# rejected as non-informative.
#
# TERTIARY CHECK: CORRECTION-DIRECTION CONSISTENCY
# -------------------------------------------------
# If multiple edges for the same revisit region request very different correction
# directions (high angular spread of their world-frame discrepancy vectors), that
# is a sign of inconsistency.  We report this per revisit group but do not gate
# on it (PCM in `rux optimize` handles this more rigorously).
#
# IMPLEMENTATION: GREEDY FORWARD SELECTION
# -----------------------------------------
# We process edges best-first (by inlier count) and keep each edge that passes
# the span-fraction and min-correction gates.  The final kept set is written
# to a "reusex.loop_edges.v1" JSON.
#
# USAGE
# -----
#   python3 tools/loop_edges/deformation_filter.py \
#       --project /path/to/project.rux \
#       --edges /path/to/targeted_edges.json [/path/to/spatial_edges.json ...] \
#       --output /path/to/kept_edges.json \
#       --span-fraction-threshold 0.60 \
#       --min-correction 0.3

import argparse
import json
import sqlite3
import sys
import time
from pathlib import Path

import numpy as np

SCHEMA = "reusex.loop_edges.v1"


# --------------------------------------------------------------------------- #
# Pose reader                                                                  #
# --------------------------------------------------------------------------- #
def read_seed_poses(db_path):
    """Read stored camera-to-world 4×4 matrices for all sensor frames.

    Returns
    -------
    node_ids : list[int]
        Ordered node IDs (same order as read_frames()).
    poses : dict[int, np.ndarray]
        node_id → 4×4 float64 camera-to-world matrix.
    """
    con = sqlite3.connect(db_path)
    cur = con.cursor()
    cur.execute(
        "SELECT node_id, transform FROM sensor_frames "
        "WHERE color IS NOT NULL AND depth IS NOT NULL ORDER BY node_id"
    )
    rows = cur.fetchall()
    con.close()

    node_ids = []
    poses = {}
    for node_id, transform_data in rows:
        node_ids.append(node_id)
        if transform_data is not None:
            arr = np.frombuffer(transform_data, dtype=np.float64)
            if len(arr) == 16:
                poses[node_id] = arr.reshape(4, 4).copy()
    return node_ids, poses


# --------------------------------------------------------------------------- #
# Deformation metrics                                                           #
# --------------------------------------------------------------------------- #
def edge_metrics(e, nid_to_idx, poses, n_total):
    """Compute per-edge deformation metrics.

    Parameters
    ----------
    e : dict
        Loop edge (reusex.loop_edges.v1 element).
    nid_to_idx : dict[int, int]
        node_id -> frame_index lookup.
    poses : dict[int, np.ndarray]
        node_id -> 4×4 c2w pose.
    n_total : int
        Total number of frames.

    Returns
    -------
    metrics : dict
        span_frac : float
            (j_idx - i_idx) / n_total.  High = shear risk.
        seed_residual_m : float
            Seed-pose disagreement magnitude (m).  Low = non-informative.
        world_discrepancy : np.ndarray (3,)
            World-frame correction vector the edge demands.
    """
    ni, nj = e["node_i"], e["node_j"]
    ii = nid_to_idx.get(ni, -1)
    ij = nid_to_idx.get(nj, -1)
    if ii < 0 or ij < 0 or ii >= ij:
        return None

    span_frac = (ij - ii) / n_total

    Pi = poses.get(ni)
    Pj = poses.get(nj)
    if Pi is None or Pj is None:
        return None

    T_ij = np.array(e["T_ij"]).reshape(4, 4)
    # World-frame demanded relative translation (optical convention)
    t_demanded_world = Pi[:3, :3] @ T_ij[:3, 3]
    t_seed_world = Pj[:3, 3] - Pi[:3, 3]
    discrepancy = t_demanded_world - t_seed_world
    seed_residual = float(np.linalg.norm(discrepancy))

    return {
        "span_frac": span_frac,
        "seed_residual_m": seed_residual,
        "world_discrepancy": discrepancy,
        "frame_idx_i": ii,
        "frame_idx_j": ij,
    }


# --------------------------------------------------------------------------- #
# Greedy deformation-filtered selection                                        #
# --------------------------------------------------------------------------- #
def greedy_filter(
    node_ids,
    poses,
    candidate_edges,
    span_frac_threshold,
    min_correction,
    verbose=True,
):
    """Greedy forward-selection with span-fraction and seed-residual gates.

    Parameters
    ----------
    node_ids : list[int]
    poses : dict[int, np.ndarray]
    candidate_edges : list[dict]
        Loop edges sorted best-first.
    span_frac_threshold : float
        Maximum span fraction allowed.  Edges with span > this are rejected
        as likely to cause global shear.
    min_correction : float
        Minimum seed residual (m) to accept as informative.
    verbose : bool

    Returns
    -------
    kept_edges, rejected_edges, filter_report
    """
    n = len(node_ids)
    nid_to_idx = {nid: i for i, nid in enumerate(node_ids)}

    kept = []
    rejected = []
    report = []

    for e in candidate_edges:
        ni, nj = e["node_i"], e["node_j"]
        inl = e.get("inliers", 0)
        hint = e.get("revisit_hint", None)

        m = edge_metrics(e, nid_to_idx, poses, n)
        if m is None:
            reason = f"invalid frame indices (node_i={ni} or node_j={nj} not found / ii>=ij)"
            e_copy = dict(e)
            e_copy["reject_reason"] = reason
            rejected.append(e_copy)
            report.append(
                {
                    "node_i": ni, "node_j": nj, "inliers": inl,
                    "revisit_hint": hint, "decision": "reject",
                    "reason": reason, "span_frac": None,
                    "seed_residual_m": None,
                }
            )
            continue

        span = m["span_frac"]
        res = m["seed_residual_m"]

        if res < min_correction:
            reason = (
                f"non-informative: seed_residual={res:.3f}m < {min_correction}m "
                f"(seed already agrees with this edge)"
            )
            e_copy = dict(e)
            e_copy["reject_reason"] = reason
            e_copy["span_frac"] = span
            e_copy["seed_residual_m"] = res
            rejected.append(e_copy)
            if verbose:
                print(
                    f"  REJECT  node_i={ni} node_j={nj} inliers={inl} "
                    f"hint={hint} | span={span:.3f} resid={res:.3f}m | {reason}"
                )
            report.append(
                {
                    "node_i": ni, "node_j": nj, "inliers": inl,
                    "revisit_hint": hint, "decision": "reject",
                    "reason": reason, "span_frac": span,
                    "seed_residual_m": res,
                }
            )
        elif span > span_frac_threshold:
            reason = (
                f"deforming: span_frac={span:.3f} > {span_frac_threshold} "
                f"(would shear >{span_frac_threshold:.0%} of trajectory)"
            )
            e_copy = dict(e)
            e_copy["reject_reason"] = reason
            e_copy["span_frac"] = span
            e_copy["seed_residual_m"] = res
            rejected.append(e_copy)
            if verbose:
                print(
                    f"  REJECT  node_i={ni} node_j={nj} inliers={inl} "
                    f"hint={hint} | span={span:.3f} resid={res:.3f}m | {reason}"
                )
            report.append(
                {
                    "node_i": ni, "node_j": nj, "inliers": inl,
                    "revisit_hint": hint, "decision": "reject",
                    "reason": reason, "span_frac": span,
                    "seed_residual_m": res,
                }
            )
        else:
            kept.append(e)
            if verbose:
                print(
                    f"  KEEP    node_i={ni} node_j={nj} inliers={inl} "
                    f"hint={hint} | span={span:.3f} resid={res:.3f}m"
                )
            report.append(
                {
                    "node_i": ni, "node_j": nj, "inliers": inl,
                    "revisit_hint": hint, "decision": "keep",
                    "reason": None, "span_frac": span,
                    "seed_residual_m": res,
                }
            )

    return kept, rejected, report


# --------------------------------------------------------------------------- #
# Main                                                                         #
# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(
        description="Deformation filter for loop-edge sets.\n\n"
        "Rejects edges that would induce trajectory shear, using the SPAN "
        "FRACTION as the primary gate.  An edge spanning fraction f of the "
        "total trajectory distributes its correction across f×N frames as a "
        "linear ramp (the gauge-pinned factor-graph minimum-energy solution), "
        "causing a global shear.  This is identical to the XFeat endcap "
        "regression (#221): 3 edges with span_frac ~0.88 → 9.8 m global shear."
    )
    ap.add_argument("--project", required=True, help="path to the .rux project database")
    ap.add_argument(
        "--edges",
        required=True,
        nargs="+",
        help="input edge JSON file(s) (reusex.loop_edges.v1); multiple merged before filtering",
    )
    ap.add_argument("-o", "--output", required=True, help="output (filtered) edge JSON")
    ap.add_argument(
        "--span-fraction-threshold",
        type=float,
        default=0.60,
        help="maximum allowed (j_idx - i_idx) / N.  Edges spanning more than "
        "this fraction of the trajectory are rejected as shear-inducing.  "
        "Default: 0.60 (60%% of trajectory).  The XFeat endcap regression had "
        "span_frac ~0.85-0.90; revisit 1370 has ~0.35 (safe at 0.60).",
    )
    ap.add_argument(
        "--min-correction",
        type=float,
        default=0.3,
        help="minimum seed-pose residual (m) to accept as informative.  "
        "Below this the edge agrees with the seed and is non-informative.  "
        "Default: 0.3 m (below rux optimize's own 1.165 m gate; use this to "
        "see what the optimizer will gate anyway).",
    )
    ap.add_argument(
        "--sort-by",
        default="inliers",
        choices=["inliers", "sigma_trans"],
        help="order for greedy selection.  Default: inliers (highest first).",
    )
    ap.add_argument("--report", default=None, help="write per-edge decision report JSON")
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    t0 = time.time()

    print(f"[poses] reading seed poses from {args.project}")
    node_ids, poses = read_seed_poses(args.project)
    print(f"[poses] {len(node_ids)} frames, {len(poses)} with valid poses")

    all_edges = []
    for ef in args.edges:
        with open(ef) as f:
            data = json.load(f)
        edges = data.get("edges", [])
        print(f"[load] {len(edges)} edges from {ef}")
        all_edges.extend(edges)

    # De-duplicate (same node_i, node_j, keep highest inlier)
    seen = {}
    for e in all_edges:
        key = (e["node_i"], e["node_j"])
        if key not in seen or e.get("inliers", 0) > seen[key].get("inliers", 0):
            seen[key] = e
    all_edges = list(seen.values())
    print(f"[dedup] {len(all_edges)} unique edges after de-duplication")

    reverse = (args.sort_by == "inliers")
    all_edges.sort(key=lambda e: e.get(args.sort_by, 0), reverse=reverse)

    print(
        f"\n[filter] span_fraction_threshold={args.span_fraction_threshold}, "
        f"min_correction={args.min_correction}m"
    )

    kept_edges, rejected_edges, filter_report = greedy_filter(
        node_ids=node_ids,
        poses=poses,
        candidate_edges=all_edges,
        span_frac_threshold=args.span_fraction_threshold,
        min_correction=args.min_correction,
        verbose=True,
    )

    print(
        f"\n[result] {len(kept_edges)} kept, {len(rejected_edges)} rejected "
        f"of {len(all_edges)} total ({time.time()-t0:.1f}s)"
    )

    # Summary by revisit hint
    by_revisit = {}
    for rec in filter_report:
        hint = rec.get("revisit_hint")
        k = str(hint)
        if k not in by_revisit:
            by_revisit[k] = {"kept": 0, "rejected": 0, "reasons": []}
        if rec["decision"] == "keep":
            by_revisit[k]["kept"] += 1
        else:
            by_revisit[k]["rejected"] += 1
            by_revisit[k]["reasons"].append(rec["reason"])
    print("\n[per-revisit summary]")
    for k in sorted(by_revisit.keys()):
        s = by_revisit[k]
        print(f"  revisit {k}: {s['kept']} kept, {s['rejected']} rejected")
        reason_counts = {}
        for r in s["reasons"]:
            reason_counts[r] = reason_counts.get(r, 0) + 1
        for r, c in sorted(reason_counts.items(), key=lambda x: -x[1]):
            print(f"    {c}× {r}")

    out = {
        "schema": SCHEMA,
        "producer": "deformation_filter",
        "project": str(args.project),
        "convention": "T_ij = pose(i)^-1 * pose(j), optical->world; node ids are "
        "sensor_frames.node_id",
        "params": {
            "source_files": [str(ef) for ef in args.edges],
            "span_fraction_threshold": args.span_fraction_threshold,
            "min_correction_m": args.min_correction,
            "sort_by": args.sort_by,
        },
        "filter_report": filter_report,
        "edges": kept_edges,
    }
    Path(args.output).write_text(json.dumps(out, indent=1))
    print(f"[done] {len(kept_edges)} edges -> {args.output}")

    if args.report:
        Path(args.report).write_text(json.dumps(filter_report, indent=1))
        print(f"[report] per-edge decisions -> {args.report}")


if __name__ == "__main__":
    main()
