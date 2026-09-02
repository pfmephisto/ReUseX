// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// P2 of issue #225: wide-baseline loop-closure edges for the plane-landmark
// pose graph.
//
// The plane-landmark back-end (PlaneGraphOptimizer) fixes *local* global
// consistency where frames co-observe the same walls, but it cannot constrain
// two temporally distant views of the same place that share no plane landmark
// chain — the "basin problem" from docs/research/registration-improvements.md.
// This module supplies the missing wide-baseline constraints: it proposes
// loop-candidate frame pairs, matches them with image features, lifts the
// matches to metric 3D using the stored per-frame depth, and estimates a robust
// metric relative pose (RANSAC 3D-3D). Each accepted pair becomes a
// gtsam::BetweenFactor<Pose3> loop edge fed into the SAME GncOptimizer graph,
// so a wrong edge is down-weighted rather than corrupting the solution.
//
// Front-end design (research doc §Q2, commercial-safe path): the matcher is
// abstracted behind FrameMatcher so the default OpenCV ORB detector+matcher
// (BSD, no model weights, verifiable offline) can be swapped for a learned
// matcher (EfficientLoFTR / LightGlue+ALIKED via the existing TensorRT backend)
// without touching the graph wiring. The bundled superpoint.pt is
// non-commercial and deliberately NOT used here.
//
// This header carries no GTSAM include (STANDARDS.md §2); the factor lives in
// PlaneGraphOptimizer.cpp.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::geometry {

/// A metric relative-pose constraint between two frames, ready to become a
/// gtsam::BetweenFactor<Pose3>. Indices are positions in the frame list the
/// optimizer works on (NOT database node ids). The measurement is the relative
/// pose T such that pose(j) = pose(i) * T, i.e. T = pose(i)^-1 * pose(j),
/// expressed in the same optical->world convention the graph uses.
struct LoopEdge {
  int i = -1;                ///< first frame (index into the frames vector)
  int j = -1;                ///< second frame (index into the frames vector)
  Eigen::Matrix4d T_ij;      ///< relative pose pose(i)^-1 * pose(j)
  double sigma_rot = 0.05;   ///< rotational std of the edge (rad)
  double sigma_trans = 0.05; ///< translational std of the edge (m)
  int inliers = 0;           ///< RANSAC inlier correspondences supporting it
};

/// How loop-candidate frame pairs are proposed before geometric verification.
enum class LoopProposal {
  /// Pick automatically (default): `exhaustive` when the temporally-distant
  /// pair count is within `exhaustive_budget` (small scans — the reliable
  /// choice, since the appearance shortlist can miss the true loop under indoor
  /// perceptual aliasing), otherwise `appearance`.
  automatic,
  /// Spatial shortlist from the SEED poses (camera-centre proximity + view
  /// agreement). Cheap, but structurally BLIND to loops the drift has pulled
  /// apart — a start/end revisit under trajectory drift is never proposed
  /// because the drifted poses place those frames far apart. Use only when the
  /// seed poses are already globally consistent.
  spatial,
  /// Appearance shortlist from a pose-INDEPENDENT bag-of-words over the frames'
  /// own ORB descriptors. Finds revisits by how the images LOOK, not where the
  /// (possibly drifted) poses say the camera was — this is what detects the
  /// start/end loop that fixes accumulated drift. Default.
  appearance,
  /// Every temporally distant pair (i, j) with j - i > min_frame_gap. Pose- and
  /// appearance-independent; the ground-truth proposer for small scans where
  /// the
  /// O(N^2) matcher cost is affordable (a few hundred frames).
  exhaustive
};

/// Parameters for wide-baseline loop-edge detection.
struct LoopClosureOptions {
  bool enable =
      false; ///< off by default; opt-in via `rux optimize --loop-closure`

  /// Candidate-proposal strategy (see LoopProposal). Default `automatic`.
  LoopProposal proposal = LoopProposal::automatic;
  /// `automatic` uses exhaustive proposal when the temporally-distant pair
  /// count is at most this; above it, appearance (BoW) retrieval. ~80k pairs is
  /// a few minutes of matching.
  int exhaustive_budget = 80000;

  // --- Candidate proposal (common) -----------------------------------------
  /// Skip pairs closer than this in frame index (those are handled by the
  /// temporal odometry factors, not loop edges).
  int min_frame_gap = 50;
  /// Keep at most this many candidates per frame (nearest / most-similar
  /// first). Caps the O(N^2) proposal down to ~N*K matcher calls.
  int max_candidates_per_frame = 5;

  // --- Spatial proposal only -----------------------------------------------
  /// Propose a pair only if the seed camera centres are within this distance.
  float max_candidate_distance = 2.5f; ///< m
  /// …and their optical viewing directions agree within this angle.
  float max_view_angle = 45.0f; ///< deg

  // --- Appearance proposal only (bag-of-words over ORB) --------------------
  int vocab_size = 400;            ///< visual words (binary k-means clusters)
  int vocab_sample_per_frame = 80; ///< descriptors sampled per frame for vocab
  int vocab_iterations = 8;        ///< k-means (k-majority) refinement passes

  // --- Feature matching + robust relative pose -----------------------------
  // Verification defaults are tuned to actually surface wide-baseline loops on
  // iPad-LiDAR scans: 1500 features / 0.05 m were too strict and found nothing;
  // 3000 features + a 0.10 m 3D-3D threshold (depth is noisy at range) recover
  // strong loops (measured: a start/end pair with ~90 RANSAC inliers).
  int max_features = 3000;    ///< ORB features per frame
  float ratio_test = 0.85f;   ///< Lowe ratio threshold for descriptor match
  int min_match_inliers = 60; ///< reject a pair below this many RANSAC inliers
  float ransac_inlier_dist = 0.10f; ///< 3D-3D inlier threshold (m)
  int ransac_iterations = 300;      ///< RANSAC hypotheses
  float min_depth = 0.3f;           ///< ignore keypoints with depth outside
  float max_depth = 5.0f;           ///< [min_depth, max_depth] (m)
  /// Optional sanity gate: reject an edge whose relative translation disagrees
  /// with the seed poses by more than this (m). DISABLED by default (<= 0):
  /// the whole point of loop closure is to correct drift, so a large
  /// seed-disagreement is the signal, not the noise — hard-gating it throws
  /// away exactly the informative constraints (the #221 sweep learned this the
  /// hard way). Geometric RANSAC inlier count + GNC do the outlier rejection
  /// instead. Set > 0 only when the seed poses are trusted and you want a
  /// coarse guard against gross mismatches.
  float max_seed_disagreement = 0.0f;
  /// LOWER gate: keep a loop edge only if its relative translation disagrees
  /// with the seed poses by at least this (m). An edge that already agrees with
  /// the seed carries NO drift-correction information — it only re-imposes the
  /// noisier ORB+depth estimate on poses that are already right, which perturbs
  /// an already-good scan (measured: honka GT 0.795 -> 0.73 from redundant
  /// edges). This is the opposite of max_seed_disagreement and is safe: it
  /// drops only non-informative edges, never the large-drift corrections. On a
  /// well-aligned scan this makes loop closure a near no-op; on a drifted scan
  /// the true loop (large disagreement) is kept. Set 0 to keep every edge.
  float min_seed_disagreement = 0.10f;

  // --- Edge noise ----------------------------------------------------------
  /// Base translational/rotational std at min_match_inliers; both shrink as
  /// sqrt(min_match_inliers/inliers) so well-supported edges pull harder (down
  /// to the floors below). Deliberately loose: an ORB+depth relative pose has
  /// cm-scale error, so an over-confident edge injects that error broadly
  /// (measured: tight defaults dropped honka GT 0.795 -> 0.770). GNC
  /// down-weights gross outliers on top.
  float base_sigma_trans = 0.10f; ///< m
  float base_sigma_rot = 0.05f;   ///< rad
  float min_sigma_trans = 0.04f;  ///< floor on the translational std (m)
  float min_sigma_rot = 0.02f;    ///< floor on the rotational std (rad)

  // --- Pairwise Consistency Maximization (false-positive rejection) --------
  /// Keep only the largest MUTUALLY CONSISTENT set of loop edges (Mangelson et
  /// al., PCM). Two true loops form a near-identity cycle when chained through
  /// the seed odometry between their endpoints; a perceptual-aliasing false
  /// positive does not. This is what makes trusting loop edges safe on scans
  /// with repeated structure. Strongly recommended before loop_edges_trusted.
  bool pcm = true;
  float pcm_trans_threshold = 0.30f; ///< cycle translation tolerance (m)
  float pcm_rot_threshold = 0.15f; ///< cycle rotation tolerance (rad, ~8.6 deg)
  /// Cap the edge set fed to the O(M^2) consistency test (keep the highest-
  /// inlier edges). 0 = no cap.
  int pcm_max_edges = 3000;

  unsigned seed = 42; ///< RANSAC determinism
};

/// Summary statistics from loop-edge detection.
struct LoopClosureResult {
  int candidates = 0;    ///< spatially proposed pairs that were matched
  int edges = 0;         ///< accepted loop edges
  int total_inliers = 0; ///< summed RANSAC inliers across accepted edges
};

/// Detect wide-baseline loop edges among the given frames.
///
/// @param db          project database (read-only image/depth/intrinsics
/// access)
/// @param node_ids    database node id of each frame, in frame-index order (so
///                    LoopEdge::i/j index straight into the caller's vector)
/// @param seed_poses  optical->world pose of each frame (used to propose
/// spatial
///                    candidates and sanity-check edges)
/// @param options     detection parameters
/// @param out_result  optional statistics
/// @returns           accepted loop edges (possibly empty)
std::vector<LoopEdge>
detect_loop_edges(ProjectDB &db, const std::vector<int> &node_ids,
                  const std::vector<Eigen::Matrix4d> &seed_poses,
                  const LoopClosureOptions &options,
                  LoopClosureResult *out_result = nullptr);

} // namespace reusex::geometry
