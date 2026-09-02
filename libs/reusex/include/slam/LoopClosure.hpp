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

/// Parameters for wide-baseline loop-edge detection.
struct LoopClosureOptions {
  bool enable =
      false; ///< off by default; opt-in via `rux optimize --loop-closure`

  // --- Candidate proposal (spatial shortlist from the seed poses) ----------
  /// Skip pairs closer than this in frame index (those are handled by the
  /// temporal odometry factors, not loop edges).
  int min_frame_gap = 50;
  /// Propose a pair only if the seed camera centres are within this distance.
  float max_candidate_distance = 2.5f; ///< m
  /// …and their optical viewing directions agree within this angle (a loop is a
  /// revisit, so the camera should look at roughly the same place).
  float max_view_angle = 45.0f; ///< deg
  /// Keep at most this many spatial candidates per frame (nearest first). Caps
  /// the O(N^2) proposal down to N*K matcher calls.
  int max_candidates_per_frame = 3;

  // --- Feature matching + robust relative pose -----------------------------
  int max_features = 1500;    ///< ORB features per frame
  float ratio_test = 0.80f;   ///< Lowe ratio threshold for descriptor match
  int min_match_inliers = 60; ///< reject a pair below this many RANSAC inliers
  float ransac_inlier_dist = 0.05f; ///< 3D-3D inlier threshold (m)
  int ransac_iterations = 300;      ///< RANSAC hypotheses
  float min_depth = 0.3f;           ///< ignore keypoints with depth outside
  float max_depth = 5.0f;           ///< [min_depth, max_depth] (m)
  /// Reject an edge whose relative translation disagrees with the seed poses by
  /// more than this (m). A correct loop edge should be near the seed estimate
  /// on already-good scans; a wildly different edge is a mismatch. Set high to
  /// admit large corrections. <= 0 disables the sanity gate.
  float max_seed_disagreement = 0.75f;

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
