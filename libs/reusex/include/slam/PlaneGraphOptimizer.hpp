// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Plane-landmark pose-graph back-end (P1 of issue #225).
//
// ReUseX's first owned global pose-optimization stage. Where Joint Pairwise
// Registration (JointPairwiseRegistration) polishes poses locally via
// frame-to-frame point-to-plane residuals — and provably saturates because
// pairwise energy cannot express global consistency — this optimizer changes
// the objective to *shared global plane landmarks*:
//
//   1. detect the dominant planes each frame sees (from its surfels),
//   2. associate those observations across frames into persistent
//      gtsam::OrientedPlane3 landmarks,
//   3. build a factor graph (per-frame Pose3 + odometry BetweenFactors from the
//      seed poses + one OrientedPlane3Factor per observation + a soft prior on
//      the first pose for gauge), and
//   4. solve it with GncOptimizer (Graduated Non-Convexity) wrapping
//      Levenberg-Marquardt, so a handful of bad plane associations are
//      down-weighted rather than corrupting the global solution.
//
// The surfel extraction and the pose write-back (removal of the constant local
// optical->sensor transform) mirror the JPR path exactly, so the two stages are
// interchangeable in the pipeline.
//
// This header is deliberately free of any GTSAM include (STANDARDS.md §2): the
// factor graph lives entirely in the .cpp.

#pragma once

#include "reusex/segmentation/Surfel.hpp"
#include "reusex/segmentation/surfel_extraction.hpp"

#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::geometry {

/// Parameters for plane-landmark pose-graph optimization. All defaults live
/// here (STANDARDS.md §4); CLI flags mirror them.
struct PlaneGraphOptions {
  // --- Per-frame plane detection (sequential RANSAC on frame surfels) -------
  // "Mid-density" defaults: measured to beat the old conservative values
  // (4 / 200 / obs 5) on both office flatness and honka laser GT once
  // per-observation inlier weighting (plane_weight_by_inliers) is on.
  int max_planes_per_frame = 6;     ///< keep at most this many planes per frame
  int min_plane_inliers = 120;      ///< reject planes with fewer inliers
  float ransac_distance = 0.02f;    ///< inlier point-to-plane distance (m)
  float ransac_normal_angle = 20.f; ///< inlier normal agreement threshold (deg)
  int ransac_iterations = 200;      ///< RANSAC hypotheses per plane

  // --- Cross-frame association into landmarks -------------------------------
  float assoc_normal_angle = 10.f; ///< max normal angle to merge (deg)
  float assoc_distance = 0.10f;    ///< max plane-offset difference to merge (m)
  int min_landmark_observations =
      4; ///< landmark must be seen by >= this many frames
  /// Overlap gate: two observations may only merge into the same landmark if
  /// their in-plane footprints overlap. The in-plane gap between their inlier
  /// centroids must be below (sum of the two footprint radii + this margin).
  /// This is what stops two distinct offset/disjoint parallel walls that happen
  /// to agree in normal+offset (aliasing) from collapsing into one landmark.
  /// <= 0 disables the overlap check (pure normal+offset gating, legacy).
  float assoc_overlap_margin = 0.30f;
  /// Reject landmarks whose observation inlier-centroids are near-collinear: if
  /// the second singular value of the centroid scatter is below this fraction
  /// of the first, the observations pin only a line, leaving a rotational DoF
  /// about that line unconstrained. Such landmarks are dropped from the graph.
  /// <= 0 disables the degeneracy check.
  float min_landmark_spread_ratio = 0.05f;

  // --- Alternating association/optimization rounds --------------------------
  /// Number of (associate at current poses -> optimize -> refit) rounds. One
  /// round reproduces the original one-shot behaviour; 2-3 rounds let landmarks
  /// re-form on the improved poses (EM-style), which recovers drift the greedy
  /// one-shot association aliases away. Rounds stop early once the pose update
  /// of a round falls below assoc_round_tol. Two rounds is the tuned default
  /// (best office flatness at cm-level shift without regressing MuSHRoom GT).
  int assoc_rounds = 2;
  float assoc_round_tol = 0.02f; ///< stop rounds when max pose shift < this (m)

  // --- Factor graph noise (std-devs) ----------------------------------------
  // The seed trajectory (RTABMap SLAM) is already good, so odometry is trusted
  // tightly and the plane factors act as a gentle global regularizer: this is
  // what keeps corrections at cm scale and beats the no-op baseline instead of
  // degrading it (the #225 P1 defaults over-trusted the fragile plane factors).
  float odometry_sigma_rot = 0.005f;  ///< odometry rotation std (rad)
  float odometry_sigma_trans = 0.01f; ///< odometry translation std (m)
  /// Observability guard: a frame whose landmark observations span fewer than
  /// two independent normal directions is under-constrained by planes along the
  /// missing directions. Its two odometry factors (to prev/next frame) get
  /// their sigmas multiplied by this factor (< 1 tightens them) so the trusted
  /// odometry holds the trajectory where planes cannot. 1.0 disables the guard.
  float underconstrained_odom_scale = 0.25f;
  float plane_sigma_normal = 0.24f;   ///< plane-normal measurement std (rad)
  float plane_sigma_distance = 0.19f; ///< plane-distance measurement std (m)
  /// Per-observation inlier weighting of the plane factors. A plane fit from
  /// few surfels is far less reliable than one fit from thousands, yet both
  /// entered the graph with identical noise — so denser extraction (which adds
  /// mostly small, weak planes) previously WARPED poses instead of helping
  /// (measured: honka GT 0.795 -> 0.759). When enabled, each factor's sigmas
  /// are scaled by sqrt(ref / inliers), where `ref` is the median detection
  /// inlier count (self-calibrating across sampling settings), clamped to
  /// [plane_weight_min, plane_weight_max]. Well-supported planes pull harder;
  /// weak ones are down-weighted rather than trusted equally.
  bool plane_weight_by_inliers = true;
  float plane_weight_min = 0.5f;  ///< min sigma scale (strongest planes)
  float plane_weight_max = 3.0f;  ///< max sigma scale (weakest planes)
  float prior_sigma_rot = 0.001f; ///< first-pose gauge prior rotation std (rad)
  float prior_sigma_trans =
      0.001f; ///< first-pose gauge prior translation std (m)

  // --- Solver ---------------------------------------------------------------
  bool use_gnc = true; ///< wrap LM in Graduated Non-Convexity
  /// GNC TLS inlier threshold on the factor error (0.5 * whitened squared
  /// residual). gtsam's default of 1.0 rejects exactly the informative
  /// (drifted) plane observations this stage exists to exploit; the default
  /// here is the chi-square 99% quantile for a 3-DoF measurement / 2
  /// (11.345 / 2), so only genuinely wrong associations are down-weighted.
  float gnc_inlier_cost = 5.67f;
  int max_iterations = 100; ///< LM (inner) iteration cap
  unsigned seed = 42;       ///< RANSAC seed (STANDARDS.md §6)

  SurfelExtractionParams surfel; ///< surfel extraction settings
};

/// Summary statistics from a plane-graph optimization run.
struct PlaneGraphResult {
  bool converged = false;      ///< optimizer produced a solution
  int frames = 0;              ///< frames that participated
  int planes_detected = 0;     ///< total per-frame plane detections
  int landmarks = 0;           ///< persistent landmarks entering the graph
  int plane_factors = 0;       ///< OrientedPlane3 observation factors
  int iterations = 0;          ///< optimizer iterations performed
  double initial_error = 0.0;  ///< factor-graph error before optimization
  double final_error = 0.0;    ///< factor-graph error after optimization
  double max_pose_shift = 0.0; ///< largest per-frame translation change (m)
  int rounds = 0; ///< association/optimization rounds actually performed
  int landmarks_rejected_overlap = 0; ///< merges blocked by the overlap gate
  int landmarks_rejected_degenerate =
      0;                           ///< landmarks dropped as near-collinear
  int underconstrained_frames = 0; ///< frames whose odometry was tightened
};

/// Plane-landmark pose-graph optimizer operating purely in memory.
class PlaneGraphOptimizer {
    public:
  explicit PlaneGraphOptimizer(PlaneGraphOptions options);

  /// Globally optimize the world poses of the given frames in place (each
  /// FrameSurfels::world_pose is updated). Does not touch any database. When
  /// too few plane landmarks are found the poses are left unchanged.
  PlaneGraphResult optimize(std::vector<FrameSurfels> &frames) const;

    private:
  PlaneGraphOptions options_;
};

/// High-level entry point: extract surfels for every sensor frame in @p db,
/// globally optimize their poses via the plane-landmark graph, and (unless
/// @p dry_run) write the optimized world poses back into the sensor_frames
/// table.
///
/// The optimizer works on the combined optical->world pose; on write-back the
/// constant local (optical->sensor) transform is removed so the stored
/// `transform` column keeps its worldTf meaning (identical to
/// refine_sensor_poses).
///
/// @param db       Project database (read/write).
/// @param options  Optimization parameters.
/// @param dry_run  When true, compute and report statistics without writing.
/// @returns        Optimization statistics.
PlaneGraphResult optimize_sensor_poses(ProjectDB &db,
                                       const PlaneGraphOptions &options,
                                       bool dry_run = false);

} // namespace reusex::geometry
