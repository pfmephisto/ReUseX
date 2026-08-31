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

#include "reusex/geometry/Surfel.hpp"
#include "reusex/geometry/surfel_extraction.hpp"

#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::geometry {

/// Parameters for plane-landmark pose-graph optimization. All defaults live
/// here (STANDARDS.md §4); CLI flags mirror them.
struct PlaneGraphOptions {
  // --- Per-frame plane detection (sequential RANSAC on frame surfels) -------
  int max_planes_per_frame = 4;     ///< keep at most this many planes per frame
  int min_plane_inliers = 200;      ///< reject planes with fewer inliers
  float ransac_distance = 0.02f;    ///< inlier point-to-plane distance (m)
  float ransac_normal_angle = 20.f; ///< inlier normal agreement threshold (deg)
  int ransac_iterations = 200;      ///< RANSAC hypotheses per plane

  // --- Cross-frame association into landmarks -------------------------------
  float assoc_normal_angle = 10.f; ///< max normal angle to merge (deg)
  float assoc_distance = 0.10f;    ///< max plane-offset difference to merge (m)
  int min_landmark_observations =
      5; ///< landmark must be seen by >= this many frames

  // --- Factor graph noise (std-devs) ----------------------------------------
  float odometry_sigma_rot = 0.05f;   ///< odometry rotation std (rad)
  float odometry_sigma_trans = 0.10f; ///< odometry translation std (m)
  float plane_sigma_normal = 0.05f;   ///< plane-normal measurement std (rad)
  float plane_sigma_distance = 0.03f; ///< plane-distance measurement std (m)
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
