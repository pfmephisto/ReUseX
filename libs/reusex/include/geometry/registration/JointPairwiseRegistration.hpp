// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Joint Pairwise Registration (JPR) for within-scan pose refinement.
//
// Reimplements the C++ core of the Joint Pairwise Registration method from
//   Q. Huang et al., "Uncertainty Quantification for Multi-Scan Registration",
//   ACM SIGGRAPH 2020 (MIT-licensed reference implementation:
//   https://github.com/huangqx/MultiScanRegistration).
//
// The reference is MATLAB/MEX research code operating on object scans; this is
// a native C++ port adapted to refine the per-frame sensor poses of a single
// ReUseX scan. Each sensor frame is treated as one JPR "scan": its depth is
// back-projected to an optical-frame surfel set, its pose is seeded from the
// stored (RTABMap) world pose, and all frame poses are jointly refined by
// minimizing point-to-plane distances between overlapping frame pairs.
//
// Deviations from the reference (documented intentionally):
//  - Pose increments use a minimal se(3) twist parametrization solved by a
//    sparse Gauss-Newton step, rather than the reference's 12-dim [t; vec(R)].
//  - Nearest-neighbour search uses pcl::KdTreeFLANN instead of the ANN library.
//  - Gauge freedom is fixed by a soft prior to the seed poses (and/or an
//    optional hard-anchored frame), which also prevents drift away from the
//    already-good RTABMap poses we are refining.

#pragma once

#include "reusex/geometry/Surfel.hpp"
#include "reusex/geometry/surfel_extraction.hpp"

#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::geometry {

/// Parameters for joint pairwise pose refinement.
struct JprParams {
  int max_iterations = 20;             ///< maximum Gauss-Newton outer iterations
  int neighbor_window = 5;             ///< temporal pairing half-window
  float max_corr_distance = 0.10f;     ///< reject correspondences beyond this (m)
  float normal_angle_threshold = 45.f; ///< reject if normals disagree beyond (deg)
  float robust_width = 0.05f;          ///< robust kernel width (m): Welsch sigma / Huber delta

  enum class Kernel { welsch, huber };
  Kernel kernel = Kernel::welsch;

  float prior_weight = 1.0f;   ///< soft prior pulling poses toward seeds (0 disables)
  int anchor_frame = -1;       ///< frame index (not node_id) to hard-fix; -1 = none
  float convergence_eps = 1e-4f; ///< stop when max per-frame ||dxi|| below this

  SurfelExtractionParams surfel; ///< surfel extraction settings
};

/// Summary statistics from a registration run.
struct JprResult {
  int frames = 0;          ///< number of frames that participated
  int iterations = 0;      ///< outer iterations performed
  double initial_rms = 0.0; ///< mean point-to-plane residual before (m)
  double final_rms = 0.0;   ///< mean point-to-plane residual after (m)
};

/// Joint pairwise pose optimizer operating purely in memory.
class JointPairwiseRegistration {
    public:
  explicit JointPairwiseRegistration(JprParams params);

  /// Jointly refine the world poses of the given frames in place (each
  /// FrameSurfels::world_pose is updated). Does not touch any database.
  JprResult refine(std::vector<FrameSurfels> &frames) const;

    private:
  JprParams params_;
};

/// High-level entry point: extract surfels for every sensor frame in @p db,
/// jointly refine their poses via JPR, and (unless @p dry_run) write the
/// refined world poses back into the sensor_frames table.
///
/// The optimizer works on the combined optical->world pose; on write-back the
/// constant local (optical->sensor) transform is removed so the stored
/// `transform` column keeps its worldTf meaning.
///
/// @param db       Project database (read/write).
/// @param params   Refinement parameters.
/// @param dry_run  When true, compute and report statistics without writing.
/// @returns        Convergence statistics.
JprResult refine_sensor_poses(ProjectDB &db, const JprParams &params,
                              bool dry_run = false);

} // namespace reusex::geometry
