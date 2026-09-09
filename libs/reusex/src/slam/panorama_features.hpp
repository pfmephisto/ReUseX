// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Internal (NOT public API) helpers shared by the two panorama front-ends:
//   - PanoramaAlignment.cpp  — resects ONE pooled panorama pose in WORLD space
//                              (`rux align 360`, placement refinement)
//   - PanoramaLoopEdges.cpp  — resects the panorama SEPARATELY in each matched
//                              frame's own optical frame (issue #236, loop
//                              edges)
//
// Both slice an equirect into perspective views, ORB-match those slices against
// sensor frames, lift the frame keypoints to metric 3D via the stored depth,
// and refine a bearing-space pose with the same Gauss-Newton step. Only the
// COORDINATE FRAME of the 3D points and the grouping of correspondences differ,
// so the geometry primitives live here rather than being duplicated.
//
// This header deliberately sits in src/ (not include/): it is an implementation
// detail of the slam module, not part of the stable API (docs/STANDARDS.md §2).

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <array>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::geometry::pano_detail {

// --- small pose/rotation utilities ----------------------------------------

/// Row-major std::array<16> (the sensor_frame_pose convention) -> Eigen 4x4.
Eigen::Matrix4d to_matrix4(const std::array<double, 16> &a);

/// Eigen 4x4 -> row-major std::array<16>.
std::array<double, 16> from_matrix4(const Eigen::Matrix4d &M);

/// so(3) exponential (Rodrigues) of a rotation vector.
Eigen::Matrix3d exp_so3(const Eigen::Vector3d &w);

/// Skew-symmetric cross-product matrix.
Eigen::Matrix3d skew(const Eigen::Vector3d &v);

/// Angular error (rad) between the observed unit bearing @p b_obs and the
/// bearing of point @p X under the pano_from_reference transform (@p Q, @p t).
double bearing_angle(const Eigen::Matrix3d &Q, const Eigen::Vector3d &t,
                     const Eigen::Vector3d &X, const Eigen::Vector3d &b_obs);

// --- frame features --------------------------------------------------------

/// ORB features of one sensor frame with each keypoint lifted to metric 3D.
///
/// The 3D points are stored in the frame's OWN OPTICAL coordinates. That choice
/// is what makes the loop-edge front-end pose-independent: a panorama resected
/// against `local` sees nothing of `sensor_frame_pose`, so the relative pose it
/// implies between two frames is a genuine measurement rather than a
/// restatement of the (drifted) seed trajectory. Callers that want world
/// coordinates — the alignment path, which resects a single pooled world pose —
/// compose with `T_world_cam` via world().
struct FrameFeatures {
  cv::Mat descriptors;                 ///< Nx32 CV_8U
  std::vector<cv::KeyPoint> keypoints; ///< pixel locations (figures/debug)
  std::vector<Eigen::Vector3d> local;  ///< 3D per keypoint, OPTICAL frame
  std::vector<char> valid;             ///< depth was finite and in range
  Eigen::Matrix4d T_world_cam = Eigen::Matrix4d::Identity(); ///< seed pose
  int node_id = -1;

  /// The k-th keypoint's 3D point in world coordinates (seed-pose dependent).
  Eigen::Vector3d world(size_t k) const {
    return T_world_cam.block<3, 3>(0, 0) * local[k] +
           T_world_cam.block<3, 1>(0, 3);
  }
};

/// Detect ORB features in a frame's colour image and lift them to 3D using the
/// stored depth map and intrinsics. Returns an empty result (no descriptors)
/// when the frame has no colour/depth or degenerate intrinsics.
FrameFeatures extract_frame_features(ProjectDB &db, int node_id,
                                     const cv::Ptr<cv::ORB> &orb,
                                     float min_depth, float max_depth);

// --- bearing-space pose refinement ----------------------------------------

/// Parameters of refine_bearing_pose.
struct BearingRefineOptions {
  double ang_gate = 0.01; ///< inlier angular tolerance (rad)
  int iterations = 10;    ///< Gauss-Newton steps
  int min_inliers = 25;   ///< abandon below this many gated correspondences
};

/// Gauss-Newton refinement of a pano_from_reference pose (@p Q, @p t) over
/// point<->bearing correspondences, re-gating the inlier set as the pose
/// improves.
///
/// @p points and @p bearings are parallel arrays: `bearings[k]` is the unit
/// bearing, in the PANORAMA frame, under which `points[k]` (in the reference
/// frame — world for alignment, frame-local for loop edges) was observed.
///
/// Left perturbation `T <- exp(xi) T`; residual `e = b_hat - b_obs`.
///
/// @param out_initial_inliers  if non-null, receives the inlier count of the
///                             INITIAL pose (before refinement), so callers can
///                             report why a resection was abandoned.
/// @returns indices into @p points of the final inlier set, or empty when the
///          initial or final gate falls below `min_inliers`.
std::vector<int>
refine_bearing_pose(const std::vector<Eigen::Vector3d> &points,
                    const std::vector<Eigen::Vector3d> &bearings,
                    const BearingRefineOptions &opt, Eigen::Matrix3d &Q,
                    Eigen::Vector3d &t, int *out_initial_inliers = nullptr);

} // namespace reusex::geometry::pano_detail
