// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Content-based alignment of 360 panoramas to the scan.
//
// A panorama is imported and placed on the trajectory purely by nearest EXIF
// timestamp (`rux import 360`), inheriting the matched sensor frame's pose.
// That is only as good as the clock sync and gives the panorama no independent
// orientation. This module refines the panorama's 6-DoF pose from image
// content:
//
//   1. render virtual pinhole slices of the equirect
//   (geometry/EquirectProjection)
//   2. ORB-match each slice against nearby sensor frames — the SAME
//   license-clean
//      OpenCV ORB + Lowe-ratio front-end used by LoopClosure
//   3. lift frame matches to metric 3D via the frame's stored depth + pose
//   4. resect the panorama pose per slice with solvePnPRansac (each slice is a
//      true pinhole with known K and known slice->panorama rotation), then
//      refine over all slices' inliers in panorama-bearing space
//
// The refined pose is a panorama-optical->world transform in the SAME
// convention as sensor_frame_pose, so it drops straight into the viewer /
// exporters and (as a follow-up, issue #236) into the pose graph as loop
// constraints.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <string>

namespace reusex {
class ProjectDB;
}

namespace reusex::geometry {

/// Parameters for content-based panorama alignment.
struct PanoramaAlignmentOptions {
  // --- candidate sensor frames -------------------------------------------
  /// Try the timestamp-seed frame plus this many frames on each side of it (in
  /// sensor-frame-id order). A wider window co-observes the panorama from more
  /// directions, which conditions the translation in bearing resection.
  int candidate_window = 25;
  /// Cap the number of candidate frames actually matched (nearest-to-seed
  /// first), bounding the O(slices x frames) matcher cost.
  int max_candidates = 20;

  // --- panorama slicing (geometry/EquirectProjection) --------------------
  int n_yaw = 8;         ///< perspective slices around the equator
  double fov_deg = 90.0; ///< per-slice horizontal FOV (overlap for matching)
  int slice = 1008;      ///< slice size (px, square)

  // --- ORB front-end (mirrors LoopClosureOptions) ------------------------
  int max_features = 3000;  ///< ORB features per image
  float ratio_test = 0.85f; ///< Lowe ratio threshold
  float min_depth = 0.3f;   ///< ignore frame keypoints outside [min,max] depth
  float max_depth = 6.0f;

  // --- PnP resection -----------------------------------------------------
  int min_slice_correspondences = 6; ///< need this many to PnP a slice
  int min_inliers = 25;              ///< accept alignment above this (total)
  float ransac_reproj_px = 5.0f;     ///< solvePnPRansac reprojection threshold
  int ransac_iterations = 500;
  int refine_iterations = 10; ///< bearing-space Gauss-Newton refinement steps

  /// Plausibility gate: reject an alignment whose refined centre is further
  /// than this from the timestamp-seed frame. Bearing-only resection is weakly
  /// constrained in translation when the matched frames span a narrow cone, and
  /// can slide the centre far away with a still-small angular residual. The
  /// timestamp seed is a good spatial prior (the panorama was captured in the
  /// same session, metres from the nearest frame), so a large disagreement is
  /// an ill-conditioned solve, not a real correction. Set <= 0 to disable.
  double max_correction_m = 5.0;
  unsigned seed = 42;

  /// If non-empty, write an ORB-correspondence figure (panorama slice <-> the
  /// best-matching sensor frame, inlier matches drawn) for each aligned
  /// panorama to `<debug_dir>/<debug_name>_matches.jpg`.
  std::string debug_dir;
  std::string debug_name;
};

/// Outcome of aligning one panorama.
struct PanoramaAlignmentResult {
  int pano_id = -1;
  int seed_node = -1;
  bool aligned = false;
  int inliers = 0;       ///< pooled PnP inliers supporting the pose
  double rms_deg = -1.0; ///< angular reprojection RMS over inliers (deg)
  /// panorama-optical->world, row-major 4x4 (same convention as
  /// ProjectDB::sensor_frame_pose). Identity when !aligned.
  std::array<double, 16> pose = {1, 0, 0, 0, 0, 1, 0, 0,
                                 0, 0, 1, 0, 0, 0, 0, 1};
  /// Translation delta from the timestamp-seed placement (m); -1 if no seed.
  double delta_from_seed_m = -1.0;
};

/// Align one panorama against nearby sensor frames. Read-only on @p db (does
/// not write the pose back — the caller decides, e.g. `rux align 360
/// --dry-run`).
///
/// @param db            project database
/// @param pano_id       panoramic_images.id
/// @param seed_node_id  timestamp-matched sensor frame node id (>=0), used to
///                      seed the candidate window and measure the correction
/// @param opt           parameters
PanoramaAlignmentResult align_panorama(ProjectDB &db, int pano_id,
                                       int seed_node_id,
                                       const PanoramaAlignmentOptions &opt);

} // namespace reusex::geometry
