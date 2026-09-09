// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::gsplat {

/// One posed training image for the Gaussian-splatting optimizer.
///
/// The camera convention is the pipeline's optical frame — x right, y down,
/// z forward — matching both the sensor-frame intrinsics stored in ProjectDB
/// and the panorama frame documented in geometry/EquirectProjection.hpp, so
/// frames and panorama slices compose without any axis flip.
struct TrainingView {
  int id = -1;       ///< sensor-frame node id, or a synthetic id for a slice
  std::string name;  ///< human-readable label, e.g. "frame_0001995"
  cv::Mat image;     ///< BGR8, exactly the resolution K describes
  Eigen::Matrix3d K; ///< pinhole intrinsics for `image`
  Eigen::Matrix4d T_cw = Eigen::Matrix4d::Identity(); ///< world -> camera
  bool from_panorama = false; ///< true when this is a 360 tangent slice

  int width() const { return image.cols; }
  int height() const { return image.rows; }
};

/// Selection and preprocessing knobs for building the training set.
struct TrainingViewOptions {
  bool include_frames = true;

  /// Also dice content-aligned 360 panoramas into overlapping tangent views.
  /// Only panoramas whose `pose_source == "aligned"` are used — a panorama
  /// carrying only its timestamp-matched neighbour pose is not trustworthy
  /// enough to train against.
  bool include_panorama_slices = false;
  int pano_n_yaw = 8;         ///< equator slices per panorama
  double pano_fov_deg = 90.0; ///< per-slice horizontal FOV [deg]
  int pano_tile = 1024;       ///< slice size [px, square]

  /// Take every Nth sensor frame. 3DGS wants dense multi-view overlap, so the
  /// default keeps every frame; raise it to trade coverage for speed.
  int frame_stride = 1;

  /// Inclusive sensor-frame node-id range, -1 for unbounded. Validation on
  /// NewOffice showed a building-wide sparse sample renders black while a
  /// contiguous capture segment reconstructs cleanly, so region selection is a
  /// first-class knob rather than an afterthought
  /// (docs/research/gaussian-splatting.md).
  int first_frame = -1;
  int last_frame = -1;

  /// Downscale images (and intrinsics) so the long edge is at most this many
  /// pixels. 0 disables resizing. Training cost is per-pixel, so this is the
  /// single most effective speed knob.
  int max_image_size = 0;

  /// Hard cap on the number of views (0 = unlimited), applied after striding.
  std::size_t max_views = 0;
};

/// Load posed training views from a project.
///
/// Sensor-frame poses go through the same composition the COLMAP exporter
/// uses: ProjectDB stores `pose = T_wb` (sensor base in world) and intrinsics
/// carry `local_transform = T_bc`, so `T_wc = T_wb * T_bc` and
/// `T_cw = (T_wc)^{-1}`. Panorama slices use `T_w_slice = T_w_pano *
/// [R_pano_from_view | 0]`.
///
/// Frames without a colour image or with degenerate intrinsics are skipped
/// with a debug/warn line rather than silently dropped.
///
/// @throws std::runtime_error if the project yields no usable view.
std::vector<TrainingView>
load_training_views(const ProjectDB &db, const TrainingViewOptions &opt = {});

} // namespace reusex::gsplat
