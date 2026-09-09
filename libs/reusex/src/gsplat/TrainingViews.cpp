// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/gsplat/TrainingViews.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>
#include <reusex/core/logging.hpp>
#include <reusex/geometry/EquirectProjection.hpp>

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace reusex::gsplat {

namespace {

constexpr double kEps = 1e-9;

/// Row-major double[16] -> Eigen::Matrix4d (no SE(3) assumptions), matching
/// io/colmap.cpp so both consumers read ProjectDB poses identically.
Eigen::Matrix4d to_matrix4d(const std::array<double, 16> &m) {
  Eigen::Matrix4d out;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      out(r, c) = m[static_cast<std::size_t>(r) * 4 + c];
  return out;
}

/// Invert a rigid transform without forming a general 4x4 inverse.
Eigen::Matrix4d invert_rigid(const Eigen::Matrix4d &T) {
  Eigen::Matrix4d inv = Eigen::Matrix4d::Identity();
  const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  inv.block<3, 3>(0, 0) = R.transpose();
  inv.block<3, 1>(0, 3) = -R.transpose() * T.block<3, 1>(0, 3);
  return inv;
}

/// Shrink an image so its long edge is <= max_size, scaling K to match.
/// Returns false when nothing was done.
bool downscale(cv::Mat &img, Eigen::Matrix3d &K, int max_size) {
  if (max_size <= 0)
    return false;
  const int longest = std::max(img.cols, img.rows);
  if (longest <= max_size)
    return false;
  const double s = static_cast<double>(max_size) / longest;
  cv::Mat dst;
  // INTER_AREA is the correct filter for downscaling: box-averaging keeps the
  // photometric loss comparing like with like instead of chasing aliasing.
  cv::resize(img, dst, cv::Size(), s, s, cv::INTER_AREA);
  // Scale intrinsics by the *realised* ratio; cv::resize rounds the size.
  const double sx = static_cast<double>(dst.cols) / img.cols;
  const double sy = static_cast<double>(dst.rows) / img.rows;
  K(0, 0) *= sx;
  K(0, 2) *= sx;
  K(1, 1) *= sy;
  K(1, 2) *= sy;
  img = dst;
  return true;
}

Eigen::Matrix3d intrinsics_matrix(const core::SensorIntrinsics &intr) {
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  K(0, 0) = intr.fx;
  K(1, 1) = intr.fy;
  K(0, 2) = intr.cx;
  K(1, 2) = intr.cy;
  return K;
}

} // namespace

std::vector<TrainingView> load_training_views(const ProjectDB &db,
                                              const TrainingViewOptions &opt) {
  if (opt.frame_stride < 1)
    throw std::runtime_error("gsplat: frame_stride must be >= 1");

  std::vector<TrainingView> views;

  std::size_t skipped_no_image = 0, skipped_bad_intrinsics = 0,
              skipped_no_pose = 0, frames_considered = 0;

  if (opt.include_frames) {
    std::vector<int> ids = db.sensor_frame_ids();
    std::sort(ids.begin(), ids.end());

    // Region selection first, stride second: striding before clipping would
    // make the retained frames depend on where the range starts.
    std::vector<int> selected;
    selected.reserve(ids.size());
    for (int id : ids) {
      if (opt.first_frame >= 0 && id < opt.first_frame)
        continue;
      if (opt.last_frame >= 0 && id > opt.last_frame)
        continue;
      selected.push_back(id);
    }

    for (std::size_t i = 0; i < selected.size();
         i += static_cast<std::size_t>(opt.frame_stride)) {
      const int node_id = selected[i];
      ++frames_considered;

      // Pose first: a frame with no stored pose would otherwise enter the
      // training set as a camera at the world origin looking down +Z, and its
      // photometric loss is real gradient pulling the model toward a scene
      // that is not there — a smeared reconstruction with nothing in the log
      // to attribute it to (#330).
      if (!db.has_sensor_frame_pose(node_id)) {
        core::debug("gsplat: node {} has no usable stored pose, skipping",
                    node_id);
        ++skipped_no_pose;
        continue;
      }

      cv::Mat img = db.sensor_frame_image(node_id);
      if (img.empty()) {
        core::debug("gsplat: node {} has no colour image, skipping", node_id);
        ++skipped_no_image;
        continue;
      }

      core::SensorIntrinsics intr = db.sensor_frame_intrinsics(node_id);
      if (intr.width <= 0 || intr.height <= 0 || intr.fx <= kEps ||
          intr.fy <= kEps) {
        core::warn("gsplat: node {} has invalid intrinsics "
                   "(fx={} fy={} {}x{}), skipping",
                   node_id, intr.fx, intr.fy, intr.width, intr.height);
        ++skipped_bad_intrinsics;
        continue;
      }
      // The stored intrinsics describe the sensor's native resolution; if the
      // stored image differs, the image wins and K is rescaled to it (same
      // correction io/colmap.cpp applies).
      if (img.cols != intr.width || img.rows != intr.height) {
        const double sx = static_cast<double>(img.cols) / intr.width;
        const double sy = static_cast<double>(img.rows) / intr.height;
        intr.fx *= sx;
        intr.cx *= sx;
        intr.fy *= sy;
        intr.cy *= sy;
        intr.width = img.cols;
        intr.height = img.rows;
      }

      TrainingView v;
      v.id = node_id;
      v.name = fmt::format("frame_{:07d}", node_id);
      v.K = intrinsics_matrix(intr);
      v.image = img;
      downscale(v.image, v.K, opt.max_image_size);

      const Eigen::Matrix4d T_wb = to_matrix4d(db.sensor_frame_pose(node_id));
      const Eigen::Matrix4d T_bc = to_matrix4d(intr.local_transform);
      v.T_cw = invert_rigid(T_wb * T_bc);

      views.push_back(std::move(v));
      if (opt.max_views > 0 && views.size() >= opt.max_views)
        break;
    }
  }

  // ---- content-aligned 360 panoramas -> tangent pinhole views -------------
  if (opt.include_panorama_slices &&
      (opt.max_views == 0 || views.size() < opt.max_views)) {
    int pano_used = 0, slices = 0, pano_unaligned = 0;
    int slice_id = 100'000'000; // never collides with a sensor-frame node id
    for (const auto &pano : db.list_panoramic_images()) {
      if (!pano.has_pose || pano.pose_source != "aligned") {
        ++pano_unaligned;
        continue;
      }
      cv::Mat equirect = db.panoramic_image(pano.id);
      if (equirect.empty())
        continue;

      const Eigen::Matrix4d T_w_pano = to_matrix4d(pano.pose);
      auto tangent = geometry::overlapping_views(
          equirect, opt.pano_n_yaw, opt.pano_fov_deg, opt.pano_tile);
      for (std::size_t k = 0; k < tangent.size(); ++k) {
        const auto &t = tangent[k];

        // The slice shares the panorama's centre and differs only by a
        // rotation, so the world pose is a pure right-multiplication.
        Eigen::Matrix4d T_w_slice = Eigen::Matrix4d::Identity();
        T_w_slice.block<3, 3>(0, 0) =
            T_w_pano.block<3, 3>(0, 0) * t.R_pano_from_view;
        T_w_slice.block<3, 1>(0, 3) = T_w_pano.block<3, 1>(0, 3);

        TrainingView v;
        v.id = slice_id++;
        v.name = fmt::format("pano_{:04d}_{:02d}", pano.id, k);
        v.image = t.image;
        v.K = t.K;
        v.from_panorama = true;
        downscale(v.image, v.K, opt.max_image_size);
        v.T_cw = invert_rigid(T_w_slice);

        views.push_back(std::move(v));
        ++slices;
        if (opt.max_views > 0 && views.size() >= opt.max_views)
          break;
      }
      ++pano_used;
      if (opt.max_views > 0 && views.size() >= opt.max_views)
        break;
    }
    core::info("gsplat: added {} tangent slices from {} aligned panoramas "
               "({} skipped for lacking an aligned pose)",
               slices, pano_used, pano_unaligned);
  }

  if (views.empty())
    throw std::runtime_error(fmt::format(
        "gsplat: no usable training views out of {} sensor frames considered "
        "(skipped {} without a stored pose, {} without a colour image, {} with "
        "invalid intrinsics). A project whose frames carry no pose needs "
        "`rux import` to have brought poses in, or `rux optimize` / "
        "`rux register` to have produced them; also check that any "
        "--first-frame/--last-frame range covers the imported frames.",
        frames_considered, skipped_no_pose, skipped_no_image,
        skipped_bad_intrinsics));

  if (skipped_no_pose > 0 || skipped_no_image > 0 || skipped_bad_intrinsics > 0)
    core::warn("gsplat: skipped {} of {} sensor frames ({} without a stored "
               "pose, {} without a colour image, {} with invalid intrinsics)",
               skipped_no_pose + skipped_no_image + skipped_bad_intrinsics,
               frames_considered, skipped_no_pose, skipped_no_image,
               skipped_bad_intrinsics);

  // Refuse rather than train on a set that cannot possibly converge: with
  // fewer than two views there is no second ray to triangulate against, so
  // every Gaussian is free to sit anywhere along a viewing ray and the run
  // produces a confident-looking result that means nothing. Gate on the FINAL
  // view count (panorama slices included) so a project that legitimately
  // trains from 360 slices alone is unaffected, and only when poses were the
  // reason views went missing — the other skip paths already have their own
  // diagnostics and a genuinely tiny project is the user's call.
  constexpr std::size_t kMinViews = 2;
  if (skipped_no_pose > 0 && views.size() < kMinViews)
    throw std::runtime_error(fmt::format(
        "gsplat: only {} usable training view(s) after skipping {} of {} "
        "sensor frames for having no stored pose — at least {} are needed to "
        "triangulate anything. Run `rux optimize` (or re-import with poses) "
        "before training.",
        views.size(), skipped_no_pose, frames_considered, kMinViews));

  core::info("gsplat: {} training views ({}x{} first view)", views.size(),
             views.front().width(), views.front().height());
  return views;
}

} // namespace reusex::gsplat
