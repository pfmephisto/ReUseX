// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/frame_visibility.hpp"

#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace reusex::core {

namespace {

/// Row-major 4x4 (the storage layout ProjectDB uses for poses and local
/// transforms) mapped into an Eigen matrix without a copy of the layout.
using Matrix4dRM = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;

} // namespace

std::vector<FrameVisibility> visible_frames(const ProjectDB &db,
                                            const Eigen::Vector3d &world_point,
                                            const VisibilityQuery &query) {
  auto frame_ids = db.sensor_frame_ids();
  // Deterministic iteration regardless of the order the DB hands ids back
  // (STANDARDS §6); the final sort is stable in id too, but sorting up front
  // keeps the skip diagnostics reproducible as well.
  std::sort(frame_ids.begin(), frame_ids.end());

  std::vector<FrameVisibility> visible;
  visible.reserve(frame_ids.size());

  // Counters for the end-of-call diagnostic (STANDARDS §5): a caller staring
  // at an empty result deserves to know whether the point was simply out of
  // view or whether the project has no usable poses/intrinsics at all.
  std::size_t skipped_no_pose = 0;
  std::size_t skipped_bad_intrinsics = 0;
  std::size_t considered = 0;

  const Eigen::Vector4d p_world(world_point.x(), world_point.y(),
                                world_point.z(), 1.0);

  for (int id : frame_ids) {
    // A frame with no usable stored pose would be projected through
    // `sensor_frame_pose()`'s identity fallback, placing the point relative to
    // the world origin instead of the real camera (#336). Skip it.
    if (!db.has_sensor_frame_pose(id)) {
      ++skipped_no_pose;
      continue;
    }

    const auto intrinsics = db.sensor_frame_intrinsics(id);
    if (!(intrinsics.fx > 0.0) || !(intrinsics.fy > 0.0) ||
        intrinsics.width <= 0 || intrinsics.height <= 0) {
      ++skipped_bad_intrinsics;
      continue;
    }
    ++considered;

    const auto pose_array = db.sensor_frame_pose(id); // Base -> World
    const Matrix4dRM pose = Eigen::Map<const Matrix4dRM>(pose_array.data());
    const Matrix4dRM local = // Camera -> Base
        Eigen::Map<const Matrix4dRM>(intrinsics.local_transform.data());

    // world -> camera = (pose * local)^-1, the same chain vision::project()
    // applies to place a point in the camera optical frame.
    const Eigen::Matrix4d world_to_camera = (pose * local).inverse();
    const Eigen::Vector4d p_cam_h = world_to_camera * p_world;
    const double z = p_cam_h.z();

    // Behind the camera (or exactly on the plane): not visible.
    if (!(z > 0.0))
      continue;
    if (query.max_depth > 0.0 && z > query.max_depth)
      continue;

    const double inv_z = 1.0 / z;
    const double u = intrinsics.fx * p_cam_h.x() * inv_z + intrinsics.cx;
    const double v = intrinsics.fy * p_cam_h.y() * inv_z + intrinsics.cy;

    // Frustum bounds test, optionally shrunk by a pixel margin.
    const double m = query.margin_px;
    if (u < m || u >= static_cast<double>(intrinsics.width) - m || v < m ||
        v >= static_cast<double>(intrinsics.height) - m)
      continue;

    const double du = u - intrinsics.cx;
    const double dv = v - intrinsics.cy;
    const double half_diag =
        0.5 * std::sqrt(static_cast<double>(intrinsics.width) *
                            static_cast<double>(intrinsics.width) +
                        static_cast<double>(intrinsics.height) *
                            static_cast<double>(intrinsics.height));
    // half_diag is > 0 because width/height are > 0 (checked above).
    const double centrality = std::sqrt(du * du + dv * dv) / half_diag;

    visible.push_back(FrameVisibility{id, u, v, z, centrality});
  }

  // Most central first; nearer depth then lower id break ties so the order is
  // total and reproducible (STANDARDS §6).
  std::sort(visible.begin(), visible.end(),
            [](const FrameVisibility &a, const FrameVisibility &b) {
              if (a.centrality != b.centrality)
                return a.centrality < b.centrality;
              if (a.depth != b.depth)
                return a.depth < b.depth;
              return a.frame_id < b.frame_id;
            });

  if (frame_ids.empty()) {
    reusex::warn(
        "visible_frames: project has no sensor frames — point ({:.3f}, "
        "{:.3f}, {:.3f}) is visible in 0 frames",
        world_point.x(), world_point.y(), world_point.z());
  } else if (considered == 0) {
    reusex::warn(
        "visible_frames: none of {} sensor frames were usable ({} had "
        "no stored pose, {} had degenerate intrinsics) — point ({:.3f}, "
        "{:.3f}, {:.3f}) is visible in 0 frames",
        frame_ids.size(), skipped_no_pose, skipped_bad_intrinsics,
        world_point.x(), world_point.y(), world_point.z());
  } else {
    reusex::debug("visible_frames: point ({:.3f}, {:.3f}, {:.3f}) visible in "
                  "{}/{} frames ({} skipped: {} no pose, {} bad intrinsics)",
                  world_point.x(), world_point.y(), world_point.z(),
                  visible.size(), considered,
                  skipped_no_pose + skipped_bad_intrinsics, skipped_no_pose,
                  skipped_bad_intrinsics);
  }

  return visible;
}

} // namespace reusex::core
