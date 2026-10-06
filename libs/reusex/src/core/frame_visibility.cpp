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

namespace {

/// One posed, usable sensor frame, reduced to what a projection needs.
struct FrameCamera {
  int id = 0;
  Eigen::Matrix4d world_to_camera = Eigen::Matrix4d::Identity();
  double fx = 0, fy = 0, cx = 0, cy = 0;
  int width = 0, height = 0;
};

struct CameraSet {
  std::vector<FrameCamera> cameras;
  std::size_t total = 0;
  std::size_t skipped_no_pose = 0;
  std::size_t skipped_bad_intrinsics = 0;
};

/// Read every frame's pose and intrinsics once. Ids are sorted so iteration
/// is deterministic regardless of the order the DB hands them back
/// (STANDARDS §6).
CameraSet load_cameras(const ProjectDB &db) {
  auto frame_ids = db.sensor_frame_ids();
  std::sort(frame_ids.begin(), frame_ids.end());

  CameraSet set;
  set.total = frame_ids.size();
  set.cameras.reserve(frame_ids.size());
  for (int id : frame_ids) {
    // A frame with no usable stored pose would be projected through
    // `sensor_frame_pose()`'s identity fallback, placing the point relative
    // to the world origin instead of the real camera (#336). Skip it.
    if (!db.has_sensor_frame_pose(id)) {
      ++set.skipped_no_pose;
      continue;
    }
    const auto intrinsics = db.sensor_frame_intrinsics(id);
    if (!(intrinsics.fx > 0.0) || !(intrinsics.fy > 0.0) ||
        intrinsics.width <= 0 || intrinsics.height <= 0) {
      ++set.skipped_bad_intrinsics;
      continue;
    }
    const auto pose_array = db.sensor_frame_pose(id); // Base -> World
    const Matrix4dRM pose = Eigen::Map<const Matrix4dRM>(pose_array.data());
    const Matrix4dRM local = // Camera -> Base
        Eigen::Map<const Matrix4dRM>(intrinsics.local_transform.data());

    FrameCamera cam;
    cam.id = id;
    // world -> camera = (pose * local)^-1, the same chain vision::project()
    // applies to place a point in the camera optical frame.
    cam.world_to_camera = (pose * local).inverse();
    cam.fx = intrinsics.fx;
    cam.fy = intrinsics.fy;
    cam.cx = intrinsics.cx;
    cam.cy = intrinsics.cy;
    cam.width = intrinsics.width;
    cam.height = intrinsics.height;
    set.cameras.push_back(cam);
  }
  return set;
}

/// Rank the cameras that see @p world_point, best-first.
std::vector<FrameVisibility> rank(const CameraSet &set,
                                  const Eigen::Vector3d &world_point,
                                  const VisibilityQuery &query) {
  std::vector<FrameVisibility> visible;
  const Eigen::Vector4d p_world(world_point.x(), world_point.y(),
                                world_point.z(), 1.0);

  for (const auto &cam : set.cameras) {
    const Eigen::Vector4d p_cam_h = cam.world_to_camera * p_world;
    const double z = p_cam_h.z();

    // Behind the camera (or exactly on the plane): not visible.
    if (!(z > 0.0))
      continue;
    if (query.max_depth > 0.0 && z > query.max_depth)
      continue;

    const double inv_z = 1.0 / z;
    const double u = cam.fx * p_cam_h.x() * inv_z + cam.cx;
    const double v = cam.fy * p_cam_h.y() * inv_z + cam.cy;

    // Frustum bounds test, optionally shrunk by a pixel margin.
    const double m = query.margin_px;
    if (u < m || u >= static_cast<double>(cam.width) - m || v < m ||
        v >= static_cast<double>(cam.height) - m)
      continue;

    const double du = u - cam.cx;
    const double dv = v - cam.cy;
    const double w = static_cast<double>(cam.width);
    const double h = static_cast<double>(cam.height);
    // half_diag is > 0 because width/height are > 0 (checked on load).
    const double half_diag = 0.5 * std::sqrt(w * w + h * h);
    const double centrality = std::sqrt(du * du + dv * dv) / half_diag;

    visible.push_back(FrameVisibility{cam.id, u, v, z, centrality});
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
  return visible;
}

/// The end-of-call diagnostic (STANDARDS §5): a caller staring at an empty
/// result deserves to know whether the point was simply out of view or
/// whether the project has no usable poses/intrinsics at all.
void report(const CameraSet &set, std::size_t points, std::size_t seen) {
  if (set.total == 0) {
    reusex::warn("visible_frames: project has no sensor frames — {} point(s) "
                 "are visible in 0 frames",
                 points);
  } else if (set.cameras.empty()) {
    reusex::warn("visible_frames: none of {} sensor frames were usable ({} "
                 "had no stored pose, {} had degenerate intrinsics) — {} "
                 "point(s) are visible in 0 frames",
                 set.total, set.skipped_no_pose, set.skipped_bad_intrinsics,
                 points);
  } else {
    reusex::debug("visible_frames: {}/{} point(s) seen by at least one of {} "
                  "usable frames ({} no pose, {} bad intrinsics)",
                  seen, points, set.cameras.size(), set.skipped_no_pose,
                  set.skipped_bad_intrinsics);
  }
}

} // namespace

std::vector<FrameVisibility> visible_frames(const ProjectDB &db,
                                            const Eigen::Vector3d &world_point,
                                            const VisibilityQuery &query) {
  const auto set = load_cameras(db);
  auto visible = rank(set, world_point, query);
  report(set, 1, visible.empty() ? 0 : 1);
  return visible;
}

std::vector<std::vector<FrameVisibility>>
visible_frames_batch(const ProjectDB &db,
                     const std::vector<Eigen::Vector3d> &world_points,
                     const VisibilityQuery &query) {
  std::vector<std::vector<FrameVisibility>> out;
  out.reserve(world_points.size());
  if (world_points.empty())
    return out;
  const auto set = load_cameras(db);
  std::size_t seen = 0;
  for (const auto &p : world_points) {
    out.push_back(rank(set, p, query));
    if (!out.back().empty())
      ++seen;
  }
  report(set, world_points.size(), seen);
  return out;
}

} // namespace reusex::core
