// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/frame_visibility.hpp"

#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

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

/// Project @p world_point into one camera: the visibility record when it is
/// in front, within `max_depth` and inside the (margin-shrunk) image.
std::optional<FrameVisibility> project(const FrameCamera &cam,
                                       const Eigen::Vector3d &world_point,
                                       const VisibilityQuery &query) {
  const Eigen::Vector4d p_cam_h =
      cam.world_to_camera *
      Eigen::Vector4d(world_point.x(), world_point.y(), world_point.z(), 1.0);
  const double z = p_cam_h.z();

  // Behind the camera (or exactly on the plane): not visible.
  if (!(z > 0.0))
    return std::nullopt;
  if (query.max_depth > 0.0 && z > query.max_depth)
    return std::nullopt;

  const double inv_z = 1.0 / z;
  const double u = cam.fx * p_cam_h.x() * inv_z + cam.cx;
  const double v = cam.fy * p_cam_h.y() * inv_z + cam.cy;

  // Frustum bounds test, optionally shrunk by a pixel margin.
  const double m = query.margin_px;
  if (u < m || u >= static_cast<double>(cam.width) - m || v < m ||
      v >= static_cast<double>(cam.height) - m)
    return std::nullopt;

  const double du = u - cam.cx;
  const double dv = v - cam.cy;
  const double w = static_cast<double>(cam.width);
  const double h = static_cast<double>(cam.height);
  // half_diag is > 0 because width/height are > 0 (checked on load).
  const double half_diag = 0.5 * std::sqrt(w * w + h * h);
  const double centrality = std::sqrt(du * du + dv * dv) / half_diag;
  return FrameVisibility{cam.id, u, v, z, centrality};
}

/// Most central first; nearer depth then lower id break ties so the order is
/// total and reproducible (STANDARDS §6).
void sort_best_first(std::vector<FrameVisibility> &visible) {
  std::sort(visible.begin(), visible.end(),
            [](const FrameVisibility &a, const FrameVisibility &b) {
              if (a.centrality != b.centrality)
                return a.centrality < b.centrality;
              if (a.depth != b.depth)
                return a.depth < b.depth;
              return a.frame_id < b.frame_id;
            });
}

/// Rank the cameras that see @p world_point, best-first.
std::vector<FrameVisibility> rank(const CameraSet &set,
                                  const Eigen::Vector3d &world_point,
                                  const VisibilityQuery &query) {
  std::vector<FrameVisibility> visible;
  for (const auto &cam : set.cameras)
    if (auto hit = project(cam, world_point, query))
      visible.push_back(*hit);
  sort_best_first(visible);
  return visible;
}

/// True when @p world_point lands on the surface @p depth measured (CV_16U
/// millimetres), within the tolerance, looking up to the search radius.
bool on_depth_surface(const FrameCamera &cam, const cv::Mat &depth,
                      const Eigen::Vector3d &world_point,
                      const OcclusionQuery &query) {
  // The probe itself only needs to be in front and in the image; the margin
  // and range limits are the anchor's frustum test, already passed.
  const auto hit = project(cam, world_point, VisibilityQuery{});
  if (!hit)
    return false;
  const double sx = static_cast<double>(depth.cols) / cam.width;
  const double sy = static_cast<double>(depth.rows) / cam.height;
  const int px = static_cast<int>(hit->u * sx);
  const int py = static_cast<int>(hit->v * sy);
  const int r = std::max(0, query.depth_search_radius);
  for (int y = std::max(0, py - r); y <= std::min(depth.rows - 1, py + r);
       ++y) {
    const auto *row = depth.ptr<std::uint16_t>(y);
    for (int x = std::max(0, px - r); x <= std::min(depth.cols - 1, px + r);
         ++x) {
      if (row[x] == 0) // no measurement
        continue;
      if (std::abs(hit->depth - row[x] * 1e-3) <= query.depth_tolerance)
        return true;
    }
  }
  return false;
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

std::vector<std::vector<FrameVisibility>>
visible_frames_occluded(const ProjectDB &db,
                        const std::vector<VisibilityProbe> &probes,
                        const OcclusionQuery &query) {
  std::vector<std::vector<FrameVisibility>> out(probes.size());
  if (probes.empty())
    return out;
  const auto set = load_cameras(db);

  std::size_t depth_tested = 0, no_depth = 0, frustum_hits = 0, occluded = 0;
  std::vector<std::pair<std::size_t, FrameVisibility>> candidates;
  for (const auto &cam : set.cameras) {
    candidates.clear();
    for (std::size_t i = 0; i < probes.size(); ++i)
      if (auto hit = project(cam, probes[i].anchor, query.visibility))
        candidates.emplace_back(i, *hit);
    if (candidates.empty())
      continue; // no depth decode for a frame that sees nothing
    frustum_hits += candidates.size();

    cv::Mat depth = db.sensor_frame_depth(cam.id);
    if (depth.empty() || depth.type() != CV_16UC1) {
      ++no_depth;
      if (query.keep_frames_without_depth)
        for (const auto &[i, hit] : candidates)
          out[i].push_back(hit);
      else
        occluded += candidates.size();
      continue;
    }
    ++depth_tested;
    for (const auto &[i, hit] : candidates) {
      bool seen = on_depth_surface(cam, depth, probes[i].anchor, query);
      for (std::size_t k = 0; !seen && k < probes[i].samples.size(); ++k)
        seen = on_depth_surface(cam, depth, probes[i].samples[k], query);
      if (seen)
        out[i].push_back(hit);
      else
        ++occluded;
    }
  }
  for (auto &visible : out)
    sort_best_first(visible);

  std::size_t seen = 0;
  for (const auto &visible : out)
    if (!visible.empty())
      ++seen;
  report(set, probes.size(), seen);
  reusex::debug("visible_frames_occluded: {} frames depth-tested, {} without "
                "depth ({}), {} of {} frustum hits rejected by depth "
                "(tolerance {} m)",
                depth_tested, no_depth,
                query.keep_frames_without_depth ? "kept" : "dropped", occluded,
                frustum_hits, query.depth_tolerance);
  return out;
}

} // namespace reusex::core
