// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "vision/instance_crop.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace reusex::vision {

std::optional<ProjectedBox>
project_points_to_box(const std::vector<Eigen::Vector3f> &world_points,
                      const std::array<double, 16> &pose,
                      const core::SensorIntrinsics &intrinsics, int image_width,
                      int image_height) {
  if (world_points.empty() || image_width <= 0 || image_height <= 0)
    return std::nullopt;
  if (intrinsics.fx <= 0.0 || intrinsics.fy <= 0.0 || intrinsics.width <= 0 ||
      intrinsics.height <= 0)
    return std::nullopt;

  // Row-major 4x4 world<-base pose and base<-camera local transform.
  using Matrix4dRM = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
  Eigen::Matrix4d base_to_world = Eigen::Map<const Matrix4dRM>(pose.data());
  Eigen::Matrix4d cam_to_base =
      Eigen::Map<const Matrix4dRM>(intrinsics.local_transform.data());

  // world -> camera = (base_to_world * cam_to_base)^-1
  Eigen::Matrix4d cam_to_world = base_to_world * cam_to_base;
  Eigen::Matrix4d world_to_cam = cam_to_world.inverse();

  // Scale intrinsics from their native resolution to the requested image size.
  const double sx = static_cast<double>(image_width) / intrinsics.width;
  const double sy = static_cast<double>(image_height) / intrinsics.height;
  const double fx = intrinsics.fx * sx;
  const double fy = intrinsics.fy * sy;
  const double cx = intrinsics.cx * sx;
  const double cy = intrinsics.cy * sy;

  int x_min = image_width, y_min = image_height, x_max = -1, y_max = -1;
  int in_bounds = 0;

  for (const auto &p : world_points) {
    Eigen::Vector4d pw(p.x(), p.y(), p.z(), 1.0);
    Eigen::Vector4d pc = world_to_cam * pw;
    const double z = pc.z();
    if (!(z > 0.0)) // behind camera or degenerate
      continue;

    const double u = fx * (pc.x() / z) + cx;
    const double v = fy * (pc.y() / z) + cy;
    const int px = static_cast<int>(std::lround(u));
    const int py = static_cast<int>(std::lround(v));
    if (px < 0 || py < 0 || px >= image_width || py >= image_height)
      continue;

    ++in_bounds;
    x_min = std::min(x_min, px);
    y_min = std::min(y_min, py);
    x_max = std::max(x_max, px);
    y_max = std::max(y_max, py);
  }

  if (in_bounds == 0)
    return std::nullopt;

  ProjectedBox box;
  box.x_min = x_min;
  box.y_min = y_min;
  box.x_max = x_max;
  box.y_max = y_max;
  box.in_bounds = in_bounds;
  return box;
}

} // namespace reusex::vision
