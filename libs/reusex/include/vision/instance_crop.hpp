// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Pinhole projection of a set of world-space points into a sensor frame, and
// selection of the best view + 2D bounding box for cropping an instance out of
// its color image (#373).
//
// The projection math mirrors what vision/project.cpp does with rtabmap
// (world -> base via pose^-1, base -> camera via localTransform^-1, then a
// pinhole divide), but is reimplemented here with plain Eigen so it is small,
// dependency-light and directly unit-testable — no rtabmap CameraModel, no
// z-buffer. We only need where the points land, not a full depth render.

#include "reusex/core/SensorIntrinsics.hpp"

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

namespace reusex::vision {

/// A projected 2D bounding box of an instance in one frame, with the count of
/// points that landed inside the image.
struct ProjectedBox {
  int x_min = 0, y_min = 0, x_max = 0, y_max = 0;
  int in_bounds = 0; ///< Number of points projected inside the image.
};

/// Project @p world_points into the camera described by @p pose (row-major 4x4
/// base-to-world) and @p intrinsics, returning the pixel bounding box of the
/// points that fall in front of the camera AND inside the image.
///
/// Returns nullopt when no point projects in-bounds. Deterministic: no
/// randomness, results depend only on the inputs.
///
/// @param image_width/@param image_height  Target image size the box is
///        clamped to; intrinsics are scaled from their native size to this.
std::optional<ProjectedBox>
project_points_to_box(const std::vector<Eigen::Vector3f> &world_points,
                      const std::array<double, 16> &pose,
                      const core::SensorIntrinsics &intrinsics, int image_width,
                      int image_height);

} // namespace reusex::vision
