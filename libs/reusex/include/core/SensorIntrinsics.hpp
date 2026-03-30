// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <array>
#include <string>

namespace ReUseX::core {

/// Lightweight camera intrinsics replacing rtabmap::CameraModel in downstream
/// code. Stores pinhole parameters plus the 4x4 camera-to-base transform.
struct SensorIntrinsics {
  double fx = 0, fy = 0, cx = 0, cy = 0;
  int width = 0, height = 0;
  /// 4x4 row-major local transform (camera frame to robot/sensor base frame).
  std::array<double, 16> local_transform = {1, 0, 0, 0, 0, 1, 0, 0,
                                             0, 0, 1, 0, 0, 0, 0, 1};

  /// Serialize to JSON string for storage in ProjectDB.
  std::string to_json() const;

  /// Deserialize from JSON string. Returns default-constructed on failure.
  static SensorIntrinsics from_json(const std::string &json);
};

} // namespace ReUseX::core
