// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <Eigen/Core>

namespace ReUseX {
inline auto planePointDist(const Eigen::Vector4d &plane,
                           const Eigen::Vector3d &point) {
  return (plane.head<3>().dot(point) + plane[3]) /
         plane.head<3>().squaredNorm();
}

} // namespace ReUseX
