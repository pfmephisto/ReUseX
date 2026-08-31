// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Eigen-only type aliases. Include this instead of <reusex/types.hpp> when a
// translation unit needs only the linear-algebra helpers (no PCL point-cloud
// containers). Keeping the Eigen and PCL aliases separate avoids force-pulling
// the (heavy) PCL headers into TUs that never touch a point cloud.

#include <Eigen/Core>

#include <utility>
#include <vector>

namespace reusex {

template <typename Scalar, int Rows>
using EigenVectorContainer =
    std::vector<Eigen::Matrix<Scalar, Rows, 1>,
                Eigen::aligned_allocator<Eigen::Matrix<Scalar, Rows, 1>>>;

using Pair = std::pair<Eigen::Vector4d, Eigen::Vector3d>;
using PlanePair = std::pair<Pair, Pair>;

} // namespace reusex
