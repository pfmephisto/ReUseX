// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/types.hpp"

namespace ReUseX {
class ProjectDB;
}

namespace ReUseX::vision {
/**
 * @brief Project the labels stored in the database on tho the assebled point
 * cloud.
 *
 * For each view in the database this constructs a croped point cloud, computes
 * a z-buffer and the assigns the precomputed labels to the closses point in the
 * point cloud.
 */
auto project(ProjectDB &db, CloudConstPtr cloud) -> CloudLPtr;
} // namespace ReUseX::vision
