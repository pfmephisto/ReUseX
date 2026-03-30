// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <filesystem>

namespace ReUseX {
class ProjectDB;
}

namespace ReUseX::io {

/// Import raw sensor data from an RTABMap database into a ProjectDB.
///
/// For each node the function extracts color, depth, confidence, the optimized
/// world pose, and camera intrinsics, applies the 90 deg CW rotation required
/// by the RTABMap convention, and stores everything in the project database.
///
/// No heavy processing (voxelization, normal estimation, etc.) is performed.
/// Point cloud generation is handled separately by
/// ReUseX::geometry::reconstruct_point_clouds().
///
/// @param db  Target project database (must be open in write mode).
/// @param rtabmap_db_path  Path to the source RTABMap .db file.
void import_rtabmap(ProjectDB &db,
                    const std::filesystem::path &rtabmap_db_path);

} // namespace ReUseX::io
