// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/types.hpp"

#include <filesystem>
#include <tuple>

namespace ReUseX {
class ProjectDB;
}

namespace ReUseX::io {

auto import_rtabmap_database(const std::filesystem::path &database_path,
                             float resolution, float min_distance,
                             float max_distance, float sampling_factor)
    -> std::tuple<CloudPtr, CloudNPtr, CloudLPtr>;

/**
 * @brief Import sensor frame color images from an RTABMap database into
 * ProjectDB
 *
 * Reads JPEG images from RTABMap's Data table, applies 90 deg CW rotation
 * (RTABMap convention), and stores them as pre-rotated JPEGs in
 * sensor_frames.
 *
 * @param projectDb Target project database (must be open in write mode)
 * @param rtabmapDbPath Path to the RTABMap .db file
 */
void import_sensor_frames(ProjectDB &projectDb,
                          const std::filesystem::path &rtabmapDbPath);

} // namespace ReUseX::io
