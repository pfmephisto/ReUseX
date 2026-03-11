// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <ReUseX/types.hpp>

#include <filesystem>
#include <opencv2/core/mat.hpp>
#include <optional>
#include <tuple>

namespace ReUseX::io {

auto import_rtabmap_database(const std::filesystem::path &database_path,
                             float resolution, float min_distance,
                             float max_distance, float sampling_factor)
    -> std::tuple<CloudPtr, CloudNPtr, CloudLPtr>;

bool checkRTABMapDatabase(std::filesystem::path const &dbPath);
bool initRTABMapDatabase(std::filesystem::path const &dbPath);

bool writeLabelsToRTABMapDatabase(std::filesystem::path const &dbPath,
                                  cv::Mat const &labels,
                                  std::optional<size_t> id = std::nullopt);
cv::Mat readLabelsFromRTABMapDatabase(std::filesystem::path const &dbPath,
                                      size_t id);

} // namespace ReUseX::io
