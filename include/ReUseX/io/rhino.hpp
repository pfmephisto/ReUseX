// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once
#include "ReUseX/types.hpp"
#include "ReUseX/utils/fmt_formatter.hpp"
#include "ReUseX/vision/Yolo.hpp"
#include "spdmon/spdmon.hpp"

#include <fmt/format.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <pcl/common/colors.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>

#include <opennurbs_public.h>

#include <opennurbs_layer.h>
#include <opennurbs_pointcloud.h>

#include <filesystem>
#include <optional>
#include <ranges>
#include <set>

namespace fs = std::filesystem;

namespace ReUseX::io {
auto configure_rhino_model() -> std::unique_ptr<ONX_Model>;

auto create_rhino_layers(ONX_Model &model,
                         const std::set<std::string> &layer_names,
                         std::optional<std::vector<ON_Color>> layer_colors = {},
                         const ON_Layer *base_layer = nullptr)
    -> std::vector<int>;

auto make_rhino_pointcloud(CloudConstPtr cloud)
    -> std::unique_ptr<ON_PointCloud>;

[[nodiscard]]
auto save_rhino_pointcloud(CloudConstPtr pcl_cloud, CloudLConstPtr pcl_labels)
    -> std::unique_ptr<ONX_Model>;
} // namespace ReUseX::io
