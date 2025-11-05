// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "pcl/planar_region_growing.hpp"
#include "rux/segment/planes.hpp"

#include "ReUseX/utils/fmt_formatter.hpp"
#include "spdmon/spdmon.hpp"

#include <fmt/format.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <pcl/common/colors.h>
#include <pcl/filters/filter.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "ReUseX/io/reusex.hpp"
namespace fs = std::filesystem;

namespace ReUseX::geometry {
auto foo(CloudConstPtr cloud, CloudNConstPtr normals,
         const float angle_threshold, const float plane_dist_threshold,
         const int minInliers, const float radius, const float interval_0,
         const float interval_factor, const bool visualize) -> CloudLPtr;
}
