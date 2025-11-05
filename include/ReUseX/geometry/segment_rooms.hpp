// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "ReUseX/io/reusex.hpp"
#include "ReUseX/types.hpp"
#include "ReUseX/utils/fmt_formatter.hpp"
#include "pcl/markov_clustering.hpp"
#include "spdmon/spdmon.hpp"

// GraphBLAS imports complex.h which defines a macro named 'I' that conflicts
// with the type in CLI11
#ifdef I
#undef I
#endif

#include <fmt/format.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <pcl/common/pca.h>
#include <pcl/correspondence.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace fs = std::filesystem;

namespace ReUseX::geometry {
auto bar(CloudConstPtr cloud, CloudNPtr normals, const float grid_size,
         const float inflation, const float expansion,
         const float pruning_threshold, const float convergence_threshold,
         const int max_iter, const bool visualize) -> CloudLPtr;
}
