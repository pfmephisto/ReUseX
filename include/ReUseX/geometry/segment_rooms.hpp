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

#include <boost/parameter.hpp>
#include <boost/parameter/keyword.hpp>
#include <boost/parameter/name.hpp>

namespace parameter = boost::parameter;
namespace fs = std::filesystem;

namespace ReUseX::geometry {
auto segment_rooms_impl(CloudConstPtr cloud, CloudNPtr normals,
                        const float grid_size, const float inflation,
                        const float expansion, const float pruning_threshold,
                        const float convergence_threshold, const int max_iter,
                        const bool visualize) -> CloudLPtr;

BOOST_PARAMETER_NAME(cloud)
BOOST_PARAMETER_NAME(normals)
BOOST_PARAMETER_NAME(grid_size)
BOOST_PARAMETER_NAME(inflation)
BOOST_PARAMETER_NAME(expansion)
BOOST_PARAMETER_NAME(pruning_threshold)
BOOST_PARAMETER_NAME(convergence_threshold)
BOOST_PARAMETER_NAME(max_iter)
BOOST_PARAMETER_NAME(visualize)

BOOST_PARAMETER_FUNCTION((CloudLPtr),   // 1. parenthesized return type
                         segment_rooms, // 2. name of the function template
                         tag,           // 3. namespace of tag types
                         (required      //
                          (cloud, (CloudConstPtr))                //
                          (normals, (CloudNPtr))                  //
                          )                                       //
                         (optional                                //
                          (grid_size, (double), 0.5)              //
                          (inflation, (double), 2.0)              //
                          (expansion, (int), 2)                   //
                          (pruning_threshold, (double), 0.0001)   //
                          (convergence_threshold, (double), 1e-8) //
                          (max_iter, (int), 100)                  //
                          (visualize, (bool), false)              //
                          )) {
  return segment_rooms_impl(cloud, normals, grid_size, inflation, expansion,
                            pruning_threshold, convergence_threshold, max_iter,
                            visualize);
}

} // namespace ReUseX::geometry
