// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <ReUseX/core/logging.hpp>
#include <ReUseX/core/processing_observer.hpp>
#include <ReUseX/io/reusex.hpp>
#include <ReUseX/types.hpp>
#include <ReUseX/utils/fmt_formatter.hpp>
#include <pcl/markov_clustering.hpp>
#include <spdmon/spdmon.hpp>

// GraphBLAS imports complex.h which defines a macro named 'I' that conflicts
// with the type in CLI11
#ifdef I
#undef I
#endif

#include <fmt/format.h>


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

#include <atomic>

namespace parameter = boost::parameter;

namespace ReUseX::geometry {
struct SegmentRoomsRequest {
  CloudConstPtr cloud;
  CloudNConstPtr normals;
  CloudLConstPtr planes;

  float grid_size = 0.5F;
  float inflation = 2.0F;
  int expansion = 2;
  float pruning_threshold = 0.0001F;
  float convergence_threshold = 1e-8F;
  int max_iter = 100;
  bool visualize = false;

  core::IProcessingObserver *observer = nullptr;
  const std::atomic_bool *cancel_token = nullptr;
};

auto segment_rooms_impl(const SegmentRoomsRequest &request) -> CloudLPtr;
auto segment_rooms(const SegmentRoomsRequest &request) -> CloudLPtr;

BOOST_PARAMETER_NAME(cloud)
BOOST_PARAMETER_NAME(normals)
BOOST_PARAMETER_NAME(planes)
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
                          (normals, (CloudNConstPtr))             //
                          (planes, (CloudLConstPtr))              //
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
  SegmentRoomsRequest request;
  request.cloud = cloud;
  request.normals = normals;
  request.planes = planes;
  request.grid_size = grid_size;
  request.inflation = inflation;
  request.expansion = expansion;
  request.pruning_threshold = pruning_threshold;
  request.convergence_threshold = convergence_threshold;
  request.max_iter = max_iter;
  request.visualize = visualize;
  return segment_rooms(request);
}

} // namespace ReUseX::geometry
