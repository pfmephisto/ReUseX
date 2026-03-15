// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <ReUseX/core/logging.hpp>
#include <ReUseX/core/processing_observer.hpp>
#include <pcl/planar_region_growing.hpp>

#include <ReUseX/types.hpp>
#include <ReUseX/utils/fmt_formatter.hpp>
#include <spdmon/spdmon.hpp>

#include <fmt/format.h>


#include <pcl/common/colors.h>
#include <pcl/filters/filter.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/parameter.hpp>
#include <boost/parameter/keyword.hpp>
#include <boost/parameter/name.hpp>

#include <ReUseX/io/reusex.hpp>

#include <atomic>

namespace parameter = boost::parameter;

namespace ReUseX::geometry {

struct SegmentPlanesRequest {
  CloudConstPtr cloud;
  CloudNConstPtr normals;

  float angle_threshold = 25.0F;
  float plane_dist_threshold = 0.07F;
  int min_inliers = 1000;
  float radius = 0.5F;
  float interval_0 = 16.0F;
  float interval_factor = 1.5F;
  bool visualize = false;

  // Optional observer. Caller retains ownership. If the same observer instance
  // is shared across concurrent pipelines, the observer implementation must be
  // thread-safe. Current segmentation APIs invoke callbacks synchronously and
  // complete all observer calls before returning.
  core::IProcessingObserver *observer = nullptr;
  // Optional cancellation flag. Caller retains ownership and must keep this
  // alive for the full duration of the segment_planes(...) call.
  const std::atomic_bool *cancel_token = nullptr;
};

auto segment_planes_impl(const SegmentPlanesRequest &request)
    -> std::tuple<CloudLPtr, CloudLocPtr, CloudNPtr>;

auto segment_planes(const SegmentPlanesRequest &request)
    -> std::tuple<CloudLPtr, CloudLocPtr, CloudNPtr>;

BOOST_PARAMETER_NAME(cloud)
BOOST_PARAMETER_NAME(normals)
BOOST_PARAMETER_NAME(angle_threshold)
BOOST_PARAMETER_NAME(plane_dist_threshold)
BOOST_PARAMETER_NAME(min_inliers)
BOOST_PARAMETER_NAME(radius)
BOOST_PARAMETER_NAME(interval_0)
BOOST_PARAMETER_NAME(interval_factor)
BOOST_PARAMETER_NAME(visualize)

BOOST_PARAMETER_FUNCTION(
    (std::tuple<CloudLPtr, CloudLocPtr, CloudNPtr>), // 1. parenthesized return
                                                     // type
    segment_planes,                        // 2. name of the function template
    tag,                                   // 3. namespace of tag types
    (required                              //
     (cloud, (CloudConstPtr))              //
     (normals, (CloudNConstPtr))           //
     )                                     //
    (optional                              //
     (angle_threshold, (float), 25.0)      //
     (plane_dist_threshold, (float), 0.07) //
     (min_inliers, (int), 1000)            //
     (radius, (float), 0.5)                //
     (interval_0, (int), 16)               //
     (interval_factor, (float), 1.5)       //
      (visualize, (bool), false)            //
      )) {
  SegmentPlanesRequest request;
  request.cloud = cloud;
  request.normals = normals;
  request.angle_threshold = angle_threshold;
  request.plane_dist_threshold = plane_dist_threshold;
  request.min_inliers = min_inliers;
  request.radius = radius;
  request.interval_0 = interval_0;
  request.interval_factor = interval_factor;
  request.visualize = visualize;
  return segment_planes(request);
}

} // namespace ReUseX::geometry
