// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <ReUseX/core/logging.hpp>
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

namespace parameter = boost::parameter;

namespace ReUseX::geometry {

auto segment_planes_impl(CloudConstPtr cloud, CloudNConstPtr normals,
                         const float angle_threshold,
                         const float plane_dist_threshold,
                         const int min_inliers, const float radius,
                         const float interval_0, const float interval_factor,
                         const bool visualize)
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
  return segment_planes_impl(cloud, normals, angle_threshold,
                             plane_dist_threshold, min_inliers, radius,
                             interval_0, interval_factor, visualize);
}

} // namespace ReUseX::geometry
