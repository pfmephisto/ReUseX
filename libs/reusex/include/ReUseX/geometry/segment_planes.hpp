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

#include <ReUseX/io/reusex.hpp>

#include <atomic>

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

  // Optional cancellation flag. Caller retains ownership and must keep this
  // alive for the full duration of the segment_planes(...) call.
  const std::atomic_bool *cancel_token = nullptr;
};

auto segment_planes_impl(const SegmentPlanesRequest &request)
    -> std::tuple<CloudLPtr, CloudLocPtr, CloudNPtr>;

auto segment_planes(const SegmentPlanesRequest &request)
    -> std::tuple<CloudLPtr, CloudLocPtr, CloudNPtr>;

} // namespace ReUseX::geometry
