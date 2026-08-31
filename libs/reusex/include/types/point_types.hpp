// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// PCL point-cloud type aliases used throughout the library. Include this
// (rather than the <reusex/types.hpp> umbrella) when a TU needs the point-cloud
// containers but not the Eigen helpers, so unrelated TUs are not forced to pull
// in PCL.

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace reusex {

using PointT = pcl::PointXYZRGB;
using NormalT = pcl::Normal;
using LabelT = pcl::Label;
using LocT = pcl::PointXYZ;

using Indices = pcl::Indices;
using IndicesPtr = pcl::IndicesPtr;
using IndicesConstPtr = pcl::IndicesConstPtr;

using Cloud = pcl::PointCloud<PointT>;
using CloudPtr = typename Cloud::Ptr;
using CloudConstPtr = typename Cloud::ConstPtr;

using CloudN = pcl::PointCloud<NormalT>;
using CloudNPtr = typename CloudN::Ptr;
using CloudNConstPtr = typename CloudN::ConstPtr;

using CloudL = pcl::PointCloud<LabelT>;
using CloudLPtr = typename CloudL::Ptr;
using CloudLConstPtr = typename CloudL::ConstPtr;

using CloudLoc = pcl::PointCloud<LocT>;
using CloudLocPtr = typename CloudLoc::Ptr;
using CloudLocConstPtr = typename CloudLoc::ConstPtr;

} // namespace reusex
