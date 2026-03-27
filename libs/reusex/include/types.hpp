// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/core/stages.hpp"

#include <Eigen/Core>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <utility>
#include <vector>

namespace ReUseX {

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

template <typename Scalar, int Rows>
using EigenVectorContainer =
    std::vector<Eigen::Matrix<Scalar, Rows, 1>,
                Eigen::aligned_allocator<Eigen::Matrix<Scalar, Rows, 1>>>;

using Pair = std::pair<Eigen::Vector4d, Eigen::Vector3d>;
using PlanePair = std::pair<Pair, Pair>;

} // namespace ReUseX
