// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <filesystem>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

using PointT = pcl::PointXYZRGB;
using NormalT = pcl::Normal;
using LabelT = pcl::Label;

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

namespace fs = std::filesystem;

/// Collection of all options of Subcommand A.
namespace GlobalParams {
// PointT
const fs::path cloud = fs::current_path() / "cloud.pcd";
// PointN
const fs::path normals = fs::current_path() / "normals.pcd";
// PointL
const fs::path labels = fs::current_path() / "labels.pcd";
const fs::path planes = fs::current_path() / "planes.pcd";
const fs::path rooms = fs::current_path() / "rooms.pcd";

const double resulution = 0.05;
const double grid_size = 0.5;
const bool visualize = false;
}; // namespace GlobalParams
