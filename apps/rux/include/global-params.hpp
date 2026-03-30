// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <reusex/types.hpp>

#include <filesystem>

using PointT = ReUseX::PointT;
using NormalT = ReUseX::NormalT;
using LabelT = ReUseX::LabelT;
using LocT = pcl::PointXYZ;

using Indices = ReUseX::Indices;
using IndicesPtr = ReUseX::IndicesPtr;
using IndicesConstPtr = ReUseX::IndicesConstPtr;

using Cloud = ReUseX::Cloud;
using CloudPtr = ReUseX::CloudPtr;
using CloudConstPtr = ReUseX::CloudConstPtr;

using CloudN = ReUseX::CloudN;
using CloudNPtr = ReUseX::CloudNPtr;
using CloudNConstPtr = ReUseX::CloudNConstPtr;

using CloudL = ReUseX::CloudL;
using CloudLPtr = ReUseX::CloudLPtr;
using CloudLConstPtr = ReUseX::CloudLConstPtr;

using CloudLoc = pcl::PointCloud<LocT>;
using CloudLocPtr = typename CloudLoc::Ptr;
using CloudLocConstPtr = typename CloudLoc::ConstPtr;

enum RuxError {
  SUCCESS = 0,
  GENERIC = -1,
  IO = -2,
  INVALID_ARGUMENT = -3,
  NOT_IMPLEMENTED = -4
};

namespace fs = std::filesystem;

/// Collection of all options of Subcommand A.
namespace GlobalParams {

const fs::path db = fs::current_path() / "database.db";
const fs::path project_db = fs::current_path() / "project.rux";

// PointT
const fs::path cloud = fs::current_path() / "cloud.pcd";
// PointN
const fs::path normals = fs::current_path() / "normals.pcd";
// PointL
const fs::path labels = fs::current_path() / "labels.pcd";
const fs::path planes = fs::current_path() / "planes.pcd";
const fs::path rooms = fs::current_path() / "rooms.pcd";

const fs::path plane_centroids = fs::current_path() / "plane_centroids.pcd";
const fs::path plane_normals = fs::current_path() / "plane_normals.pcd";

const double resulution = 0.05;
const double grid_size = 0.5;
}; // namespace GlobalParams
