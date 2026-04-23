// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <reusex/types.hpp>

#include <filesystem>

using PointT = reusex::PointT;
using NormalT = reusex::NormalT;
using LabelT = reusex::LabelT;
using LocT = pcl::PointXYZ;

using Indices = reusex::Indices;
using IndicesPtr = reusex::IndicesPtr;
using IndicesConstPtr = reusex::IndicesConstPtr;

using Cloud = reusex::Cloud;
using CloudPtr = reusex::CloudPtr;
using CloudConstPtr = reusex::CloudConstPtr;

using CloudN = reusex::CloudN;
using CloudNPtr = reusex::CloudNPtr;
using CloudNConstPtr = reusex::CloudNConstPtr;

using CloudL = reusex::CloudL;
using CloudLPtr = reusex::CloudLPtr;
using CloudLConstPtr = reusex::CloudLConstPtr;

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
struct RuxOptions {
  fs::path project_db = fs::current_path() / "project.rux";
};
