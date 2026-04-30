// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/types.hpp"
#include "reusex/core/SensorIntrinsics.hpp"

#include <pcl/PolygonMesh.h>
#include <pcl/TextureMesh.h>
#include <rtabmap/core/DBDriver.h>
#include <opencv2/core/mat.hpp>

#include <map>

namespace reusex::geometry {

/// Camera data for texture mapping
struct CameraData {
  cv::Mat image;  ///< Color image
  core::SensorIntrinsics intrinsics;  ///< Camera intrinsics
  Eigen::Matrix4d pose;  ///< World pose (SE(3))
};

/// Quality parameters for texture projection
struct TextureQualityParams {
  float texels_per_meter = 400.0f;  ///< Target texture detail (pixels per meter) - adaptive resolution
  int min_resolution = 256;      ///< Minimum texture size (small surfaces)
  int max_resolution = 4096;     ///< Maximum texture size (large surfaces)
  int atlas_tile_size = 2048;    ///< Atlas tile size for PCL visualization (lower = less memory)
  float distance_threshold = 0.02f;  ///< Max distance from point to surface (meters) - smaller = sharper
  float search_radius = 0.04f;   ///< K-d tree search radius (meters)
  int max_neighbors = 100;       ///< Max points to check per pixel
  bool use_quadratic_falloff = true;  ///< Use 1/d^2 instead of 1/d for sharper detail
};

pcl::TextureMesh::Ptr texture_mesh_with_cloud(
    pcl::PolygonMesh::Ptr mesh,
    CloudConstPtr cloud,
    CloudNConstPtr normals = nullptr,
    bool debug_distinct_colors = false,
    const TextureQualityParams& quality = TextureQualityParams());

/// Texture mesh using RTABMap signatures (legacy API)
pcl::TextureMesh::Ptr
texture_mesh(pcl::PolygonMesh::Ptr mesh,
             std::map<int, rtabmap::Transform> const &poses,
             std::map<int, rtabmap::Signature> const &nodes);

/// Texture mesh using simple camera data (ProjectDB API)
pcl::TextureMesh::Ptr
texture_mesh(pcl::PolygonMesh::Ptr mesh,
             std::map<int, CameraData> const &cameras);
} // namespace reusex::geometry
