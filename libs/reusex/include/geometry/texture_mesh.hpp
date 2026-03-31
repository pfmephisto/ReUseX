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

namespace ReUseX::geometry {

/// Camera data for texture mapping
struct CameraData {
  cv::Mat image;  ///< Color image
  core::SensorIntrinsics intrinsics;  ///< Camera intrinsics
  Eigen::Matrix4d pose;  ///< World pose (SE(3))
};

pcl::TextureMesh::Ptr texture_mesh_with_cloud(pcl::PolygonMesh::Ptr mesh,
                                              CloudConstPtr cloud);

/// Texture mesh using RTABMap signatures (legacy API)
pcl::TextureMesh::Ptr
texture_mesh(pcl::PolygonMesh::Ptr mesh,
             std::map<int, rtabmap::Transform> const &poses,
             std::map<int, rtabmap::Signature> const &nodes);

/// Texture mesh using simple camera data (ProjectDB API)
pcl::TextureMesh::Ptr
texture_mesh(pcl::PolygonMesh::Ptr mesh,
             std::map<int, CameraData> const &cameras);
} // namespace ReUseX::geometry
