// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "ReUseX/types.hpp"
#include "ReUseX/visualize/Visualizer.hpp"

#include <pcl/PolygonMesh.h>
#include <pcl/common/common.h>

namespace ReUseX::geometry {

struct MeshOptions {
  float search_threshold = 10.0f;
  float new_plane_offset = 0.05f;
};

pcl::PolygonMeshPtr
mesh(CloudConstPtr cloud, CloudNConstPtr normals,
     EigenVectorContainer<double, 4> &planes,
     EigenVectorContainer<double, 3> &centroids,
     std::vector<IndicesPtr> &inliers, CloudLConstPtr rooms,
     MeshOptions const opt = MeshOptions{},
     std::shared_ptr<ReUseX::visualize::Visualizer> viewer = nullptr);
} // namespace ReUseX::geometry
