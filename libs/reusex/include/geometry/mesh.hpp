// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/types.hpp"

#include <pcl/PolygonMesh.h>
#include <pcl/common/common.h>

#include <memory>

namespace reusex::geometry {

/**
 * @brief Options for mesh generation.
 */
struct MeshOptions {
  float search_threshold = 10.0f;   ///< Search threshold for mesh generation
  float new_plane_offset = 0.05f;   ///< Offset for new plane creation
  IndicesConstPtr filter = nullptr; ///< Optional filter to limit processing

  /// MIP solver wall-clock time limit in seconds (issue #212). Guards against
  /// the solver hanging indefinitely on complex buildings.
  double time_limit_seconds = 120.0;

  /// Objective wall-weight passed to the Solidifier (was hardcoded, #212).
  double alpha = 0.04;

  /// Maximum cells the cell complex may generate before failing fast (#213).
  size_t max_cells = 500;
};

/**
 * @brief Generate a mesh from point cloud and geometric primitives.
 *
 * @param cloud Input point cloud.
 * @param normals Point cloud normals.
 * @param planes Detected plane coefficients.
 * @param centroids Plane centroids.
 * @param inliers Indices of points belonging to each plane.
 * @param rooms Room labels for points.
 * @param opt Mesh generation options (includes optional filter).
 * @return Generated polygon mesh.
 */
pcl::PolygonMeshPtr mesh(CloudConstPtr cloud, CloudNConstPtr normals,
                         EigenVectorContainer<double, 4> &planes,
                         EigenVectorContainer<double, 3> &centroids,
                         std::vector<IndicesPtr> &inliers, CloudLConstPtr rooms,
                         MeshOptions const opt = MeshOptions{});
} // namespace reusex::geometry
