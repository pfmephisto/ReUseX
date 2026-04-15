// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "reusex/geometry/BuildingComponent.hpp"
#include "reusex/types.hpp"

#include <Eigen/Core>
#include <pcl/PolygonMesh.h>

#include <map>
#include <vector>

namespace ReUseX::geometry {

/// A connected component of approximately-vertical, coplanar mesh faces.
struct WallCandidate {
  Eigen::Vector4d plane;   ///< Hessian normal form [a,b,c,d]: ax+by+cz+d=0
  Eigen::Vector3d centroid; ///< Area-weighted centroid of component faces
  Eigen::Vector3d normal;   ///< Unit outward normal (approximately horizontal)
  std::vector<Eigen::Vector3d> boundary_vertices; ///< Exterior edge vertices
  std::vector<int> face_indices; ///< Mesh face indices in this component
};

/// How to compute the window boundary polygon.
enum class WindowBoundaryMode { rectangle, polyline };

/// Configuration for the create_windows pipeline.
struct CreateWindowsOptions {
  WindowBoundaryMode mode = WindowBoundaryMode::rectangle;
  float wall_offset = 0.5f;            ///< Offset along outward wall normal (meters)
  float alpha = 0.5f;                  ///< ConcaveHull alpha for polyline mode
  float wall_normal_z_threshold = 0.3f; ///< |normal.z| < this → vertical
  float coplanarity_angle_deg = 10.0f;  ///< Max angle deviation within wall component
};

/// Output of create_windows().
struct CreateWindowsResult {
  std::vector<BuildingComponent> components;
  std::vector<int> unmatched_instances; ///< Instance IDs with no wall found
};

/// Extract planar vertical wall candidates from a triangle mesh.
///
/// Decomposes the mesh into connected components of approximately-vertical,
/// coplanar faces. Each component becomes a WallCandidate with a fitted plane,
/// centroid, outward normal, and boundary vertices.
std::vector<WallCandidate>
extract_wall_candidates(const pcl::PolygonMesh &mesh,
                        float normal_z_threshold = 0.3f,
                        float coplanarity_angle_deg = 10.0f);

/// Create window BuildingComponents from instance-labeled points and wall geometry.
///
/// For each window instance, projects its points onto the nearest wall plane,
/// computes a boundary polygon (AABB or concave hull), and offsets it along
/// the outward wall normal.
CreateWindowsResult
create_windows(CloudConstPtr cloud, CloudLConstPtr instance_labels,
               const std::map<uint32_t, uint32_t> &instance_to_semantic,
               const std::vector<WallCandidate> &walls,
               const std::vector<uint32_t> &window_semantic_labels,
               const CreateWindowsOptions &options = {});

} // namespace ReUseX::geometry
