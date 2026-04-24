// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once
#include "reusex/types.hpp"

// OpenNURBS public needs to be included first (pulls in all ON_* types)
#include <opennurbs_public.h>

#include <pcl/PolygonMesh.h>

#include <memory>

namespace reusex::io {

// Forward declaration
struct ExportScene;

/// Create a configured ONX_Model with ReUseX metadata, units, and tolerances.
auto configure_rhino_model() -> std::unique_ptr<ONX_Model>;

/// Export a full ExportScene to a Rhino ONX_Model with layer hierarchy.
auto export_to_rhino(const ExportScene &scene) -> std::unique_ptr<ONX_Model>;

/// Convert a PCL XYZRGB point cloud to an ON_PointCloud.
auto make_rhino_pointcloud(CloudConstPtr cloud)
    -> std::unique_ptr<ON_PointCloud>;

/// Convert a PCL XYZRGB point cloud with normals to an ON_PointCloud.
auto make_rhino_pointcloud(CloudConstPtr cloud, CloudNConstPtr normals)
    -> std::unique_ptr<ON_PointCloud>;

/// Convert a PCL PolygonMesh to an ON_Mesh.
auto make_rhino_mesh(const pcl::PolygonMesh &mesh) -> std::unique_ptr<ON_Mesh>;

/// Create a UV sphere mesh.
auto make_sphere_mesh(double cx, double cy, double cz, double radius,
                      int resolution = 16) -> std::unique_ptr<ON_Mesh>;

} // namespace reusex::io
