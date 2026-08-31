// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <pcl/PolygonMesh.h>
#include <pcl/TextureMesh.h>
#include <pcl/common/colors.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "global-params.hpp"

#include <reusex/geometry/BuildingComponent.hpp>

#include <array>
#include <string>
#include <variant>
#include <vector>

namespace rux::view {

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/// A point cloud loaded for visualization with associated metadata.
struct LoadedCloud {
  CloudPtr cloud;      ///< The point cloud data
  std::string name;    ///< Display name in viewer
  bool visible = true; ///< Current visibility state
};

/// A polygon mesh loaded for visualization with associated metadata.
struct LoadedMesh {
  std::variant<pcl::PolygonMesh::Ptr, pcl::TextureMesh::Ptr>
      mesh;            ///< The mesh data (regular or textured)
  std::string name;    ///< Display name in viewer
  bool visible = true; ///< Current visibility state
                       /// Check if this mesh has texture information.
  bool is_textured() const {
    return std::holds_alternative<pcl::TextureMesh::Ptr>(mesh);
  }
};

/// Tracks the currently visible label overlay in the viewer.
struct ViewerState {
  int current_label =
      -1; ///< Index of currently shown label (-1 = none, 0-8 = label index)
  bool legend_visible = false;         ///< Whether the label legend is shown
  std::vector<std::string> legend_ids; ///< Shape IDs for legend text elements
};

/// A single entry in the label legend overlay.
struct LabelLegendEntry {
  int label_id;
  std::string name;
  pcl::RGB color;
};

/// A label cloud with its associated metadata for legend display.
struct LabelCloudInfo {
  pcl::PointCloud<pcl::PointXYZL>::Ptr cloud;
  std::string db_name;                          ///< Original cloud name in DB
  std::vector<LabelLegendEntry> legend_entries; ///< Pre-computed at load time
};

/// Metadata for a panoramic image (pixel data loaded on-demand).
struct PanoramaInfo {
  int id;                      ///< Database row ID
  std::string filename;        ///< Original filename
  int node_id;                 ///< Linked sensor frame (-1 if unmatched)
  bool pose_valid = false;     ///< Whether a 3D pose is available
  double px, py, pz;           ///< Position from sensor frame pose
  std::array<double, 16> pose; ///< Full 4x4 SE(3) row-major world pose
};

} // namespace rux::view
