// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "reusex/geometry/BuildingComponent.hpp"
#include "reusex/types.hpp"

#include <pcl/PolygonMesh.h>

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::io {

/// Intermediate representation for exporting all project data.
/// Used by both Rhino and Speckle exporters.
struct ExportScene {

  // --- Cloud ---
  struct CloudLayer {
    CloudConstPtr cloud;    ///< XYZRGB (always present if layer exists)
    CloudNConstPtr normals; ///< may be nullptr
  };
  std::optional<CloudLayer> cloud;

  // --- Semantic ---
  struct SemanticInstance {
    uint32_t instance_id = 0; ///< 0 if no instance segmentation
    CloudConstPtr points;
  };
  struct SemanticCategory {
    std::string name;                  ///< e.g., "wall", "ceiling"
    int label_id = 0;                  ///< semantic label ID
    std::array<uint8_t, 3> color = {}; ///< Glasbey RGB
    std::vector<SemanticInstance> instances;
  };
  std::vector<SemanticCategory> semantic;

  // --- Meshes ---
  struct MeshEntry {
    std::string name;
    pcl::PolygonMesh::Ptr mesh;
  };
  std::vector<MeshEntry> meshes;

  // --- 360 Panoramas ---
  struct PanoEntry {
    std::string image_name;     ///< filename
    std::string image_url;      ///< empty placeholder for future
    double x = 0, y = 0, z = 0; ///< position from sensor frame pose
  };
  std::vector<PanoEntry> panoramas;

  // --- Materials ---
  struct MaterialEntry {
    std::string name;           ///< designation or GUID
    std::string image_filename; ///< original photo filename, empty if unknown
    double x = 0, y = 0, z = 0; ///< translation from linked sensor frame
    /// Row-major 4x4 world pose of the linked sensor frame (identity if
    /// no frame is linked). Translation also mirrored to x/y/z for the
    /// translation-only Rhino exporter.
    std::array<double, 16> transform{1, 0, 0, 0, 0, 1, 0, 0,
                                     0, 0, 1, 0, 0, 0, 0, 1};
    std::map<std::string, std::string> properties;
  };
  std::vector<MaterialEntry> materials;

  // --- Building Components ---
  struct ComponentEntry {
    std::string name;
    geometry::ComponentType type;
    geometry::CoplanarPolygon boundary;
    double confidence = -1.0;
    std::string notes;
    std::map<std::string, std::string> properties;
  };
  std::vector<ComponentEntry> components;
};

/// Gather all exportable data from a ProjectDB.
/// Only populates layers that have data.
ExportScene gather_export_scene(const ProjectDB &db);

} // namespace reusex::io
