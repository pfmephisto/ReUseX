// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "processing_observer.hpp"
#include "view/viewer_types.hpp"

#include <reusex/geometry/BuildingComponent.hpp>

#include <filesystem>
#include <string>
#include <vector>

namespace rux::view {

namespace fs = std::filesystem;

/// Result of loading geometry from a .rux project database.
struct ProjectLoadResult {
  std::vector<LoadedCloud> clouds;
  std::vector<LoadedMesh> meshes;
  std::vector<LabelCloudInfo> labels;
  std::vector<reusex::geometry::BuildingComponent> components;
  std::vector<PanoramaInfo> panoramas;
};

/// Load all viewable geometry from a ReUseX project database.
///
/// @param path Filesystem path to .rux file
/// @param observer Visualization observer for enqueueing viewer tasks
/// @param filter_expr Optional filter expression to limit visualization
/// @return ProjectLoadResult containing loaded clouds, meshes, and labels
ProjectLoadResult load_from_project_db(const fs::path &path,
                                       rux::VizualizationObserver &observer,
                                       const std::string &filter_expr);

} // namespace rux::view
