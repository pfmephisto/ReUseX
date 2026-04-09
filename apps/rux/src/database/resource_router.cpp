// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "database/resource_router.hpp"

#include "database/cloud_router.hpp"
#include "database/mesh_router.hpp"
#include "database/passport_router.hpp"
#include "database/project_router.hpp"

#include <spdlog/spdlog.h>

namespace rux::database {

RouterRegistry::RouterRegistry(std::shared_ptr<ReUseX::ProjectDB> db)
    : db_(db) {
  // Register routers for each collection type
  routers_["clouds"] = std::make_unique<CloudRouter>(db);
  routers_["meshes"] = std::make_unique<MeshRouter>(db);
  routers_["projects"] = std::make_unique<ProjectRouter>(db);
  routers_["materials"] = std::make_unique<PassportRouter>(db);

  // TODO: Add routers for frames, labels, log
  // routers_["frames"] = std::make_unique<FrameRouter>(db);
  // routers_["labels"] = std::make_unique<LabelRouter>(db);
  // routers_["log"] = std::make_unique<LogRouter>(db);

  spdlog::trace("Initialized router registry with {} routers", routers_.size());
}

ResourceRouter &RouterRegistry::get_router(std::string_view collection) {
  auto it = routers_.find(std::string(collection));
  if (it == routers_.end()) {
    throw std::runtime_error(
        "No router for collection: " + std::string(collection) +
        "\nSupported collections: clouds, materials, meshes, projects");
  }
  return *it->second;
}

std::vector<std::vector<PathComponent>>
expand_wildcards(const std::vector<PathComponent> &components,
                 ResourceRouter &router) {
  // Check if there are wildcards to expand
  bool has_wildcard = false;
  for (const auto &comp : components) {
    if (comp.has_wildcard()) {
      has_wildcard = true;
      break;
    }
  }

  if (!has_wildcard) {
    // No wildcards, return as-is
    return {components};
  }

  // Expand wildcards (only in item position)
  if (components.empty() || !components[0].is_item() ||
      !components[0].has_wildcard()) {
    // Wildcard not in expected position
    return {components};
  }

  // Get all items and expand
  auto items = router.list();
  auto matches = expand_wildcard(components[0].value, items);

  if (matches.empty()) {
    spdlog::warn("Wildcard pattern '{}' matched no items", components[0].value);
    return {};
  }

  spdlog::debug("Wildcard pattern '{}' matched {} items", components[0].value,
                matches.size());

  // Create expanded paths
  std::vector<std::vector<PathComponent>> expanded_paths;
  expanded_paths.reserve(matches.size());

  for (const auto &match : matches) {
    std::vector<PathComponent> path = components;
    path[0].value = match; // Replace wildcard with actual name
    expanded_paths.push_back(std::move(path));
  }

  return expanded_paths;
}

} // namespace rux::database
