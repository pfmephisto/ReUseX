// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "database/mesh_router.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <pcl/io/ply_io.h>
#include <spdlog/spdlog.h>

namespace rux::database {

std::vector<std::string> MeshRouter::list() const {
  auto names = db_->list_meshes();
  std::sort(names.begin(), names.end()); // Alphabetical order
  return names;
}

nlohmann::json MeshRouter::get_metadata(std::string_view name) const {
  if (!db_->has_mesh(name)) {
    throw std::runtime_error("Mesh not found: " + std::string(name));
  }

  nlohmann::json meta;
  meta["name"] = name;

  // Get mesh to extract metadata
  auto mesh = db_->mesh(name);
  if (mesh) {
    meta["vertex_count"] = mesh->cloud.width * mesh->cloud.height;
    meta["polygon_count"] = mesh->polygons.size();
  } else {
    meta["vertex_count"] = 0;
    meta["polygon_count"] = 0;
  }

  return meta;
}

std::vector<uint8_t> MeshRouter::get_mesh_binary(std::string_view name) const {
  if (!db_->has_mesh(name)) {
    throw std::runtime_error("Mesh not found: " + std::string(name));
  }

  auto mesh = db_->mesh(name);
  if (!mesh) {
    throw std::runtime_error("Failed to load mesh: " + std::string(name));
  }

  // Serialize to PLY format using temp file
  auto temp_file = std::filesystem::temp_directory_path() /
                   ("rux_mesh_" + std::string(name) + ".ply");

  if (pcl::io::savePLYFileBinary(temp_file.string(), *mesh) != 0) {
    std::filesystem::remove(temp_file);
    throw std::runtime_error("Failed to serialize mesh: " + std::string(name));
  }

  // Read temp file into buffer
  std::ifstream file(temp_file, std::ios::binary);
  if (!file) {
    std::filesystem::remove(temp_file);
    throw std::runtime_error("Failed to read temp file");
  }

  std::vector<uint8_t> buffer((std::istreambuf_iterator<char>(file)),
                              std::istreambuf_iterator<char>());
  file.close();
  std::filesystem::remove(temp_file);

  return buffer;
}

DataPayload MeshRouter::get(const std::vector<PathComponent> &components) {
  if (components.empty()) {
    // Collection level: return list of mesh names
    auto names = list();
    return nlohmann::json(names);
  }

  // Resolve item name
  std::string item_name;
  if (components[0].is_index()) {
    auto resolved = resolve_index(*components[0].index);
    if (!resolved) {
      throw std::runtime_error("Array index out of range: " +
                               std::to_string(*components[0].index));
    }
    item_name = *resolved;
  } else if (components[0].is_item()) {
    item_name = components[0].value;
  } else {
    throw std::runtime_error("Expected item or index after collection");
  }

  if (components.size() == 1) {
    // meshes.mymesh → return binary mesh data
    return get_mesh_binary(item_name);
  }

  // Property access
  const auto &prop = components[1].value;

  if (prop == "metadata") {
    return get_metadata(item_name);
  } else if (prop == "vertex_count" || prop == "polygon_count") {
    auto meta = get_metadata(item_name);
    return std::to_string(meta[prop].get<size_t>());
  } else {
    throw std::runtime_error(
        "Unknown property: " + prop +
        "\nAvailable properties: metadata, vertex_count, polygon_count");
  }
}

void MeshRouter::set(const std::vector<PathComponent> & /*components*/,
                     const DataPayload & /*data*/) {
  throw std::runtime_error(
      "Mesh set operation not yet implemented. Use 'rux create mesh' instead.");
}

void MeshRouter::del(const std::vector<PathComponent> & /*components*/) {
  throw std::runtime_error(
      "Mesh deletion not yet implemented (no delete API in ProjectDB).");
}

} // namespace rux::database
