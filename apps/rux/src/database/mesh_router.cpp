// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "database/mesh_router.hpp"

#include <algorithm>
#include <spdlog/spdlog.h>
#include <sstream>
#include <string>

namespace rux::database {

std::vector<std::string> MeshRouter::list() const {
  auto names = db_->list_meshes();
  std::sort(names.begin(), names.end());
  return names;
}

nlohmann::json MeshRouter::get_metadata(std::string_view name) const {
  auto meta = db_->mesh_metadata(name);

  nlohmann::json j;
  j["name"] = meta.name;
  j["format"] = meta.format;
  j["vertex_count"] = meta.vertex_count;
  j["polygon_count"] = meta.face_count;

  if (!meta.stage.empty())
    j["stage"] = meta.stage;
  if (!meta.parameters.empty())
    j["parameters"] = nlohmann::json::parse(meta.parameters, nullptr, false);
  if (!meta.created_at.empty())
    j["created_at"] = meta.created_at;

  // Add texture info for textured meshes
  if (meta.format == "obj_textured") {
    auto textures = db_->mesh_texture_metadata(name);
    if (!textures.empty()) {
      nlohmann::json tex_arr = nlohmann::json::array();
      for (const auto &t : textures) {
        nlohmann::json tj;
        tj["tex_name"] = t.tex_name;
        tj["format"] = t.format;
        tj["width"] = t.width;
        tj["height"] = t.height;
        tex_arr.push_back(std::move(tj));
      }
      j["textures"] = std::move(tex_arr);
    }
  }

  return j;
}

std::pair<std::vector<uint8_t>, std::vector<uint8_t>>
MeshRouter::split_obj_mtl_blob(const std::vector<uint8_t> &combined) {
  // Find null byte separator between OBJ and MTL parts
  auto it = std::find(combined.begin(), combined.end(), '\0');
  if (it == combined.end()) {
    // No separator: entire blob is OBJ, no MTL
    return {combined, {}};
  }
  std::vector<uint8_t> obj(combined.begin(), it);
  std::vector<uint8_t> mtl(it + 1, combined.end());
  return {std::move(obj), std::move(mtl)};
}

std::vector<uint8_t>
MeshRouter::rewrite_obj_mtllib(const std::vector<uint8_t> &obj,
                               std::string_view mesh_name) {
  std::string content(obj.begin(), obj.end());
  std::istringstream in(content);
  std::ostringstream out;
  std::string line;

  while (std::getline(in, line)) {
    if (line.rfind("mtllib ", 0) == 0) {
      out << "mtllib " << mesh_name << ".mtl\n";
    } else {
      out << line << '\n';
    }
  }

  auto s = out.str();
  return {s.begin(), s.end()};
}

std::vector<uint8_t>
MeshRouter::rewrite_mtl_map_kd(const std::vector<uint8_t> &mtl,
                               std::string_view mesh_name) {
  std::string content(mtl.begin(), mtl.end());
  std::istringstream in(content);
  std::ostringstream out;
  std::string line;

  while (std::getline(in, line)) {
    if (line.rfind("map_Kd ", 0) == 0) {
      out << "map_Kd " << mesh_name << "_texture.jpg\n";
    } else {
      out << line << '\n';
    }
  }

  auto s = out.str();
  return {s.begin(), s.end()};
}

std::vector<uint8_t> MeshRouter::get_data(std::string_view name) const {
  auto blob = db_->mesh_data_blob(name);
  auto format = db_->mesh_format(name);

  if (format == "ply_binary") {
    return blob;
  }

  // OBJ textured: split and return OBJ part with rewritten mtllib
  auto [obj, mtl] = split_obj_mtl_blob(blob);
  return rewrite_obj_mtllib(obj, name);
}

std::vector<uint8_t> MeshRouter::get_material(std::string_view name) const {
  auto format = db_->mesh_format(name);
  if (format != "obj_textured") {
    throw std::runtime_error("Mesh '" + std::string(name) +
                             "' is PLY format (no material data)");
  }

  auto blob = db_->mesh_data_blob(name);
  auto [obj, mtl] = split_obj_mtl_blob(blob);

  if (mtl.empty()) {
    throw std::runtime_error("Mesh '" + std::string(name) +
                             "' has no embedded MTL data");
  }

  return rewrite_mtl_map_kd(mtl, name);
}

std::vector<uint8_t> MeshRouter::get_texture(std::string_view name) const {
  auto format = db_->mesh_format(name);
  if (format != "obj_textured") {
    throw std::runtime_error("Mesh '" + std::string(name) +
                             "' is PLY format (no textures)");
  }

  auto textures = db_->mesh_texture_blobs(name);
  if (textures.empty()) {
    throw std::runtime_error("Mesh '" + std::string(name) +
                             "' has no texture data in database");
  }

  // Return the first texture's JPEG data
  return std::move(textures[0].image_data);
}

DataPayload MeshRouter::get(const std::vector<PathComponent> &components) {
  if (components.empty()) {
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

  if (!db_->has_mesh(item_name)) {
    throw std::runtime_error("Mesh not found: " + item_name);
  }

  if (components.size() == 1) {
    // meshes.X → return metadata (JSON)
    return get_metadata(item_name);
  }

  // Property access
  const auto &prop = components[1].value;

  if (prop == "data") {
    return get_data(item_name);
  } else if (prop == "material") {
    return get_material(item_name);
  } else if (prop == "texture") {
    return get_texture(item_name);
  } else if (prop == "metadata") {
    return get_metadata(item_name);
  } else if (prop == "format") {
    return db_->mesh_format(item_name);
  } else if (prop == "vertex_count") {
    auto meta = db_->mesh_metadata(item_name);
    return std::to_string(meta.vertex_count);
  } else if (prop == "polygon_count") {
    auto meta = db_->mesh_metadata(item_name);
    return std::to_string(meta.face_count);
  } else {
    throw std::runtime_error(
        "Unknown property: " + prop +
        "\nAvailable properties: data, material, texture, metadata, "
        "format, vertex_count, polygon_count");
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
