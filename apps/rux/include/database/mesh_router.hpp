// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "resource_router.hpp"

namespace rux::database {

/**
 * @brief Router for mesh resources
 *
 * Handles paths like:
 * - meshes → list all meshes (JSON array)
 * - meshes.mymesh → get mesh metadata (JSON)
 * - meshes.mymesh.data → get mesh geometry (binary PLY or OBJ)
 * - meshes.mymesh.material → get MTL text (textured meshes only)
 * - meshes.mymesh.texture → get JPEG texture (textured meshes only)
 * - meshes.mymesh.format → get format string
 * - meshes.mymesh.vertex_count → get vertex count
 * - meshes.mymesh.polygon_count → get polygon count
 * - meshes.mymesh.metadata → get metadata (JSON)
 * - meshes[0] → get first mesh metadata (JSON)
 */
class MeshRouter : public ResourceRouter {
public:
  using ResourceRouter::ResourceRouter;

  DataPayload get(const std::vector<PathComponent> &components) override;
  void set(const std::vector<PathComponent> &components,
           const DataPayload &data) override;
  void del(const std::vector<PathComponent> &components) override;
  std::vector<std::string> list() const override;

private:
  nlohmann::json get_metadata(std::string_view name) const;
  std::vector<uint8_t> get_data(std::string_view name) const;
  std::vector<uint8_t> get_material(std::string_view name) const;
  std::vector<uint8_t> get_texture(std::string_view name) const;

  static std::pair<std::vector<uint8_t>, std::vector<uint8_t>>
  split_obj_mtl_blob(const std::vector<uint8_t> &combined);

  static std::vector<uint8_t>
  rewrite_obj_mtllib(const std::vector<uint8_t> &obj,
                     std::string_view mesh_name);

  static std::vector<uint8_t>
  rewrite_mtl_map_kd(const std::vector<uint8_t> &mtl,
                     std::string_view mesh_name);
};

} // namespace rux::database
