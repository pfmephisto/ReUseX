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
 * - meshes.mymesh → get mesh data (binary PLY)
 * - meshes.mymesh.metadata → get metadata (JSON)
 * - meshes[0] → get first mesh (binary PLY)
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
  std::vector<uint8_t> get_mesh_binary(std::string_view name) const;
};

} // namespace rux::database
