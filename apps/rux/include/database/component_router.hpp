// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "resource_router.hpp"

namespace rux::database {

/**
 * @brief Read-only router for building component resources.
 *
 * Handles paths like:
 * - components        → list all component names (JSON array)
 * - components[0]     → get component fields by index
 * - components.NAME   → get component fields by name
 * - components.NAME.PROP → get a single field (name, guid, type, parent_id,
 *                          confidence, notes, metadata, vertex_count, plane)
 *
 * Writing (set/del) is not supported — components are created by the pipeline.
 */
class ComponentRouter : public ResourceRouter {
    public:
  using ResourceRouter::ResourceRouter;

  DataPayload get(const std::vector<PathComponent> &components) override;
  void set(const std::vector<PathComponent> &components,
           const DataPayload &data) override;
  void del(const std::vector<PathComponent> &components) override;
  std::vector<std::string> list() const override;
};

} // namespace rux::database
