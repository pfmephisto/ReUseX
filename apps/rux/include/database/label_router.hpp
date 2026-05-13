// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "resource_router.hpp"

namespace rux::database {

/**
 * @brief Router for segmentation_images (read-only)
 *
 * Each entry is a per-frame semantic label raster keyed by sensor-frame
 * node_id. The in-memory format is CV_32S with -1 for background and 0+ for
 * class ids.
 *
 * Handles paths like:
 * - labels                 → list all node_ids (JSON array)
 * - labels.<id>            → PNG of the raster, stored CV_16U +1 offset
 *                            (background = 0). Use `.metadata` for headers.
 * - labels[N]              → same as above, by index
 * - labels.<id>.metadata   → JSON with node_id, width, height, num_classes
 * - labels.<id>.image      → PNG of the raster (same as default)
 */
class LabelRouter : public ResourceRouter {
    public:
  using ResourceRouter::ResourceRouter;

  DataPayload get(const std::vector<PathComponent> &components) override;
  void set(const std::vector<PathComponent> &components,
           const DataPayload &data) override;
  void del(const std::vector<PathComponent> &components) override;
  std::vector<std::string> list() const override;

    private:
  int resolve_node_id(const PathComponent &component) const;
  std::vector<uint8_t> encoded_image(int nodeId) const;
  nlohmann::json metadata_json(int nodeId) const;
};

} // namespace rux::database
