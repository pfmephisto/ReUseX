// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "resource_router.hpp"

namespace rux::database {

/**
 * @brief Router for sensor_frames (read-only)
 *
 * Frames are referenced by integer node_id.
 *
 * Handles paths like:
 * - frames                       → list all node_ids (JSON array)
 * - frames.<id>                  → metadata for one frame (JSON)
 * - frames[N]                    → metadata for the Nth frame (JSON)
 * - frames.<id>.metadata         → metadata (JSON)
 * - frames.<id>.color            → JPEG color image (binary)
 * - frames.<id>.depth            → PNG depth image, CV_16UC1 mm (binary)
 * - frames.<id>.confidence       → PNG confidence image, CV_8UC1 (binary)
 * - frames.<id>.pose             → 4x4 world pose (JSON, row-major)
 * - frames.<id>.intrinsics       → camera intrinsics (JSON)
 * - frames.<id>.timestamp        → epoch seconds (JSON number or null)
 */
class FrameRouter : public ResourceRouter {
    public:
  using ResourceRouter::ResourceRouter;

  DataPayload get(const std::vector<PathComponent> &components) override;
  void set(const std::vector<PathComponent> &components,
           const DataPayload &data) override;
  void del(const std::vector<PathComponent> &components) override;
  std::vector<std::string> list() const override;

    private:
  int resolve_node_id(const PathComponent &component) const;
  nlohmann::json metadata_json(int nodeId) const;
};

} // namespace rux::database
