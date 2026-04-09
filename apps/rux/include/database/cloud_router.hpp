// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "resource_router.hpp"

namespace rux::database {

/**
 * @brief Router for point cloud resources
 *
 * Handles paths like:
 * - clouds → list all clouds (JSON array of names)
 * - clouds.mycloud → get cloud data (binary PCD)
 * - clouds.mycloud.metadata → get metadata (JSON object)
 * - clouds.mycloud.point_count → get point count (text)
 * - clouds.mycloud.type → get cloud type (text)
 * - clouds[0] → get first cloud by name (binary PCD)
 * - clouds[0].metadata → get first cloud metadata (JSON)
 *
 * Metadata properties:
 * - type: Cloud type (PointXYZRGB, Normal, Label, PointXYZ)
 * - point_count: Number of points
 */
class CloudRouter : public ResourceRouter {
public:
  using ResourceRouter::ResourceRouter;

  DataPayload get(const std::vector<PathComponent> &components) override;
  void set(const std::vector<PathComponent> &components,
           const DataPayload &data) override;
  void del(const std::vector<PathComponent> &components) override;
  std::vector<std::string> list() const override;

private:
  /**
   * @brief Get cloud metadata as JSON
   */
  nlohmann::json get_metadata(std::string_view name) const;

  /**
   * @brief Get cloud data as binary PCD
   */
  std::vector<uint8_t> get_cloud_binary(std::string_view name) const;

  /**
   * @brief Set cloud from binary PCD data
   */
  void set_cloud_from_binary(std::string_view name,
                             const std::vector<uint8_t> &data);

  /**
   * @brief Set cloud from text PCD data
   */
  void set_cloud_from_text(std::string_view name, std::string_view data);
};

} // namespace rux::database
