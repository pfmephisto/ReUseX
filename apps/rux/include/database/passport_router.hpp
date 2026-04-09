// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "resource_router.hpp"

namespace rux::database {

/**
 * @brief Router for material passport resources
 *
 * Handles paths like:
 * - materials → list all passports (JSON array of GUIDs)
 * - materials.guid-1234 → get passport data (JSON)
 * - materials[0] → get first passport (JSON)
 */
class PassportRouter : public ResourceRouter {
public:
  using ResourceRouter::ResourceRouter;

  DataPayload get(const std::vector<PathComponent> &components) override;
  void set(const std::vector<PathComponent> &components,
           const DataPayload &data) override;
  void del(const std::vector<PathComponent> &components) override;
  std::vector<std::string> list() const override;

private:
  /**
   * @brief Get passport as JSON
   */
  nlohmann::json get_passport_json(std::string_view guid) const;
};

} // namespace rux::database
