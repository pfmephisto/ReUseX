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
 * - materials[0] → get passport metadata + stored properties
 * - materials[0].contact_email → get single property value
 */
class PassportRouter : public ResourceRouter {
public:
  using ResourceRouter::ResourceRouter;

  DataPayload get(const std::vector<PathComponent> &components) override;
  void set(const std::vector<PathComponent> &components,
           const DataPayload &data) override;
  void del(const std::vector<PathComponent> &components) override;
  std::vector<std::string> list() const override;
};

} // namespace rux::database
