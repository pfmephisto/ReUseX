// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "resource_router.hpp"

namespace rux::database {

/**
 * @brief Router for pipeline_log entries (read-only)
 *
 * Handles paths like:
 * - log              → list all entries (JSON array, newest first)
 * - log[0]           → most recent entry as JSON
 * - log.<id>         → entry with that numeric id (JSON)
 * - log.<id>.<prop>  → individual fields:
 *                       stage, started_at, finished_at, parameters,
 *                       status, error_msg
 */
class LogRouter : public ResourceRouter {
    public:
  using ResourceRouter::ResourceRouter;

  DataPayload get(const std::vector<PathComponent> &components) override;
  void set(const std::vector<PathComponent> &components,
           const DataPayload &data) override;
  void del(const std::vector<PathComponent> &components) override;
  std::vector<std::string> list() const override;
};

} // namespace rux::database
