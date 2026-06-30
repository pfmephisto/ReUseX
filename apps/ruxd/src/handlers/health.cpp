// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <handlers.hpp>

namespace ruxd {

void register_health_routes(crow::SimpleApp &app) {
  // Liveness probe.
  CROW_ROUTE(app, "/health")
  ([] { return json_response(crow::status::OK, {{"status", "ok"}}); });
}

} // namespace ruxd
