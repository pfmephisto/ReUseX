// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <handlers.hpp>

namespace ruxd {

void register_health_routes(crow::SimpleApp &app) {
  // Liveness probe. Both the root and /health answer (routes must start with
  // '/'; an empty path makes Crow throw std::out_of_range while building its
  // routing trie).
  CROW_ROUTE(app, "/")
  ([] { return json_response(crow::status::OK, {{"status", "ok"}}); });

  CROW_ROUTE(app, "/health")
  ([] { return json_response(crow::status::OK, {{"status", "ok"}}); });
}

} // namespace ruxd
