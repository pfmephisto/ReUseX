// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <handlers.hpp>

namespace ruxd {

void register_meta_routes(App &app, EndpointRegistry &reg) {
  // The handlers capture the registry by reference and read it at request
  // time, so they reflect every route registered before app.run() — including
  // these two meta routes themselves.
  add_route(app, reg,
            {"GET", "/endpoints", "List available endpoints", false,
             {{200, "Endpoint list"}}},
            [&reg](const crow::request &) {
              return json_response(crow::status::OK, reg.to_json());
            });

  add_route(app, reg,
            {"GET", "/openapi.json", "OpenAPI 3.1 specification", false,
             {{200, "OpenAPI document"}}},
            [&reg](const crow::request &) {
              return json_response(crow::status::OK, reg.to_openapi());
            });
}

} // namespace ruxd
