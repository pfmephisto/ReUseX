// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <handlers.hpp>

#include <reusex/core/logging.hpp>

namespace ruxd {

void register_segment_routes(crow::SimpleApp &app) {
  // Point cloud plane segmentation. Not yet implemented — logs the request and
  // returns a stub response. The future implementation will deserialize the
  // request payload and call reusex::geometry::segment_planes(...).
  CROW_ROUTE(app, "/segment/planes")
      .methods(crow::HTTPMethod::POST)([](const crow::request &req) {
        reusex::core::info(
            "POST /segment/planes — received segmentation request "
            "(not yet implemented), body size = {} bytes",
            req.body.size());
        return json_response(crow::status::NOT_IMPLEMENTED,
                             {{"status", "not_implemented"},
                              {"message", "segmentation not yet wired up"}});
      });
}

} // namespace ruxd
