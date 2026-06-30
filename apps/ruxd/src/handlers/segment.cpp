// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <handlers.hpp>

#include <reusex/core/logging.hpp>

namespace ruxd {

void register_segment_routes(App &app, EndpointRegistry &reg) {
  // Point cloud plane segmentation. Not yet implemented — logs the request and
  // returns a stub response. The future implementation will deserialize the
  // request payload and call reusex::geometry::segment_planes(...). Marked as
  // requiring authentication so the auth requirement is captured in the docs
  // (the 401 response is added automatically by the registry).
  add_route(
      app, reg,
      {"POST",
       "/segment/planes",
       "Run point cloud plane segmentation",
       true,
       {{202, "Segmentation accepted"},
        {501, "Not implemented yet"}}},
      [](const crow::request &req) {
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
