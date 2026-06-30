// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <handlers.hpp>

#include <clients.hpp>

namespace ruxd {

void register_health_routes(App &app, EndpointRegistry &reg,
                            Clients &clients) {
  // Root — lightweight liveness, reports only that the process is up.
  add_route(app, reg,
            {"GET", "/", "Liveness probe", false, {{200, "Service is up"}}},
            [](const crow::request &) {
              return json_response(crow::status::OK, {{"status", "ok"}});
            });

  // Health — public, but the response depends on authentication:
  //   - unauthenticated: just {"status":"ok"} (no backend detail leaked).
  //   - authenticated:   per-backend overview, 200 when all reachable else 503.
  add_route(
      app, reg,
      {"GET",
       "/health",
       "Service status (per-backend detail when authenticated)",
       false,
       {{200, "OK (always for unauthenticated; all backends reachable)"},
        {503, "One or more backends unreachable (authenticated only)"}}},
      [&app, &clients](const crow::request &req) {
        const bool authed =
            app.get_context<BearerAuthMiddleware>(req).authenticated;
        if (!authed) {
          return json_response(crow::status::OK, {{"status", "ok"}});
        }

        const PingResult pg = clients.postgres.ping();
        const PingResult rd = clients.redis.ping();
        const PingResult s3 = clients.s3.ping();
        const bool all_ok = pg.ok && rd.ok && s3.ok;

        const nlohmann::json body{
            {"status", all_ok ? "ok" : "degraded"},
            {"postgres", {{"ok", pg.ok}, {"detail", pg.detail}}},
            {"redis", {{"ok", rd.ok}, {"detail", rd.detail}}},
            {"s3", {{"ok", s3.ok}, {"detail", s3.detail}}},
        };
        return json_response(
            all_ok ? crow::status::OK : crow::status::SERVICE_UNAVAILABLE,
            body);
      });
}

} // namespace ruxd
