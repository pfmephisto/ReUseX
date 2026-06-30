// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <handlers.hpp>

#include <clients.hpp>

namespace ruxd {

namespace {

struct BackendCheck {
  bool all_ok;
  nlohmann::json detail; // per-backend {ok, detail}
};

BackendCheck check_backends(Clients &clients) {
  const PingResult pg = clients.postgres.ping();
  const PingResult rd = clients.redis.ping();
  const PingResult s3 = clients.s3.ping();
  return {
      pg.ok && rd.ok && s3.ok,
      {
          {"postgres", {{"ok", pg.ok}, {"detail", pg.detail}}},
          {"redis", {{"ok", rd.ok}, {"detail", rd.detail}}},
          {"s3", {{"ok", s3.ok}, {"detail", s3.detail}}},
      },
  };
}

crow::status code_for(bool all_ok) {
  return all_ok ? crow::status::OK : crow::status::SERVICE_UNAVAILABLE;
}

} // namespace

void register_health_routes(App &app, EndpointRegistry &reg,
                            Clients &clients) {
  // Root — simple liveness.
  add_route(app, reg,
            {"GET", "/", "Liveness probe", false, {{200, "Service is up"}}},
            [](const crow::request &) {
              return json_response(crow::status::OK, {{"status", "ok"}});
            });

  // Kubernetes liveness — only that the process is up. Deliberately does NOT
  // touch backends, so a transient backend outage cannot trigger pod restarts.
  add_route(app, reg,
            {"GET", "/livez", "Kubernetes liveness probe", false,
             {{200, "Process alive"}}},
            [](const crow::request &) {
              return json_response(crow::status::OK, {{"status", "ok"}});
            });

  // Kubernetes readiness — ready to serve only when every backend is
  // reachable. Unauthenticated (probes don't carry credentials); status only,
  // no per-backend detail.
  add_route(app, reg,
            {"GET", "/readyz", "Kubernetes readiness probe", false,
             {{200, "All backends reachable"},
              {503, "One or more backends unreachable"}}},
            [&clients](const crow::request &) {
              const BackendCheck c = check_backends(clients);
              return json_response(
                  code_for(c.all_ok),
                  {{"status", c.all_ok ? "ready" : "not_ready"}});
            });

  // Health — public summary that reflects degraded state to everyone (ok /
  // degraded + status code) but exposes per-backend detail only to
  // authenticated callers.
  add_route(
      app, reg,
      {"GET", "/health",
       "Service health (per-backend detail when authenticated)", false,
       {{200, "All backends reachable"},
        {503, "One or more backends unreachable"}}},
      [&app, &clients](const crow::request &req) {
        const BackendCheck c = check_backends(clients);
        nlohmann::json body{{"status", c.all_ok ? "ok" : "degraded"}};
        if (app.get_context<BearerAuthMiddleware>(req).authenticated) {
          body.update(c.detail);
        }
        return json_response(code_for(c.all_ok), body);
      });
}

} // namespace ruxd
