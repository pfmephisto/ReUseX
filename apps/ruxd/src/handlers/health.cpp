// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <handlers.hpp>

#include <clients.hpp>

namespace ruxd {

void register_health_routes(crow::SimpleApp &app, Clients &clients) {
  // Root — lightweight liveness, reports only that the process is up.
  CROW_ROUTE(app, "/")
  ([] { return json_response(crow::status::OK, {{"status", "ok"}}); });

  // Health — overview of every configured backend. Pings each and reports its
  // status; returns 200 only when all are reachable, otherwise 503 with
  // per-backend detail.
  CROW_ROUTE(app, "/health")
  ([&clients] {
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
        all_ok ? crow::status::OK : crow::status::SERVICE_UNAVAILABLE, body);
  });
}

} // namespace ruxd
