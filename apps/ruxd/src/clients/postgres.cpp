// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <clients.hpp>

#include <pqxx/pqxx>

#include <utility>

namespace ruxd {

PostgresClient::PostgresClient(std::string dsn) : dsn_(std::move(dsn)) {}

PingResult PostgresClient::ping() const {
  if (!is_configured()) {
    return {false, "not configured"};
  }
  try {
    // TODO: Use a connection pool for the real query paths
    // category=CLI estimate=4h
    // pqxx::connection is not thread-safe and opening one per request is
    // wasteful under load. Introduce a small pool (one connection per worker
    // thread, or a fixed-size pool guarded by a mutex/condition variable) and
    // hand out connections to handlers. ping() opens a short-lived connection,
    // which is fine for a readiness probe but not for hot paths.
    pqxx::connection conn(dsn_);
    pqxx::work tx(conn);
    tx.exec("SELECT 1");
    tx.commit();
    return {true, "ok"};
  } catch (const std::exception &e) {
    return {false, e.what()};
  }
}

} // namespace ruxd
