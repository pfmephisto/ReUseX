// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <clients.hpp>
#include <connection_pool.hpp>

#include <pqxx/pqxx>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <thread>
#include <utility>

namespace ruxd {

namespace {

// Pool capacity: explicit override wins, otherwise one connection per worker
// thread. Crow's "auto" concurrency is hardware_concurrency(), so mirror that;
// never fall below 1.
std::size_t resolve_pool_size(const Config &cfg) {
  if (cfg.pg_pool_size > 0) {
    return cfg.pg_pool_size;
  }
  if (cfg.threads > 0) {
    return cfg.threads;
  }
  return std::max(1u, std::thread::hardware_concurrency());
}

} // namespace

// The pool lives behind a pimpl so clients.hpp stays free of <pqxx/pqxx>.
struct PostgresClient::Impl {
  ConnectionPool<pqxx::connection> pool;

  Impl(const std::string &dsn, ConnectionPoolOptions options)
      : pool([dsn] { return std::make_unique<pqxx::connection>(dsn); },
             std::move(options),
             // Recycle only connections the driver still considers open; a
             // closed one is destroyed and its slot refilled lazily.
             [](pqxx::connection &conn) { return conn.is_open(); }) {}
};

PostgresClient::PostgresClient(const Config &cfg)
    : dsn_(cfg.pg_url),
      impl_(std::make_unique<Impl>(
          cfg.pg_url, ConnectionPoolOptions{
                          resolve_pool_size(cfg),
                          std::chrono::milliseconds(cfg.pg_acquire_timeout_ms),
                          "postgres"})) {
  if (is_configured()) {
    spdlog::info("postgres: connection pool capacity {} (acquire timeout {}ms)",
                 impl_->pool.capacity(), cfg.pg_acquire_timeout_ms);
  }
}

PostgresClient::~PostgresClient() = default;

std::size_t PostgresClient::pool_capacity() const {
  return impl_->pool.capacity();
}

PostgresClient::Lease PostgresClient::acquire() {
  if (!is_configured()) {
    throw std::runtime_error(
        "postgres: not configured (no --pg-url / DATABASE_URL)");
  }
  return impl_->pool.acquire();
}

PingResult PostgresClient::ping() const {
  if (!is_configured()) {
    return {false, "not configured"};
  }
  try {
    // Deliberately NOT pooled (#201). /readyz must answer whether Postgres is
    // reachable *right now*: a pooled connection was established earlier and
    // may be stale, so a successful query over it proves less than a fresh
    // handshake. Keeping probes out of the pool also stops a probe storm from
    // starving real request traffic of the fixed number of slots. Query paths
    // use acquire() / with_postgres() instead.
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
