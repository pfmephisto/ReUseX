// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Backend client wrappers for the ruxd service worker: PostgreSQL (libpqxx),
// Redis (redis-plus-plus) and S3 object storage (aws-sdk-cpp). Each connects
// lazily and exposes a ping() used by the /ready probe. Heavy third-party
// headers are kept out via the Pimpl idiom / forward declarations so this
// header stays cheap to include.

#include "config.hpp"
#include "connection_pool.hpp"

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

// Forward declaration to avoid pulling redis-plus-plus headers in here.
namespace sw::redis {
class Redis;
}

// Forward declaration to avoid pulling libpqxx headers in here. Only
// declarations (references, unique_ptr members) use the type below; anything
// that dereferences a pooled connection must include <pqxx/pqxx> itself.
namespace pqxx {
class connection;
}

namespace ruxd {

// Result of a backend health check.
struct PingResult {
  bool ok = false;
  std::string detail; // human-readable status / error message
};

// PostgreSQL via libpqxx.
//
// A pqxx::connection is not thread-safe and opening one per request is
// wasteful, so query paths lease a connection from a fixed-size pool
// (#201). Pool capacity defaults to the worker-thread count; see
// Config::pg_pool_size. ping() deliberately stays outside the pool — see the
// comment on its definition in src/clients/postgres.cpp.
class PostgresClient {
    public:
  using Lease = ConnectionPool<pqxx::connection>::Lease;

  explicit PostgresClient(const Config &cfg);
  ~PostgresClient();

  PostgresClient(const PostgresClient &) = delete;
  PostgresClient &operator=(const PostgresClient &) = delete;

  // Readiness probe. Opens (and closes) its own connection.
  [[nodiscard]] PingResult ping() const;

  [[nodiscard]] bool is_configured() const { return !dsn_.empty(); }

  // Leases a pooled connection for the duration of one query. Blocks while the
  // pool is exhausted and throws ConnectionPoolTimeout on timeout, or
  // std::runtime_error when Postgres is not configured. Prefer
  // with_postgres() below, which handles marking torn connections broken.
  [[nodiscard]] Lease acquire();

  [[nodiscard]] std::size_t pool_capacity() const;

    private:
  struct Impl;

  std::string dsn_;
  std::unique_ptr<Impl> impl_;
};

// Runs `fn` with a pooled connection and returns its result. If `fn` throws —
// a pqxx::broken_connection, say — the lease is marked broken so the pool
// discards that connection instead of recycling a torn one, and the exception
// propagates unchanged.
//
// Callers must include <pqxx/pqxx>: dereferencing the lease needs the complete
// type.
template <class F>
decltype(auto) with_postgres(PostgresClient &client, F &&fn) {
  PostgresClient::Lease lease = client.acquire();
  try {
    return std::forward<F>(fn)(*lease);
  } catch (...) {
    lease.mark_broken();
    throw;
  }
}

// Redis via redis-plus-plus. sw::redis::Redis is thread-safe and pools
// connections internally, so a single lazily-created instance is shared.
class RedisClient {
    public:
  explicit RedisClient(std::string url);
  ~RedisClient();
  [[nodiscard]] PingResult ping();

    private:
  std::string url_;
  std::mutex init_mutex_;
  std::shared_ptr<sw::redis::Redis> redis_;
};

// RAII wrapper around Aws::InitAPI / Aws::ShutdownAPI. Exactly one instance
// must exist (and outlive every S3Client) for the lifetime of the process.
class AwsApiGuard {
    public:
  AwsApiGuard();
  ~AwsApiGuard();
  AwsApiGuard(const AwsApiGuard &) = delete;
  AwsApiGuard &operator=(const AwsApiGuard &) = delete;

    private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// S3-compatible object storage via aws-sdk-cpp. Requires an AwsApiGuard to be
// alive. The underlying client is thread-safe.
class S3Client {
    public:
  explicit S3Client(const Config &cfg);
  ~S3Client();
  [[nodiscard]] PingResult ping() const;
  [[nodiscard]] bool is_configured() const;

    private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Owns all backend clients. The AwsApiGuard is declared first so it is
// constructed before — and destroyed after — the S3 client.
struct Clients {
  explicit Clients(const Config &cfg);

  AwsApiGuard aws_guard;
  PostgresClient postgres;
  RedisClient redis;
  S3Client s3;
};

} // namespace ruxd
