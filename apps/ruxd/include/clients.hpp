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

#include <memory>
#include <mutex>
#include <string>

// Forward declaration to avoid pulling redis-plus-plus headers in here.
namespace sw::redis {
class Redis;
}

namespace ruxd {

// Result of a backend health check.
struct PingResult {
  bool ok = false;
  std::string detail; // human-readable status / error message
};

// PostgreSQL via libpqxx. A pqxx::connection is not thread-safe, so ping()
// opens a short-lived connection per call. Real query paths should use a
// connection pool (see TODO below).
class PostgresClient {
public:
  explicit PostgresClient(std::string dsn);
  [[nodiscard]] PingResult ping() const;
  [[nodiscard]] bool is_configured() const { return !dsn_.empty(); }

private:
  std::string dsn_;
};

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
