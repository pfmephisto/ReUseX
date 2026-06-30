// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>
#include <string>

namespace ruxd {

// Runtime configuration for ruxd. Every field is populated from a CLI flag
// bound to an environment variable (see main.cpp). Secrets (credentials) are
// never logged.
struct Config {
  // HTTP server.
  std::uint16_t port = 8080;
  unsigned threads = 0; // 0 = Crow auto (hardware concurrency)

  // PostgreSQL — libpqxx connection string / DSN,
  // e.g. "postgresql://user:pass@host:5432/dbname". Empty = not configured.
  std::string pg_url;

  // Redis — redis-plus-plus URI, e.g. "tcp://127.0.0.1:6379".
  std::string redis_url = "tcp://127.0.0.1:6379";

  // S3 / object storage. Endpoint empty = real AWS (region-derived endpoint);
  // set it for self-hosted S3-compatible servers (MinIO/Ceph/...).
  std::string s3_endpoint;          // e.g. "http://127.0.0.1:9000"
  std::string s3_region = "us-east-1";
  std::string s3_bucket;
  std::string s3_access_key;        // secret
  std::string s3_secret_key;        // secret
  bool s3_path_style = true;        // path-style addressing (MinIO/Ceph need it)

  // Auth — Bearer token required for authenticated routes. Empty disables auth.
  std::string auth_token;           // secret
};

} // namespace ruxd
