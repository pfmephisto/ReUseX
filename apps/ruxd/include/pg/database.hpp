// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// ruxd's Postgres access for server mode (spec 2026-10-08, phase S3): a
// fixed-size pool of libpqxx connections (connection_pool.hpp) shared by the
// stores in pg/stores.hpp. Lives in ruxd_pg_lib, which links ruxd_api_lib and
// libpqxx only, so its tests run in the light test binary.

#include "connection_pool.hpp"

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace pqxx {
class connection;
}

namespace ruxd::pg {

class Database {
    public:
  /// @p dsn: a libpq connection string or URI. Connections open lazily.
  Database(std::string dsn, std::size_t pool_size,
           std::chrono::milliseconds acquire_timeout =
               std::chrono::milliseconds(5000));
  ~Database();

  Database(const Database &) = delete;
  Database &operator=(const Database &) = delete;

  using Lease = ConnectionPool<pqxx::connection>::Lease;

  /// A pooled connection. @throws ConnectionPoolTimeout when every
  /// connection stays busy past the acquire timeout.
  Lease acquire();

  /// Run @p fn with a pooled connection. A throwing @p fn marks the
  /// connection broken (it is discarded, not recycled).
  template <class F> decltype(auto) with(F &&fn) {
    Lease lease = acquire();
    try {
      return std::forward<F>(fn)(*lease);
    } catch (...) {
      lease.mark_broken();
      throw;
    }
  }

  /// Apply the embedded migrations (pg/migrations.hpp). @return the
  /// versions applied.
  std::vector<int> migrate();

  /// Whether a pooled connection can run `SELECT 1` (readiness).
  bool ping();

  const std::string &dsn() const noexcept { return dsn_; }

    private:
  std::string dsn_;
  std::unique_ptr<ConnectionPool<pqxx::connection>> pool_;
};

} // namespace ruxd::pg
