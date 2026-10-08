// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "pg/database.hpp"

#include "pg/migrations.hpp"

#include <pqxx/pqxx>

namespace ruxd::pg {

Database::Database(std::string dsn, std::size_t pool_size,
                   std::chrono::milliseconds acquire_timeout)
    : dsn_(std::move(dsn)) {
  pool_ = std::make_unique<ConnectionPool<pqxx::connection>>(
      [dsn = dsn_] { return std::make_unique<pqxx::connection>(dsn); },
      ConnectionPoolOptions{pool_size, acquire_timeout, "postgres"},
      [](pqxx::connection &conn) { return conn.is_open(); });
}

Database::~Database() = default;

Database::Lease Database::acquire() { return pool_->acquire(); }

std::vector<int> Database::migrate() {
  // Its own connection: the advisory lock is per session, and a pooled
  // connection would carry it back into the pool on an early exit.
  pqxx::connection conn(dsn_);
  return pg::migrate(conn);
}

bool Database::ping() const {
  try {
    pqxx::connection conn(dsn_);
    pqxx::nontransaction tx(conn);
    tx.exec("SELECT 1");
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

} // namespace ruxd::pg
