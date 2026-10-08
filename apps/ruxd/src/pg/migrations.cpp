// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "pg/migrations.hpp"

#include <pqxx/pqxx>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <stdexcept>

namespace ruxd::pg {

std::vector<Migration> pending_migrations(const std::vector<Migration> &all,
                                          const std::set<int> &applied) {
  std::set<int> known;
  for (const auto &m : all)
    if (!known.insert(m.version).second)
      throw std::runtime_error("migration version " +
                               std::to_string(m.version) + " appears twice");
  for (const int version : applied)
    if (known.count(version) == 0)
      throw std::runtime_error(
          "the database has migration " + std::to_string(version) +
          ", which this ruxd does not know: it was migrated by a newer "
          "version; upgrade ruxd");
  std::vector<Migration> out;
  for (const auto &m : all)
    if (applied.count(m.version) == 0)
      out.push_back(m);
  std::sort(out.begin(), out.end(), [](const Migration &a, const Migration &b) {
    return a.version < b.version;
  });
  return out;
}

std::set<int> applied_migrations(pqxx::connection &conn) {
  pqxx::nontransaction tx(conn);
  const auto exists =
      tx.exec("SELECT to_regclass('public.schema_migrations') IS NOT NULL")
          .one_field()
          .as<bool>();
  std::set<int> out;
  if (!exists)
    return out;
  for (const auto &row : tx.exec("SELECT version FROM schema_migrations"))
    out.insert(row[0].as<int>());
  return out;
}

namespace {

/// A session-level advisory lock, released however migrate() leaves.
class AdvisoryLock {
    public:
  explicit AdvisoryLock(pqxx::connection &conn) : conn_(conn) {
    pqxx::nontransaction tx(conn_);
    tx.exec("SELECT pg_advisory_lock($1)", pqxx::params{kMigrationLockKey});
  }
  ~AdvisoryLock() {
    try {
      pqxx::nontransaction tx(conn_);
      tx.exec("SELECT pg_advisory_unlock($1)", pqxx::params{kMigrationLockKey});
    } catch (const std::exception &e) {
      // The connection is gone, and with it the lock.
      spdlog::debug("Releasing the migration lock: {}", e.what());
    }
  }
  AdvisoryLock(const AdvisoryLock &) = delete;
  AdvisoryLock &operator=(const AdvisoryLock &) = delete;

    private:
  pqxx::connection &conn_;
};

} // namespace

std::vector<int> migrate(pqxx::connection &conn,
                         const std::vector<Migration> &all) {
  const AdvisoryLock lock(conn);
  {
    pqxx::work tx(conn);
    tx.exec("CREATE TABLE IF NOT EXISTS schema_migrations ("
            "  version    integer     PRIMARY KEY,"
            "  name       text        NOT NULL,"
            "  applied_at timestamptz NOT NULL DEFAULT now())");
    tx.commit();
  }
  // Read under the lock: another process may have just finished.
  const auto todo = pending_migrations(all, applied_migrations(conn));
  std::vector<int> done;
  for (const auto &m : todo) {
    spdlog::info("postgres: applying migration {:03d}_{}", m.version, m.name);
    pqxx::work tx(conn);
    tx.exec(m.sql);
    tx.exec("INSERT INTO schema_migrations (version, name) VALUES ($1, $2)",
            pqxx::params{m.version, m.name});
    tx.commit();
    done.push_back(m.version);
  }
  return done;
}

} // namespace ruxd::pg
