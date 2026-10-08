// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// ruxd's Postgres schema migrations (spec 2026-10-08, phase S3).
//
// The SQL lives in apps/ruxd/migrations/NNN_<name>.sql and is embedded into
// the binary at build time (cmake/EmbedMigrations.cmake). migrate() applies
// the ones a database lacks, in version order, each in its own transaction,
// and records each in `schema_migrations`. It holds a session-level advisory
// lock while it runs, so two ruxd processes starting against one database
// (or `ruxd admin` racing the server) never apply a migration twice.

#include <set>
#include <string>
#include <vector>

namespace pqxx {
class connection;
}

namespace ruxd::pg {

struct Migration {
  int version = 0;
  std::string name;
  std::string sql;
};

/// The migrations compiled into this binary, by version.
const std::vector<Migration> &embedded_migrations();

/// The members of @p all whose version is not in @p applied, by version.
/// @throws std::runtime_error when @p applied holds a version this binary
///         does not know (the database is newer than the server) or @p all
///         repeats a version.
std::vector<Migration> pending_migrations(const std::vector<Migration> &all,
                                          const std::set<int> &applied);

/// The advisory lock key migrate() takes ("ruxd" in ASCII).
inline constexpr long long kMigrationLockKey = 0x72757864;

/// Bring @p conn's database up to date with @p all. @return the versions it
/// applied (empty when already current).
/// @throws whatever Postgres reports; a failed migration is rolled back and
///         nothing after it runs.
std::vector<int>
migrate(pqxx::connection &conn,
        const std::vector<Migration> &all = embedded_migrations());

/// The versions recorded in `schema_migrations` (empty when it is missing).
std::set<int> applied_migrations(pqxx::connection &conn);

} // namespace ruxd::pg
