// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>

#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace reusex;
namespace fs = std::filesystem;

namespace {

struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_projectdb_readonly") {}
};

void exec(sqlite3 *db, const char *sql) {
  char *err = nullptr;
  int rc = sqlite3_exec(db, sql, nullptr, nullptr, &err);
  std::string msg = err ? err : "";
  sqlite3_free(err);
  REQUIRE(rc == SQLITE_OK);
}

// Build a minimal old-schema (v9) fixture directly with sqlite3,
// without going through ProjectDB (which would auto-migrate on open).
void buildOldSchemaFixture(const fs::path &path) {
  sqlite3 *db = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &db) == SQLITE_OK);

  exec(db, "CREATE TABLE schema_version (version INTEGER NOT NULL, "
           "applied_at TEXT, description TEXT);");
  exec(db, "INSERT INTO schema_version (version, description) VALUES "
           "(9, 'test fixture');");

  // Minimal tables required by validateSchema() so the DB is otherwise valid.
  exec(db, "CREATE TABLE projects (id TEXT PRIMARY KEY, name TEXT);");
  exec(db, "CREATE TABLE property_definitions (id TEXT PRIMARY KEY, "
           "name_en TEXT NOT NULL);");
  exec(db, "CREATE TABLE material_passports (id TEXT PRIMARY KEY, "
           "document_guid TEXT UNIQUE, created_at TEXT);");
  exec(db, "CREATE TABLE passport_property_values (id INTEGER PRIMARY KEY);");
  exec(db, "CREATE TABLE passport_log (id INTEGER PRIMARY KEY);");

  sqlite3_close(db);
}

// Read all bytes of a file for byte-for-byte comparison.
std::vector<std::uint8_t> read_file_bytes(const fs::path &p) {
  std::ifstream f(p, std::ios::binary);
  return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>{}};
}

} // namespace

TEST_CASE("ProjectDbReadOnly_OldSchema_FileUnchanged",
          "[projectdb][readonly]") {
  TempDB tmp;
  buildOldSchemaFixture(tmp.path);

  // Snapshot file bytes before open.
  auto before = read_file_bytes(tmp.path);
  REQUIRE(!before.empty());

  // Open read-only — schema migration must NOT run.
  {
    ProjectDB db(tmp.path, /*readOnly=*/true);
    REQUIRE(db.is_open());
    REQUIRE(db.schema_version() == 9);
  }

  // File bytes must be identical — no WAL flush, no schema bump.
  auto after = read_file_bytes(tmp.path);
  REQUIRE(before == after);
}

TEST_CASE("ProjectDbReadWrite_OldSchema_Migrates", "[projectdb][readonly]") {
  TempDB tmp;
  buildOldSchemaFixture(tmp.path);

  const int before_ver = 9;
  {
    ProjectDB db(tmp.path, /*readOnly=*/false);
    REQUIRE(db.is_open());
    REQUIRE(db.schema_version() > before_ver);
  }
}

TEST_CASE("ProjectDbReadOnly_IsReadOnly_ReturnsTrue", "[projectdb][readonly]") {
  TempDB tmp;
  // Create a fresh up-to-date DB first so read-only open has a valid schema.
  {
    ProjectDB rw(tmp.path);
    REQUIRE(rw.is_open());
  }

  ProjectDB db(tmp.path, /*readOnly=*/true);
  REQUIRE(db.is_open());
  REQUIRE(db.is_read_only());
}

TEST_CASE("ProjectDbReadWrite_IsReadOnly_ReturnsFalse",
          "[projectdb][readonly]") {
  TempDB tmp;
  ProjectDB db(tmp.path, /*readOnly=*/false);
  REQUIRE(db.is_open());
  REQUIRE_FALSE(db.is_read_only());
}

TEST_CASE("ProjectDbReadOnly_WriteApi_Throws", "[projectdb][readonly]") {
  TempDB tmp;
  // Create a fresh rw DB first, then re-open read-only.
  {
    ProjectDB rw(tmp.path);
    REQUIRE(rw.is_open());
  }

  ProjectDB db(tmp.path, /*readOnly=*/true);
  REQUIRE(db.is_read_only());

  // Any write call must throw std::runtime_error.
  REQUIRE_THROWS_AS(db.save_point_cloud("test", *std::make_shared<Cloud>()),
                    std::runtime_error);
}
