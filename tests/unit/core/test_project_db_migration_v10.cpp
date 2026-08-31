// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>

#include <sqlite3.h>

#include <filesystem>
#include <string>

using namespace reusex;
namespace fs = std::filesystem;

namespace {

struct TempPath {
  fs::path path;
  TempPath()
      : path(fs::temp_directory_path() /
             ("test_migration_v10_" +
              std::to_string(reinterpret_cast<uintptr_t>(this)) + ".rux")) {}
  ~TempPath() noexcept {
    std::error_code ec;
    fs::remove(path, ec);
  }
};

void exec(sqlite3 *db, const char *sql) {
  char *err = nullptr;
  int rc = sqlite3_exec(db, sql, nullptr, nullptr, &err);
  std::string msg = err ? err : "";
  sqlite3_free(err);
  REQUIRE(rc == SQLITE_OK);
}

// Build a minimal schema-version-9 database directly with sqlite3, containing:
//  - one instance-label cloud ("instances") with label_definitions
//  - two passports (one referenced, one that will be orphaned)
//  - two instance_materials links: one valid, one dangling (missing passport)
void buildV9Fixture(const fs::path &path) {
  sqlite3 *db = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &db) == SQLITE_OK);

  exec(db, "CREATE TABLE schema_version (version INTEGER NOT NULL, "
           "applied_at TEXT, description TEXT);");
  // Mark as v9 so ProjectDB runs only migrateToV10.
  exec(db, "INSERT INTO schema_version (version, description) VALUES "
           "(9, 'test fixture');");

  exec(db, "CREATE TABLE point_clouds (id INTEGER PRIMARY KEY AUTOINCREMENT, "
           "name TEXT UNIQUE, point_type TEXT, point_count INTEGER, "
           "point_step INTEGER, width INTEGER, height INTEGER, stage TEXT, "
           "parameters TEXT);");
  exec(db, "INSERT INTO point_clouds (id, name, point_type, point_count, "
           "point_step, width, height, stage, parameters) VALUES "
           "(1, 'instances', 'Label', 4, 4, 4, 1, 'segment_instances', '');");

  exec(db, "CREATE TABLE label_definitions (cloud_id INTEGER, label_id "
           "INTEGER, name TEXT, PRIMARY KEY (cloud_id, label_id));");
  exec(db, "INSERT INTO label_definitions VALUES (1, 1, 'SM3-1 (2p)');");
  exec(db, "INSERT INTO label_definitions VALUES (1, 2, 'SM3-2 (2p)');");

  exec(db, "CREATE TABLE material_passports (id TEXT PRIMARY KEY, project_id "
           "TEXT, document_guid TEXT UNIQUE, created_at TEXT, revised_at TEXT, "
           "version_number TEXT, version_date TEXT);");
  exec(db, "INSERT INTO material_passports (id, document_guid, created_at) "
           "VALUES ('p1', 'mat-valid', '2025-01-01');");

  // v9 instance_materials: no FKs.
  exec(db,
       "CREATE TABLE instance_materials (cloud_id INTEGER, instance_id "
       "INTEGER, material_guid TEXT, PRIMARY KEY (cloud_id, instance_id));");
  // Valid link (instance 1 -> existing passport).
  exec(db, "INSERT INTO instance_materials VALUES (1, 1, 'mat-valid');");
  // Orphan link (instance 2 -> passport that does not exist).
  exec(db, "INSERT INTO instance_materials VALUES (2, 2, 'mat-missing');");

  sqlite3_close(db);
}

} // namespace

TEST_CASE("Fresh database is created at schema version 10",
          "[projectdb][migration]") {
  TempPath tmp;
  ProjectDB db(tmp.path);
  REQUIRE(db.schema_version() == 10);
}

TEST_CASE("Migration v9 -> v10 drops orphan links and preserves valid ones",
          "[projectdb][migration]") {
  TempPath tmp;
  buildV9Fixture(tmp.path);

  // Opening triggers migrateToV10.
  ProjectDB db(tmp.path);
  REQUIRE(db.schema_version() == 10);

  // Instances were backfilled from label_definitions.
  auto instances = db.instances("instances");
  REQUIRE(instances.size() == 2);
  // Semantic class parsed from "SM3-..." naming.
  for (const auto &r : instances)
    REQUIRE(r.semantic_class == 3);

  // The valid link survived; the orphan (missing passport) was dropped.
  auto links = db.instance_materials("instances");
  REQUIRE(links.size() == 1);
  REQUIRE(links.count(1) == 1);
  REQUIRE(links.at(1) == "mat-valid");
  REQUIRE(links.count(2) == 0);
}
