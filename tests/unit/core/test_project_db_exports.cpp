// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Schema v21 migration and export_templates CRUD round-trip tests (#459).
// Covers: table creation on a fresh DB, add/list/fetch/update/delete, and that
// a DB opened at v20 migrates cleanly to v21.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>

#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <filesystem>
#include <string>

using reusex::ProjectDB;
namespace fs = std::filesystem;

namespace {

struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_projectdb_exports") {}
};

// Minimal v20 fixture so migrateToV21 runs in isolation.
void build_v20_fixture(const fs::path &path) {
  sqlite3 *db = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &db) == SQLITE_OK);

  auto exec = [&](const char *sql) {
    char *err = nullptr;
    int rc = sqlite3_exec(db, sql, nullptr, nullptr, &err);
    std::string msg = err ? err : "";
    sqlite3_free(err);
    INFO("SQL failed: " << msg);
    REQUIRE(rc == SQLITE_OK);
  };

  exec("PRAGMA journal_mode=WAL;");
  exec("CREATE TABLE schema_version "
       "(version INTEGER NOT NULL, applied_at TEXT, description TEXT);");
  exec("INSERT INTO schema_version (version, description) VALUES "
       "(20, 'test fixture at v20');");

  // Tables required by migration guards in v18/v19.
  exec("CREATE TABLE material_passports ("
       "  id TEXT PRIMARY KEY, project_id TEXT, document_guid TEXT UNIQUE,"
       "  created_at TEXT, revised_at TEXT, version_number TEXT,"
       "  version_date TEXT);");
  exec("CREATE TABLE material_property_definitions ("
       "  id TEXT PRIMARY KEY, name TEXT NOT NULL,"
       "  type TEXT NOT NULL DEFAULT 'text', options TEXT,"
       "  sort_order INTEGER NOT NULL DEFAULT 0,"
       "  created_at TEXT NOT NULL DEFAULT (datetime('now')),"
       "  width INTEGER NOT NULL DEFAULT 200);");
  exec("CREATE TABLE material_thumbnails ("
       "  material_guid TEXT PRIMARY KEY "
       "    REFERENCES material_passports(document_guid) ON DELETE CASCADE,"
       "  blob BLOB NOT NULL, mime_type TEXT NOT NULL DEFAULT 'image/jpeg',"
       "  created_at TEXT NOT NULL DEFAULT (datetime('now')));");
  exec("CREATE TABLE report_pdfs ("
       "  id         INTEGER PRIMARY KEY AUTOINCREMENT,"
       "  label      TEXT    NOT NULL DEFAULT '',"
       "  created_at TEXT    NOT NULL DEFAULT (datetime('now')),"
       "  pdf_blob   BLOB    NOT NULL);");

  sqlite3_close(db);
}

} // namespace

TEST_CASE("ExportTemplates_FreshDB_TableExists",
          "[ProjectDB][exports][schema]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  REQUIRE(db.list_export_templates().empty());
  REQUIRE(db.schema_version() == ProjectDB::latest_schema_version());
}

TEST_CASE("ExportTemplates_MigratesFromV20_TableCreated",
          "[ProjectDB][exports][schema]") {
  TempDB tmp;
  build_v20_fixture(tmp.path);

  // Read-write open must migrate v20 → v21.
  ProjectDB db(tmp.path);
  REQUIRE(db.schema_version() == ProjectDB::latest_schema_version());
  REQUIRE(db.list_export_templates().empty());
}

TEST_CASE("ExportTemplates_AddAndList_RoundTrip", "[ProjectDB][exports]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  const std::string cfg = R"({"columns":["kind","id","component_name"]})";
  const auto rec = db.add_export_template("My Selection", cfg);

  REQUIRE(rec.id > 0);
  REQUIRE(rec.name == "My Selection");
  REQUIRE(rec.config_json == cfg);
  REQUIRE(!rec.created_at.empty());
  REQUIRE(!rec.updated_at.empty());

  const auto list = db.list_export_templates();
  REQUIRE(list.size() == 1);
  CHECK(list[0].id == rec.id);
  CHECK(list[0].name == "My Selection");
  CHECK(list[0].config_json == cfg);
}

TEST_CASE("ExportTemplates_Fetch_ReturnsExactRecord", "[ProjectDB][exports]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  db.add_export_template("A", R"({})");
  const auto b = db.add_export_template("B", R"({"columns":["kind"]})");

  const auto got = db.export_template(b.id);
  REQUIRE(got.has_value());
  CHECK(got->name == "B");
  CHECK(got->config_json == R"({"columns":["kind"]})");
}

TEST_CASE("ExportTemplates_Fetch_MissingReturnsNullopt",
          "[ProjectDB][exports]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  CHECK(!db.export_template(9999).has_value());
}

TEST_CASE("ExportTemplates_Update_ChangesNameAndConfig",
          "[ProjectDB][exports]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  const auto orig = db.add_export_template("Old", R"({"columns":[]})");
  const auto upd =
      db.update_export_template(orig.id, "New", R"({"columns":["kind","id"]})");

  CHECK(upd.id == orig.id);
  CHECK(upd.name == "New");
  CHECK(upd.config_json == R"({"columns":["kind","id"]})");

  const auto fetched = db.export_template(orig.id);
  REQUIRE(fetched.has_value());
  CHECK(fetched->name == "New");
}

TEST_CASE("ExportTemplates_Delete_RemovesRecord", "[ProjectDB][exports]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  const auto rec = db.add_export_template("Temp", R"({})");
  REQUIRE(db.list_export_templates().size() == 1);

  CHECK(db.delete_export_template(rec.id));
  CHECK(db.list_export_templates().empty());
  CHECK(!db.export_template(rec.id).has_value());
}

TEST_CASE("ExportTemplates_DeleteMissing_ReturnsFalse",
          "[ProjectDB][exports]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  CHECK(!db.delete_export_template(42));
}

TEST_CASE("ExportTemplates_MultipleTemplates_ListPreservesOrder",
          "[ProjectDB][exports]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  db.add_export_template("First", "{}");
  db.add_export_template("Second", "{}");
  db.add_export_template("Third", "{}");

  const auto list = db.list_export_templates();
  REQUIRE(list.size() == 3);
  CHECK(list[0].name == "First");
  CHECK(list[1].name == "Second");
  CHECK(list[2].name == "Third");
}
