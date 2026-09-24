// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Schema v20 migration and report_pdfs CRUD round-trip tests (#456).
// Covers: table creation on a fresh DB, add/list/fetch, and that a DB opened
// at v19 migrates cleanly to v20.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>

#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

using reusex::ProjectDB;
namespace fs = std::filesystem;

namespace {

struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_projectdb_reports") {}
};

// Build a minimal v19 database so migrateToV20 runs in isolation.
void build_v19_fixture(const fs::path &path) {
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

  // schema_version table with v19 recorded.
  exec("CREATE TABLE schema_version "
       "(version INTEGER NOT NULL, applied_at TEXT, description TEXT);");
  exec("INSERT INTO schema_version (version, description) VALUES "
       "(19, 'test fixture at v19');");

  // material_passports must exist for the FK in material_thumbnails (v18).
  exec("CREATE TABLE material_passports ("
       "  id TEXT PRIMARY KEY, project_id TEXT, document_guid TEXT UNIQUE,"
       "  created_at TEXT, revised_at TEXT, version_number TEXT,"
       "  version_date TEXT);");

  // Tables introduced by v18 that later migrations may check for.
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

  sqlite3_close(db);
}

} // namespace

TEST_CASE("ReportPdfs_FreshDB_TableExists", "[ProjectDB][reports][schema]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  // On a fresh DB, list_report_pdfs() should return an empty list without
  // throwing (the table was created by migrateToV20).
  REQUIRE(db.list_report_pdfs().empty());
}

TEST_CASE("ReportPdfs_AddAndList_RoundTrip", "[ProjectDB][reports]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  const std::vector<std::uint8_t> fake_pdf = {0x25, 0x50, 0x44, 0x46,
                                              0x2D}; // "%PDF-"
  const auto rec = db.add_report_pdf(fake_pdf, "Ressourcekortlægning");

  REQUIRE(rec.id > 0);
  REQUIRE(rec.label == "Ressourcekortlægning");
  REQUIRE(rec.size_bytes == fake_pdf.size());
  REQUIRE(!rec.created_at.empty());

  const auto list = db.list_report_pdfs();
  REQUIRE(list.size() == 1);
  REQUIRE(list.front().id == rec.id);
  REQUIRE(list.front().size_bytes == fake_pdf.size());
}

TEST_CASE("ReportPdfs_FetchById_ReturnsBlobOrNullopt", "[ProjectDB][reports]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  const std::vector<std::uint8_t> data = {1, 2, 3, 4, 5};
  const auto rec = db.add_report_pdf(data, "test");

  SECTION("Existing id returns the correct blob") {
    const auto fetched = db.report_pdf(rec.id);
    REQUIRE(fetched.has_value());
    REQUIRE(*fetched == data);
  }

  SECTION("Non-existent id returns nullopt") {
    const auto missing = db.report_pdf(rec.id + 9999);
    REQUIRE(!missing.has_value());
  }
}

TEST_CASE("ReportPdfs_MultipleVersions_ListNewestFirst",
          "[ProjectDB][reports]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  const std::vector<std::uint8_t> pdf1 = {0x01};
  const std::vector<std::uint8_t> pdf2 = {0x02};
  const std::vector<std::uint8_t> pdf3 = {0x03};

  const auto r1 = db.add_report_pdf(pdf1, "v1");
  const auto r2 = db.add_report_pdf(pdf2, "v2");
  const auto r3 = db.add_report_pdf(pdf3, "v3");

  const auto list = db.list_report_pdfs();
  REQUIRE(list.size() == 3);
  // Newest (highest id) first.
  REQUIRE(list[0].id == r3.id);
  REQUIRE(list[1].id == r2.id);
  REQUIRE(list[2].id == r1.id);
}

TEST_CASE("ReportPdfs_MigrationFromV19_CreatesTable",
          "[ProjectDB][reports][migration]") {
  TempDB tmp;
  build_v19_fixture(tmp.path);

  // Opening read-write triggers runMigrations(), which should apply v20.
  ProjectDB db(tmp.path, /*readOnly=*/false);

  // The table should exist and be usable.
  REQUIRE(db.list_report_pdfs().empty());

  const std::vector<std::uint8_t> pdf = {0x25, 0x50, 0x44, 0x46};
  const auto rec = db.add_report_pdf(pdf, "migrated");
  REQUIRE(rec.id > 0);
  REQUIRE(rec.size_bytes == 4);
}
