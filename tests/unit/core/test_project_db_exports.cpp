// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The /export-templates view over the v25 templates table (resources/templates
// spec §5.4): CRUD round trips keep working for ExportPage and ruxd.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/resource_templates.hpp>

#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>
#include <sqlite3.h>

#include <filesystem>
#include <string>
#include <vector>

using reusex::ProjectDB;
namespace core = reusex::core;
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

TEST_CASE("ExportTemplates_FreshDB_ListsTheSeeds", "[ProjectDB][exports]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  REQUIRE(db.schema_version() == ProjectDB::latest_schema_version());
  const auto list = db.list_export_templates();
  REQUIRE(list.size() == 2);
  CHECK(list[0].name == "Materialepas (fuld)");
  // A category-only template has no legacy columns: ExportPage reads [] as
  // "all columns".
  CHECK(nlohmann::json::parse(list[0].config_json).at("columns") ==
        nlohmann::json::array());
}

TEST_CASE("ExportTemplates_MigratesFromV20", "[ProjectDB][exports]") {
  TempDB tmp;
  build_v20_fixture(tmp.path);
  ProjectDB db(tmp.path, /*readOnly=*/false);
  REQUIRE(db.schema_version() == ProjectDB::latest_schema_version());
  CHECK(db.list_export_templates().size() == 2);
}

TEST_CASE("ExportTemplates_AddFetch_RoundTripsColumnsThroughMembers",
          "[ProjectDB][exports]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto col = db.add_property_definition("Bredde", "number", {}, 0);
  const auto rec = db.add_export_template(
      "Valg", R"({"columns":["kind","id","Bredde"],"delimiter":","})");
  CHECK(rec.name == "Valg");
  const auto cfg = nlohmann::json::parse(rec.config_json);
  CHECK(cfg.at("columns") ==
        nlohmann::json::parse(R"(["kind","id","Bredde"])"));
  CHECK(cfg.at("delimiter") == ",");
  const auto stored = db.resource_template(rec.id);
  REQUIRE(stored.has_value());
  CHECK(
      core::read_members(stored->members_json, "") ==
      std::vector<core::TemplateMember>{{core::MemberKind::key, "legacy:kind"},
                                        {core::MemberKind::key, "legacy:id"},
                                        {core::MemberKind::key, "col:" + col}});
  const auto fetched = db.export_template(rec.id);
  REQUIRE(fetched.has_value());
  CHECK(fetched->config_json == rec.config_json);
  CHECK_FALSE(db.export_template(9999).has_value());
  CHECK_THROWS_AS(db.add_export_template("Valg", "{}"),
                  core::NameConflictError);
}

TEST_CASE("ExportTemplates_Update_ColumnsAndName_KeepsOtherOptions",
          "[ProjectDB][exports]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto orig =
      db.add_export_template("Old", R"({"columns":["kind"],"delimiter":","})");
  const auto updated =
      db.update_export_template(orig.id, "New", R"({"columns":["kind","id"]})");
  CHECK(updated.name == "New");
  const auto cfg = nlohmann::json::parse(updated.config_json);
  CHECK(cfg.at("columns") == nlohmann::json::parse(R"(["kind","id"])"));
  // A body carrying only columns (all ExportPage sends) keeps csv options.
  CHECK(cfg.at("delimiter") == ",");
  CHECK_THROWS_AS(db.update_export_template(9999, "x", "{}"),
                  std::runtime_error);
}

TEST_CASE("ExportTemplates_Delete_AndOrder", "[ProjectDB][exports]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.add_export_template("First", "{}");
  const auto second = db.add_export_template("Second", "{}");
  CHECK(db.delete_export_template(second.id));
  CHECK_FALSE(db.delete_export_template(second.id));
  const auto list = db.list_export_templates();
  REQUIRE(list.size() == 3);
  CHECK(list[2].name == "First");
}

TEST_CASE("ExportTemplates_UpdateSeed_KeepsCategoryMembers",
          "[ProjectDB][exports]") {
  // R-P4: the legacy view owns only legacy:/col: members. Updating a seed's
  // columns through it must not drop its category members.
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto seed = db.resource_templates().at(0);
  const auto before = core::read_members(seed.members_json, seed.name);
  REQUIRE_FALSE(before.empty());
  const auto col = db.add_property_definition("Bredde", "number", {}, 0);
  const auto updated = db.update_export_template(
      seed.id, seed.name, R"({"columns":["kind","Bredde"]})");
  CHECK(nlohmann::json::parse(updated.config_json).at("columns") ==
        nlohmann::json::parse(R"(["kind","Bredde"])"));
  const auto stored = db.resource_template(seed.id);
  REQUIRE(stored.has_value());
  auto expected = before;
  expected.push_back({core::MemberKind::key, "legacy:kind"});
  expected.push_back({core::MemberKind::key, "col:" + col});
  CHECK(core::read_members(stored->members_json, "") == expected);
  CHECK(stored->seed == seed.seed);

  // A second update replaces the column members in place and still keeps
  // every category member.
  db.update_export_template(seed.id, seed.name, R"({"columns":["id"]})");
  expected = before;
  expected.push_back({core::MemberKind::key, "legacy:id"});
  CHECK(core::read_members(db.resource_template(seed.id)->members_json, "") ==
        expected);
}

TEST_CASE("ExportTemplates_Update_ReplacesColumnsInPlace",
          "[ProjectDB][exports]") {
  // Kept members (category, sys:) keep their relative order; new column
  // members take the first old column member's slot.
  TempDB tmp;
  ProjectDB db(tmp.path);
  ProjectDB::ResourceTemplateRecord rec;
  rec.name = "Blandet";
  rec.members_json = R"([{"key":"sys:name"},{"key":"legacy:a"},)"
                     R"({"category":"Mål"},{"key":"legacy:b"}])";
  const auto added = db.add_resource_template(rec);
  db.update_export_template(added.id, "Blandet", R"({"columns":["c","d"]})");
  CHECK(core::read_members(db.resource_template(added.id)->members_json, "") ==
        std::vector<core::TemplateMember>{{core::MemberKind::key, "sys:name"},
                                          {core::MemberKind::key, "legacy:c"},
                                          {core::MemberKind::key, "legacy:d"},
                                          {core::MemberKind::category, "Mål"}});
}

TEST_CASE("ExportTemplates_Rename_KeepsMembersOfADeletedColumn",
          "[ProjectDB][exports]") {
  // A rename (no columns in the config) touches only the name. A col: member
  // whose user column was deleted is not in the view's columns, so it must
  // survive both a rename and a columns update (kept as a missing member).
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto col = db.add_property_definition("Bredde", "number", {}, 0);
  const auto rec = db.add_export_template(
      "Valg", R"({"columns":["Bredde","kind"],"delimiter":","})");
  const auto before = db.resource_template(rec.id)->members_json;
  db.delete_property_definition(col);

  const auto renamed = db.update_export_template(rec.id, "Omdøbt", "{}");
  CHECK(renamed.name == "Omdøbt");
  CHECK(db.resource_template(rec.id)->members_json == before);
  CHECK(nlohmann::json::parse(renamed.config_json).at("delimiter") == ",");

  db.update_export_template(rec.id, "Omdøbt", R"({"columns":["id"]})");
  CHECK(
      core::read_members(db.resource_template(rec.id)->members_json, "") ==
      std::vector<core::TemplateMember>{{core::MemberKind::key, "col:" + col},
                                        {core::MemberKind::key, "legacy:id"}});
}

TEST_CASE("ExportTemplates_RepeatedColumns_FirstPositionWins",
          "[ProjectDB][exports]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto col = db.add_property_definition("Bredde", "number", {}, 0);
  const auto rec = db.add_export_template(
      "Valg", R"({"columns":["kind","Bredde","kind",7,"Bredde"]})");
  const std::vector<core::TemplateMember> want{
      {core::MemberKind::key, "legacy:kind"},
      {core::MemberKind::key, "col:" + col}};
  CHECK(core::read_members(db.resource_template(rec.id)->members_json, "") ==
        want);
  CHECK(nlohmann::json::parse(rec.config_json).at("columns") ==
        nlohmann::json::parse(R"(["kind","Bredde"])"));

  db.update_export_template(rec.id, "Valg",
                            R"({"columns":["Bredde",null,"kind","Bredde"]})");
  CHECK(core::read_members(db.resource_template(rec.id)->members_json, "") ==
        std::vector<core::TemplateMember>{
            {core::MemberKind::key, "col:" + col},
            {core::MemberKind::key, "legacy:kind"}});
}
