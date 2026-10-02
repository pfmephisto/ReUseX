// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Schema v24 -> v25 (resources/templates spec §4.2, §5.3, §5.4): a fresh
// project is created at the latest version and rolled back to v24 with raw
// SQL, then reopened read-write so migrateToV25 runs on real v24 data.

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
using json = nlohmann::json;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_migration_v25") {}
};

void exec_raw(const fs::path &path, const std::string &sql) {
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &raw) == SQLITE_OK);
  char *err = nullptr;
  const int rc = sqlite3_exec(raw, sql.c_str(), nullptr, nullptr, &err);
  INFO((err ? err : ""));
  sqlite3_free(err);
  sqlite3_close(raw);
  REQUIRE(rc == SQLITE_OK);
}

/// Undo everything v25 adds, so the next read-write open migrates again.
void roll_back_to_v24(const fs::path &path) {
  exec_raw(path, R"sql(
    DROP TABLE templates;
    CREATE TABLE export_templates (
      id INTEGER PRIMARY KEY AUTOINCREMENT, name TEXT NOT NULL,
      config TEXT NOT NULL DEFAULT '{}',
      created_at TEXT NOT NULL DEFAULT (datetime('now')),
      updated_at TEXT NOT NULL DEFAULT (datetime('now')));
    DELETE FROM schema_version WHERE version >= 25;
  )sql");
}

bool table_exists(const fs::path &path, const char *name) {
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &raw) == SQLITE_OK);
  sqlite3_stmt *s = nullptr;
  sqlite3_prepare_v2(raw,
                     "SELECT 1 FROM sqlite_master WHERE type='table' AND "
                     "name=?;",
                     -1, &s, nullptr);
  sqlite3_bind_text(s, 1, name, -1, SQLITE_STATIC);
  const bool found = sqlite3_step(s) == SQLITE_ROW;
  sqlite3_finalize(s);
  sqlite3_close(raw);
  return found;
}

std::vector<std::string> names(const ProjectDB &db) {
  std::vector<std::string> out;
  for (const auto &t : db.resource_templates())
    out.push_back(t.name);
  return out;
}
} // namespace

TEST_CASE("MigrationV25_FreshProject_HasBothSeeds", "[ProjectDB][migration]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  CHECK(db.schema_version() == 25);
  const auto list = db.resource_templates();
  REQUIRE(list.size() == 2);
  CHECK(list[0].name == "Materialepas (fuld)");
  CHECK(list[0].seed == std::optional<std::string>("materialepas"));
  CHECK(core::read_members(list[0].members_json, "") ==
        core::seed_templates()[0].members);
  CHECK(list[1].name == "Hurtig genbrugsscreening");
  CHECK(list[1].seed == std::optional<std::string>("screening"));
  CHECK(list[1].csv_json == "{}");
  CHECK_FALSE(table_exists(tmp.path, "export_templates"));
}

TEST_CASE("MigrationV25_MovesExportTemplates_ClashUnmatchedAndBroken",
          "[ProjectDB][migration]") {
  TempDB tmp;
  std::string bredde;
  {
    ProjectDB db(tmp.path);
    bredde = db.add_property_definition("Bredde", "number", {}, 0);
  }
  roll_back_to_v24(tmp.path);
  exec_raw(tmp.path, R"sql(
    INSERT INTO export_templates (name, config) VALUES
      ('Materialepas (fuld)', '{"columns":["Bredde","kind"],"delimiter":","}'),
      ('Mine', '{"columns":["Note"]}'),
      ('Mine', '{}'),
      ('Ødelagt', 'not json');
  )sql");
  ProjectDB db(tmp.path);
  CHECK(db.schema_version() == 25);
  CHECK_FALSE(table_exists(tmp.path, "export_templates"));
  CHECK(names(db) == std::vector<std::string>{
                         "Materialepas (fuld)", "Hurtig genbrugsscreening",
                         "Materialepas (fuld) (eksport)", "Mine",
                         "Mine (eksport)", "Ødelagt"});
  const auto list = db.resource_templates();
  CHECK(core::read_members(list[2].members_json, "") ==
        std::vector<core::TemplateMember>{
            {core::MemberKind::key, "col:" + bredde},
            {core::MemberKind::key, "legacy:kind"}});
  CHECK(json::parse(list[2].csv_json) == json::parse(R"({"delimiter":","})"));
  CHECK_FALSE(list[2].seed.has_value());
  CHECK(core::read_members(list[3].members_json, "") ==
        std::vector<core::TemplateMember>{{core::MemberKind::key, "sys:note"}});
  CHECK(list[5].members_json == "[]");
  CHECK(list[5].csv_json == "{}");
}

TEST_CASE("MigrationV25_SecondOpenIsANoOp", "[ProjectDB][migration]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
  }
  roll_back_to_v24(tmp.path);
  exec_raw(tmp.path,
           "INSERT INTO export_templates (name, config) VALUES ('A', '{}');");
  {
    ProjectDB db(tmp.path);
  }
  ProjectDB db(tmp.path);
  CHECK(names(db) == std::vector<std::string>{"Materialepas (fuld)",
                                              "Hurtig genbrugsscreening", "A"});
}

TEST_CASE("TemplatesStore_Crud_NameIsUnique", "[ProjectDB][templates]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  ProjectDB::ResourceTemplateRecord rec;
  rec.name = "Ny";
  rec.members_json = R"([{"key":"sys:name"}])";
  const auto added = db.add_resource_template(rec);
  CHECK(added.id > 0);
  CHECK(added.members_json == rec.members_json);
  CHECK_FALSE(added.created_at.empty());
  CHECK_THROWS_AS(db.add_resource_template(rec), core::NameConflictError);
  ProjectDB::ResourceTemplatePatch p;
  p.name = "Hurtig genbrugsscreening";
  CHECK_THROWS_AS(db.update_resource_template(added.id, p),
                  core::NameConflictError);
  p.name = "Omdøbt";
  p.csv_json = R"({"delimiter":","})";
  const auto updated = db.update_resource_template(added.id, p);
  CHECK(updated.name == "Omdøbt");
  CHECK(updated.csv_json == R"({"delimiter":","})");
  CHECK(updated.members_json == rec.members_json);
  CHECK_THROWS_AS(db.update_resource_template(9999, p), std::out_of_range);
  CHECK(db.delete_resource_template(added.id));
  CHECK_FALSE(db.delete_resource_template(added.id));
  CHECK_FALSE(db.resource_template(added.id).has_value());
}
