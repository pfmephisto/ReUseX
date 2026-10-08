// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// ProjectDB::OpenOptions::hardened and ProjectDB::check_integrity: what a
// server applies to case files that came from an upload or a registration.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <core/ProjectDB.hpp>

#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <atomic>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace reusex;
namespace fs = std::filesystem;
using Kind = ProjectDB::IntegrityReport::Kind;

namespace {

struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_projectdb_hardened") {}
};

void exec(sqlite3 *db, const char *sql) {
  char *err = nullptr;
  const int rc = sqlite3_exec(db, sql, nullptr, nullptr, &err);
  const std::string msg = err ? err : "";
  sqlite3_free(err);
  INFO(msg);
  REQUIRE(rc == SQLITE_OK);
}

void exec_on(const fs::path &file, const char *sql) {
  sqlite3 *db = nullptr;
  REQUIRE(sqlite3_open(file.string().c_str(), &db) == SQLITE_OK);
  exec(db, sql);
  sqlite3_close(db);
}

// A function with a side effect, registered on every connection the process
// opens and deliberately NOT tagged SQLITE_INNOCUOUS — the stand-in for
// anything an attacker would like a file's schema to call.
std::atomic<int> g_side_effects{0};

void side_effect(sqlite3_context *ctx, int, sqlite3_value **) {
  ++g_side_effects;
  sqlite3_result_int(ctx, 9);
}

int register_side_effect(sqlite3 *db, const char **, const void *) {
  return sqlite3_create_function(db, "rux_side_effect", 0, SQLITE_UTF8, nullptr,
                                 side_effect, nullptr, nullptr);
}

struct SideEffectFunction {
  SideEffectFunction() {
    g_side_effects = 0;
    sqlite3_auto_extension(reinterpret_cast<void (*)()>(register_side_effect));
  }
  ~SideEffectFunction() {
    sqlite3_cancel_auto_extension(
        reinterpret_cast<void (*)()>(register_side_effect));
  }
};

// The minimal v9 project of test_project_db_readonly: a read-write open
// migrates it, which INSERTs into schema_version — so a trigger there fires
// during the open itself.
void build_old_schema(const fs::path &path) {
  sqlite3 *db = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &db) == SQLITE_OK);
  exec(db, "CREATE TABLE schema_version (version INTEGER NOT NULL, "
           "applied_at TEXT, description TEXT);");
  exec(db, "INSERT INTO schema_version (version, description) VALUES "
           "(9, 'test fixture');");
  exec(db, "CREATE TABLE projects (id TEXT PRIMARY KEY, name TEXT);");
  exec(db, "CREATE TABLE property_definitions (id TEXT PRIMARY KEY, "
           "name_en TEXT NOT NULL);");
  exec(db, "CREATE TABLE material_passports (id TEXT PRIMARY KEY, "
           "document_guid TEXT UNIQUE, created_at TEXT);");
  exec(db, "CREATE TABLE passport_property_values (id INTEGER PRIMARY KEY);");
  exec(db, "CREATE TABLE passport_log (id INTEGER PRIMARY KEY);");
  sqlite3_close(db);
}

void add_trigger_on_open(const fs::path &file) {
  exec_on(file, "CREATE TRIGGER evil AFTER INSERT ON schema_version "
                "BEGIN SELECT rux_side_effect(); END;");
}

// Replaces pipeline_log, which the server reads for every case's history,
// with a view calling the function: reading the log evaluates the view.
void add_view_on_read(const fs::path &file) {
  exec_on(file, "ALTER TABLE pipeline_log RENAME TO pipeline_log_real;"
                "CREATE VIEW pipeline_log AS SELECT rux_side_effect() AS id, "
                "'x' AS stage, '' AS started_at, '' AS finished_at, '' AS "
                "parameters, 'ok' AS status, '' AS error_msg;");
}

ProjectDB::OpenOptions hardened(bool read_only) {
  ProjectDB::OpenOptions options;
  options.read_only = read_only;
  options.hardened = true;
  return options;
}

} // namespace

TEST_CASE("ProjectDbHardened_FreshProject_RoundTrips",
          "[projectdb][hardened]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path, hardened(false));
    REQUIRE(db.is_open());
    const int id = db.log_pipeline_start("test", "{}");
    db.log_pipeline_end(id, true);
  }
  ProjectDB db(tmp.path, hardened(true));
  REQUIRE(db.schema_version() == ProjectDB::latest_schema_version());
  REQUIRE(ProjectDB::check_integrity(tmp.path).ok());
}

TEST_CASE("ProjectDbHardened_OldSchema_MigratesUnderHardening",
          "[projectdb][hardened]") {
  TempDB tmp;
  build_old_schema(tmp.path);
  ProjectDB db(tmp.path, hardened(false));
  REQUIRE(db.schema_version() == ProjectDB::latest_schema_version());
}

TEST_CASE("ProjectDbHardened_TriggerOnOpen_DoesNotRun",
          "[projectdb][hardened]") {
  SideEffectFunction fn;

  SECTION("control: an ordinary open runs the trigger") {
    TempDB tmp;
    build_old_schema(tmp.path);
    add_trigger_on_open(tmp.path);
    ProjectDB db(tmp.path, /*readOnly=*/false);
    REQUIRE(g_side_effects > 0);
  }

  SECTION("hardened open migrates without running it") {
    TempDB tmp;
    build_old_schema(tmp.path);
    add_trigger_on_open(tmp.path);
    {
      ProjectDB db(tmp.path, hardened(false));
      REQUIRE(db.schema_version() == ProjectDB::latest_schema_version());
    }
    REQUIRE(g_side_effects == 0);

    const auto report = ProjectDB::check_integrity(tmp.path);
    REQUIRE(report.kind == Kind::executable_schema);
    REQUIRE_THAT(report.detail, Catch::Matchers::ContainsSubstring("evil"));
    REQUIRE(g_side_effects == 0);
  }
}

TEST_CASE("ProjectDbHardened_ViewOnRead_DoesNotRun", "[projectdb][hardened]") {
  SideEffectFunction fn;
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
  }
  add_view_on_read(tmp.path);

  SECTION("control: an ordinary connection runs the view") {
    ProjectDB db(tmp.path, /*readOnly=*/true);
    (void)db.pipeline_log();
    REQUIRE(g_side_effects > 0);
  }

  SECTION("a hardened connection refuses it") {
    for (const bool read_only : {true, false}) {
      ProjectDB db(tmp.path, hardened(read_only));
      REQUIRE_THROWS(db.pipeline_log());
    }
    REQUIRE(g_side_effects == 0);

    const auto report = ProjectDB::check_integrity(tmp.path);
    REQUIRE(report.kind == Kind::executable_schema);
    REQUIRE_THAT(report.detail,
                 Catch::Matchers::ContainsSubstring("pipeline_log"));
    REQUIRE(g_side_effects == 0);
  }
}

TEST_CASE("ProjectDbHardened_ProcessDefault_AppliesToPlainConstructor",
          "[projectdb][hardened]") {
  SideEffectFunction fn;
  TempDB tmp;
  build_old_schema(tmp.path);
  add_trigger_on_open(tmp.path);

  REQUIRE_FALSE(ProjectDB::hardened_by_default());
  ProjectDB::set_hardened_by_default(true);
  struct Reset {
    ~Reset() { ProjectDB::set_hardened_by_default(false); }
  } reset;
  {
    ProjectDB db(tmp.path, /*readOnly=*/false);
  }
  REQUIRE(g_side_effects == 0);
}

TEST_CASE("ProjectDbIntegrity_CorruptFile_IsReported",
          "[projectdb][hardened]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
    for (int i = 0; i < 200; ++i)
      db.log_pipeline_start("filler",
                            "{\"pad\": \"" + std::string(512, 'x') + "\"}");
  }
  // Fold the WAL back so the page overwrite below hits real b-tree pages.
  exec_on(tmp.path, "PRAGMA wal_checkpoint(TRUNCATE);"
                    "PRAGMA journal_mode=DELETE;");
  REQUIRE(ProjectDB::check_integrity(tmp.path).ok());

  const auto size = fs::file_size(tmp.path);
  REQUIRE(size > 16 * 4096);
  {
    // Scribble over every page past the first few, keeping each page's
    // offset 0 so the header and the schema page stay readable.
    std::fstream f(tmp.path, std::ios::in | std::ios::out | std::ios::binary);
    const std::string junk(4000, '\xA5');
    for (std::uintmax_t at = 4 * 4096; at + 4096 <= size; at += 4096) {
      f.seekp(static_cast<std::streamoff>(at + 50));
      f.write(junk.data(), static_cast<std::streamsize>(junk.size()));
    }
  }
  const auto report = ProjectDB::check_integrity(tmp.path);
  INFO(report.detail);
  REQUIRE(report.kind == Kind::corrupt);
  REQUIRE_FALSE(report.detail.empty());
}

TEST_CASE("ProjectDbIntegrity_NotADatabase_IsReported",
          "[projectdb][hardened]") {
  TempDB tmp;
  {
    std::ofstream f(tmp.path, std::ios::binary);
    f << std::string(8192, 'z');
  }
  const auto report = ProjectDB::check_integrity(tmp.path);
  REQUIRE_FALSE(report.ok());
}

namespace {

// A deterministic stand-in that a generated column may call: the one way a
// read of a real table (what probe() does) evaluates schema-declared code.
void det_side_effect(sqlite3_context *ctx, int, sqlite3_value **) {
  ++g_side_effects;
  sqlite3_result_int(ctx, 7);
}

int register_det_side_effect(sqlite3 *db, const char **, const void *) {
  return sqlite3_create_function(db, "rux_det_side_effect", 0,
                                 SQLITE_UTF8 | SQLITE_DETERMINISTIC, nullptr,
                                 det_side_effect, nullptr, nullptr);
}

struct DetSideEffectFunction {
  DetSideEffectFunction() {
    g_side_effects = 0;
    sqlite3_auto_extension(
        reinterpret_cast<void (*)()>(register_det_side_effect));
  }
  ~DetSideEffectFunction() {
    sqlite3_cancel_auto_extension(
        reinterpret_cast<void (*)()>(register_det_side_effect));
  }
};

} // namespace

TEST_CASE("ProjectDbHardened_ProcessDefault_AppliesToProbe",
          "[projectdb][hardened][probe]") {
  DetSideEffectFunction fn;
  TempDB tmp;
  build_old_schema(tmp.path);
  // schema_version's version is computed by the file's own schema.
  exec_on(tmp.path, "DROP TABLE schema_version;"
                    "CREATE TABLE schema_version (v INTEGER, version INTEGER "
                    "GENERATED ALWAYS AS (rux_det_side_effect() + v) VIRTUAL);"
                    "INSERT INTO schema_version (v) VALUES (2);");
  g_side_effects = 0;

  SECTION("control: the plain probe `rux` uses evaluates it") {
    REQUIRE_FALSE(ProjectDB::hardened_by_default());
    const auto r = ProjectDB::probe(tmp.path);
    CHECK(r.is_project);
    CHECK(r.schema_version == 9);
    CHECK(g_side_effects > 0);
  }

  SECTION("a hardened process's probe does not") {
    ProjectDB::set_hardened_by_default(true);
    struct Reset {
      ~Reset() { ProjectDB::set_hardened_by_default(false); }
    } reset;
    const auto r = ProjectDB::probe(tmp.path);
    // sqlite refuses the untrusted schema outright, so the file is not
    // vetted as a project, and the reason is reported.
    INFO(r.error);
    CHECK_FALSE(r.is_project);
    CHECK_THAT(r.error, Catch::Matchers::ContainsSubstring("unsafe use"));
    CHECK(g_side_effects == 0);
  }
}

TEST_CASE("ProjectDbHardened_ProcessDefault_RealProjectStillProbes",
          "[projectdb][hardened][probe]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
  }
  ProjectDB::set_hardened_by_default(true);
  struct Reset {
    ~Reset() { ProjectDB::set_hardened_by_default(false); }
  } reset;
  const auto r = ProjectDB::probe(tmp.path);
  CHECK(r.is_project);
  CHECK(r.error.empty());
  CHECK(r.schema_version == ProjectDB::latest_schema_version());
}
