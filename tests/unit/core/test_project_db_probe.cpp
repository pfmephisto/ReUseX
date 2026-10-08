// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// ProjectDB::probe(): "is this a ReUseX project, at which schema?" without a
// full open — the Qt client vets a file with it before a read-write open
// would give a foreign sqlite file ReUseX's tables. It must never write and
// never log.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/logging.hpp>

#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace reusex;
namespace fs = std::filesystem;
using reusex::test_support::TempPath;

namespace {

std::string bytes_of(const fs::path &p) {
  std::ifstream f(p, std::ios::binary);
  return {std::istreambuf_iterator<char>(f), {}};
}

} // namespace

TEST_CASE("ProjectDBProbe_RealProject_ReportsLatestSchema", "[core][probe]") {
  TempPath tmp("test_projectdb_probe");
  {
    ProjectDB db(tmp.path);
  }
  const auto r = ProjectDB::probe(tmp.path);
  CHECK(r.is_project);
  CHECK(r.schema_version == ProjectDB::latest_schema_version());
  CHECK(r.error.empty());
}

TEST_CASE("ProjectDBProbe_ForeignSqlite_IsNotAProject_AndUntouched",
          "[core][probe]") {
  TempPath tmp("test_projectdb_probe");
  sqlite3 *db = nullptr;
  REQUIRE(sqlite3_open(tmp.path.string().c_str(), &db) == SQLITE_OK);
  REQUIRE(sqlite3_exec(db,
                       "CREATE TABLE notes(x); INSERT INTO notes VALUES(1);",
                       nullptr, nullptr, nullptr) == SQLITE_OK);
  sqlite3_close(db);
  const std::string before = bytes_of(tmp.path);

  const auto r = ProjectDB::probe(tmp.path);
  CHECK_FALSE(r.is_project);
  CHECK(r.error.find("Required table") != std::string::npos);
  CHECK(bytes_of(tmp.path) == before);
}

TEST_CASE("ProjectDBProbe_TextAndEmptyFiles_AreNotProjects", "[core][probe]") {
  TempPath text("test_projectdb_probe");
  std::ofstream(text.path) << "just notes\n";
  auto r = ProjectDB::probe(text.path);
  CHECK_FALSE(r.is_project);
  CHECK(r.error.find("not a database") != std::string::npos);

  TempPath empty("test_projectdb_probe");
  std::ofstream(empty.path).flush();
  r = ProjectDB::probe(empty.path);
  CHECK_FALSE(r.is_project);
  CHECK(fs::file_size(empty.path) == 0); // never initialised as a database
}

TEST_CASE("ProjectDBProbe_MissingFile_IsNotCreated", "[core][probe]") {
  TempPath tmp("test_projectdb_probe");
  const auto r = ProjectDB::probe(tmp.path);
  CHECK_FALSE(r.is_project);
  CHECK_FALSE(r.error.empty());
  CHECK_FALSE(fs::exists(tmp.path));
}

TEST_CASE("ProjectDBProbe_LogsNothing", "[core][probe]") {
  TempPath tmp("test_projectdb_probe");
  {
    ProjectDB db(tmp.path);
  }
  std::vector<std::string> lines;
  core::set_log_level(core::LogLevel::trace);
  core::set_log_handler(
      [&](core::LogLevel, std::string_view m) { lines.emplace_back(m); });
  (void)ProjectDB::probe(tmp.path);
  core::reset_log_handler();
  core::set_log_level(core::LogLevel::warn);
  CHECK(lines.empty());
}
