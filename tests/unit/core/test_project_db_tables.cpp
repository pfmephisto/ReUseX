// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Raw, read-only table browsing in ProjectDB (the Qt client's table viewer):
// list_tables / table_columns / table_rows. Pinned: every user table is
// listed with its row count and no sqlite_* internals; pages are stable and
// contiguous; blob cells carry their size and head bytes but never the
// payload; long text is truncated with its full size; a bad table name is
// rejected instead of being spliced into SQL.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>

#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

using reusex::ProjectDB;
using reusex::test_support::TempPath;
using Cell = ProjectDB::TableCell;

namespace {

void exec(const std::filesystem::path &path, const std::string &sql) {
  sqlite3 *db = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &db) == SQLITE_OK);
  char *err = nullptr;
  const int rc = sqlite3_exec(db, sql.c_str(), nullptr, nullptr, &err);
  const std::string msg = err ? err : "";
  sqlite3_free(err);
  sqlite3_close(db);
  INFO(msg);
  REQUIRE(rc == SQLITE_OK);
}

/// A fresh project plus one extra table of every storage class.
void make_project(const std::filesystem::path &path) {
  {
    ProjectDB db(path);
  }
  std::string sql = "CREATE TABLE zz_demo (id INTEGER PRIMARY KEY, n REAL, "
                    "t TEXT, b BLOB, note TEXT);";
  for (int i = 1; i <= 25; ++i)
    sql += "INSERT INTO zz_demo (id, n, t, b) VALUES (" + std::to_string(i) +
           ", " + std::to_string(i) + ".5, 'row " + std::to_string(i) +
           "', zeroblob(" + std::to_string(1000 * i) + "));";
  sql +=
      "UPDATE zz_demo SET t = '" + std::string(1000, 'x') + "' WHERE id = 2;";
  sql += "CREATE TABLE \"we\"\"ird\" (k TEXT PRIMARY KEY, v INTEGER) "
         "WITHOUT ROWID; INSERT INTO \"we\"\"ird\" VALUES ('b', 2), ('a', 1);";
  exec(path, sql);

  // A PNG signature followed by a large body: only the head may come back.
  sqlite3 *db = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &db) == SQLITE_OK);
  std::string png(500008, '\0');
  png.replace(0, 8, std::string("\x89PNG\r\n\x1a\n", 8));
  sqlite3_stmt *s = nullptr;
  REQUIRE(sqlite3_prepare_v2(db, "UPDATE zz_demo SET b = ? WHERE id = 1;", -1,
                             &s, nullptr) == SQLITE_OK);
  sqlite3_bind_blob(s, 1, png.data(), static_cast<int>(png.size()),
                    SQLITE_TRANSIENT);
  CHECK(sqlite3_step(s) == SQLITE_DONE);
  sqlite3_finalize(s);
  sqlite3_close(db);
}

} // namespace

TEST_CASE("ProjectDB_ListTables_ListsUserTablesWithCountsSorted",
          "[core][project_db][tables]") {
  TempPath tmp("project_db_tables");
  make_project(tmp.path);
  ProjectDB db(tmp.path, /*readOnly=*/true);

  const auto tables = db.list_tables();
  REQUIRE_FALSE(tables.empty());
  CHECK(std::is_sorted(
      tables.begin(), tables.end(),
      [](const auto &a, const auto &b) { return a.name < b.name; }));
  for (const auto &t : tables)
    CHECK(t.name.rfind("sqlite_", 0) != 0);

  auto find = [&](const std::string &name) {
    return std::find_if(tables.begin(), tables.end(),
                        [&](const auto &t) { return t.name == name; });
  };
  REQUIRE(find("zz_demo") != tables.end());
  CHECK(find("zz_demo")->row_count == 25);
  REQUIRE(find("sensor_frames") != tables.end());
  CHECK(find("sensor_frames")->row_count == 0);
  REQUIRE(find("we\"ird") != tables.end());
  CHECK(find("we\"ird")->row_count == 2);
}

TEST_CASE("ProjectDB_TableColumns_ReportsDeclaredTypesAndKeys",
          "[core][project_db][tables]") {
  TempPath tmp("project_db_tables");
  make_project(tmp.path);
  ProjectDB db(tmp.path, true);

  const auto cols = db.table_columns("zz_demo");
  REQUIRE(cols.size() == 5);
  CHECK(cols[0].name == "id");
  CHECK(cols[0].primary_key);
  CHECK(cols[3].name == "b");
  CHECK(cols[3].declared_type == "BLOB");
  CHECK_FALSE(cols[3].primary_key);
}

TEST_CASE("ProjectDB_TableRows_PagesAreContiguousAndTyped",
          "[core][project_db][tables]") {
  TempPath tmp("project_db_tables");
  make_project(tmp.path);
  ProjectDB db(tmp.path, true);

  std::vector<std::int64_t> ids;
  for (std::int64_t off = 0; off < 30; off += 10)
    for (const auto &row : db.table_rows("zz_demo", off, 10))
      ids.push_back(row[0].integer);
  REQUIRE(ids.size() == 25);
  for (std::size_t i = 0; i < ids.size(); ++i)
    CHECK(ids[i] == static_cast<std::int64_t>(i + 1));

  const auto first = db.table_rows("zz_demo", 0, 2);
  REQUIRE(first.size() == 2);
  const auto &r = first[0];
  CHECK(r[0].kind == Cell::Kind::integer);
  CHECK(r[1].kind == Cell::Kind::real);
  CHECK(r[1].real == 1.5);
  CHECK(r[2].kind == Cell::Kind::text);
  CHECK(r[2].text == "row 1");
  CHECK_FALSE(r[2].truncated);
  CHECK(r[4].kind == Cell::Kind::null);

  // Blob: size and head bytes, never the payload.
  CHECK(r[3].kind == Cell::Kind::blob);
  CHECK(r[3].size == 500008);
  CHECK(r[3].text.size() == 16);
  CHECK(r[3].text.substr(0, 8) == std::string("\x89PNG\r\n\x1a\n", 8));

  // Long text is cut at the limit with its full size reported.
  const auto &t = first[1][2];
  CHECK(t.truncated);
  CHECK(t.size == 1000);
  CHECK(t.text.size() == 256);
  CHECK(db.table_rows("zz_demo", 1, 1, 2000)[0][2].text.size() == 1000);

  CHECK(db.table_rows("zz_demo", 25, 10).empty());
  CHECK(db.table_rows("zz_demo", 0, 0).empty());
}

TEST_CASE("ProjectDB_TableRows_WithoutRowidOrdersByPrimaryKey",
          "[core][project_db][tables]") {
  TempPath tmp("project_db_tables");
  make_project(tmp.path);
  ProjectDB db(tmp.path, true);
  const auto rows = db.table_rows("we\"ird", 0, 10);
  REQUIRE(rows.size() == 2);
  CHECK(rows[0][0].text == "a");
  CHECK(rows[1][0].text == "b");
}

TEST_CASE("ProjectDB_TableRows_RejectsUnknownTablesAndBadRanges",
          "[core][project_db][tables]") {
  TempPath tmp("project_db_tables");
  make_project(tmp.path);
  ProjectDB db(tmp.path, true);
  CHECK_THROWS_AS(db.table_columns("nope"), std::invalid_argument);
  CHECK_THROWS_AS(db.table_rows("zz_demo; DROP TABLE zz_demo", 0, 1),
                  std::invalid_argument);
  CHECK_THROWS_AS(db.table_rows("sqlite_master", 0, 1), std::invalid_argument);
  CHECK_THROWS_AS(db.table_rows("zz_demo", -1, 1), std::invalid_argument);
  CHECK(db.list_tables().size() > 2); // nothing was dropped
}
