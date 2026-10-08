// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// ProjectDB::probe(): "is this a ReUseX project, at which schema?" without a
// full open — the Qt client vets a file with it before a read-write open
// would give a foreign sqlite file ReUseX's tables. It must never write,
// never create files (no WAL sidecars) and never log; and a WAL-mode project
// in a read-only directory must probe and open read-only.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/logging.hpp>

#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <filesystem>
#include <fstream>
#include <set>
#include <string>
#include <vector>

#include <unistd.h>

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

// ---- WAL-mode projects: no sidecar files, read-only directories ----------
//
// A project rests in WAL mode (every read-write open sets it) with its WAL
// checkpointed away on close. A plain SQLITE_OPEN_READONLY open of such a
// file creates `-shm` / `-wal` beside it, and in a read-only directory cannot
// read it at all ("attempt to write a readonly database").

namespace {

std::set<std::string> names_in(const fs::path &dir) {
  std::set<std::string> out;
  for (const auto &e : fs::directory_iterator(dir))
    out.insert(e.path().filename().string());
  return out;
}

std::string journal_mode(const fs::path &db_path) {
  sqlite3 *db = nullptr;
  REQUIRE(sqlite3_open_v2(db_path.string().c_str(), &db, SQLITE_OPEN_READONLY,
                          nullptr) == SQLITE_OK);
  sqlite3_stmt *st = nullptr;
  REQUIRE(sqlite3_prepare_v2(db, "PRAGMA journal_mode;", -1, &st, nullptr) ==
          SQLITE_OK);
  std::string mode;
  if (sqlite3_step(st) == SQLITE_ROW)
    mode = reinterpret_cast<const char *>(sqlite3_column_text(st, 0));
  sqlite3_finalize(st);
  sqlite3_close(db);
  return mode;
}

/// chmod a directory read-only for the scope, restoring it after.
struct ReadOnlyDir {
  fs::path dir;
  explicit ReadOnlyDir(fs::path d) : dir(std::move(d)) {
    fs::permissions(dir, fs::perms::owner_read | fs::perms::owner_exec);
  }
  ~ReadOnlyDir() {
    std::error_code ec;
    fs::permissions(dir, fs::perms::owner_all, ec);
  }
  bool effective() const { return ::access(dir.c_str(), W_OK) != 0; }
};

} // namespace

TEST_CASE("ProjectDBProbe_WalProject_CreatesNoSidecarFiles",
          "[core][probe][wal]") {
  reusex::test_support::TempDir dir("test_projectdb_probe_wal");
  const fs::path p = dir.path / "project.rux";
  {
    ProjectDB db(p);
  }
  REQUIRE(journal_mode(p) == "wal");
  // journal_mode() itself is a plain open: clear what it left behind.
  fs::remove(p.string() + "-shm");
  fs::remove(p.string() + "-wal");
  const auto before = names_in(dir.path);
  REQUIRE(before == std::set<std::string>{"project.rux"});

  const auto r = ProjectDB::probe(p);
  CHECK(r.is_project);
  CHECK(r.schema_version == ProjectDB::latest_schema_version());
  CHECK(names_in(dir.path) == before);
}

TEST_CASE("ProjectDBProbe_LiveWal_SeesPagesOnlyInTheWal",
          "[core][probe][wal]") {
  // A read-write connection held open keeps its fresh tables in the WAL:
  // the main file alone is not a project yet, so the probe must read the
  // WAL (a plain read-only open) rather than the immutable main file.
  reusex::test_support::TempDir dir("test_projectdb_probe_wal");
  const fs::path p = dir.path / "project.rux";
  ProjectDB writer(p);
  std::error_code ec;
  REQUIRE(fs::file_size(p.string() + "-wal", ec) > 0);
  const auto r = ProjectDB::probe(p);
  CHECK(r.is_project);
  CHECK(r.schema_version == ProjectDB::latest_schema_version());
}

TEST_CASE("ProjectDB_WalProjectInReadOnlyDirectory_OpensReadOnly",
          "[core][probe][wal]") {
  reusex::test_support::TempDir dir("test_projectdb_probe_rodir");
  const fs::path p = dir.path / "project.rux";
  {
    ProjectDB db(p);
  }
  REQUIRE(journal_mode(p) == "wal");
  fs::remove(p.string() + "-shm");
  fs::remove(p.string() + "-wal");
  const auto before = names_in(dir.path);

  ReadOnlyDir guard(dir.path);
  if (!guard.effective())
    SKIP("running as a user who can write a 0500 directory (root?)");

  const auto r = ProjectDB::probe(p);
  CHECK(r.is_project);
  CHECK(r.error.empty());
  CHECK(r.schema_version == ProjectDB::latest_schema_version());

  {
    ProjectDB db(p, /*readOnly=*/true);
    CHECK(db.is_read_only());
    CHECK(db.schema_version() == ProjectDB::latest_schema_version());
    CHECK_FALSE(db.list_tables().empty());
    CHECK(db.sensor_frame_ids().empty());
  }
  CHECK(names_in(dir.path) == before);
}

TEST_CASE("ProjectDB_HardenedReadOnlyDirectory_ProbesAndOpensImmutable",
          "[core][probe][wal][hardened]") {
  // A server hardens every open; the immutable read-only open (and probe) of
  // a project in an unwritable directory must still work under it.
  reusex::test_support::TempDir dir("test_projectdb_probe_rodir_hardened");
  const fs::path p = dir.path / "project.rux";
  {
    ProjectDB db(p);
  }
  fs::remove(p.string() + "-shm");
  fs::remove(p.string() + "-wal");
  const auto before = names_in(dir.path);

  ReadOnlyDir guard(dir.path);
  if (!guard.effective())
    SKIP("running as a user who can write a 0500 directory (root?)");

  ProjectDB::set_hardened_by_default(true);
  struct Reset {
    ~Reset() { ProjectDB::set_hardened_by_default(false); }
  } reset;

  const auto r = ProjectDB::probe(p);
  CHECK(r.is_project);
  CHECK(r.error.empty());
  CHECK(r.schema_version == ProjectDB::latest_schema_version());
  {
    ProjectDB db(p, /*readOnly=*/true);
    CHECK(db.is_read_only());
    CHECK(db.schema_version() == ProjectDB::latest_schema_version());
    CHECK_FALSE(db.list_tables().empty());
  }
  CHECK(names_in(dir.path) == before);
}
