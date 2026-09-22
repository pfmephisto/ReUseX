// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/SensorIntrinsics.hpp>

#include "../../support/temp_path.hpp"

#include <opencv2/core.hpp>
#include <sqlite3.h>

#include <algorithm>
#include <array>
#include <filesystem>
#include <string>
#include <vector>

using namespace reusex;
namespace fs = std::filesystem;

namespace {

struct TempPath : reusex::test_support::TempPath {
  TempPath() : reusex::test_support::TempPath("test_multiscan") {}
};

void sql_exec(sqlite3 *db, const char *sql) {
  char *err = nullptr;
  int rc = sqlite3_exec(db, sql, nullptr, nullptr, &err);
  std::string msg = err ? err : "";
  sqlite3_free(err);
  REQUIRE(rc == SQLITE_OK);
}

cv::Mat minimal_color() {
  return cv::Mat(4, 4, CV_8UC3, cv::Scalar(100, 100, 100));
}

std::array<double, 16> identity_pose() {
  return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
}

core::SensorIntrinsics minimal_intrinsics() {
  core::SensorIntrinsics i;
  i.fx = i.fy = 100.0;
  i.cx = i.cy = 50.0;
  i.width = 100;
  i.height = 100;
  return i;
}

// Save a full sensor frame with the given node_id and scan_id (no
// depth/confidence).
void save_frame(ProjectDB &db, int node_id, int scan_id) {
  db.save_sensor_frame(node_id, minimal_color(), cv::Mat(), cv::Mat(),
                       identity_pose(), minimal_intrinsics(),
                       static_cast<double>(node_id), scan_id);
}

// Build a minimal v16 SQLite database with n sensor frames.
// Used to test that migrateToV17 back-fills scan_id correctly.
void build_v16_fixture(const fs::path &path, int n_frames = 2) {
  sqlite3 *db = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &db) == SQLITE_OK);

  sql_exec(db, "CREATE TABLE schema_version ("
               "  version INTEGER NOT NULL, applied_at TEXT, description TEXT"
               ");");
  sql_exec(db, "INSERT INTO schema_version (version, description) "
               "VALUES (16, 'test fixture v16');");

  sql_exec(db, "CREATE TABLE sensor_frames ("
               "  node_id      INTEGER PRIMARY KEY,"
               "  color        BLOB,"
               "  depth        BLOB,"
               "  confidence   BLOB,"
               "  transform    BLOB,"
               "  width        INTEGER,"
               "  height       INTEGER,"
               "  camera_model TEXT,"
               "  timestamp    REAL"
               ");");

  for (int i = 1; i <= n_frames; ++i) {
    std::string sql = "INSERT INTO sensor_frames (node_id, width, height) "
                      "VALUES (" +
                      std::to_string(i) + ", 100, 100);";
    sql_exec(db, sql.c_str());
  }

  sqlite3_close(db);
}

} // namespace

// ── Migration v17 ─────────────────────────────────────────────────────────

TEST_CASE("Migration v17: scans table and legacy back-fill",
          "[multiscan][migration]") {
  TempPath tmp;
  build_v16_fixture(tmp.path, 3);

  // Opening triggers migrateToV17.
  {
    ProjectDB db(tmp.path);
  }

  // Verify via raw SQLite: one legacy scan row and no NULL scan_ids.
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(tmp.path.string().c_str(), &raw) == SQLITE_OK);

  {
    sqlite3_stmt *stmt;
    REQUIRE(sqlite3_prepare_v2(raw, "SELECT COUNT(*) FROM scans;", -1, &stmt,
                               nullptr) == SQLITE_OK);
    REQUIRE(sqlite3_step(stmt) == SQLITE_ROW);
    REQUIRE(sqlite3_column_int(stmt, 0) == 1);
    sqlite3_finalize(stmt);
  }
  {
    sqlite3_stmt *stmt;
    REQUIRE(sqlite3_prepare_v2(
                raw,
                "SELECT COUNT(*) FROM sensor_frames WHERE scan_id IS NULL;", -1,
                &stmt, nullptr) == SQLITE_OK);
    REQUIRE(sqlite3_step(stmt) == SQLITE_ROW);
    REQUIRE(sqlite3_column_int(stmt, 0) == 0); // all frames got scan_id = 1
    sqlite3_finalize(stmt);
  }

  sqlite3_close(raw);

  // ProjectDB API: legacy scan is visible.
  ProjectDB ro(tmp.path, /*readOnly=*/true);
  auto scans = ro.scans();
  REQUIRE(scans.size() == 1);
  REQUIRE(scans[0].id == 1);
  REQUIRE(scans[0].source_path.empty()); // legacy record has no source path
  REQUIRE(scans[0].id_offset == 0);
}

TEST_CASE("Migration v17: empty DB has no legacy scan",
          "[multiscan][migration]") {
  TempPath tmp;
  // Open fresh DB — no frames, so no legacy scan should be created.
  {
    ProjectDB db(tmp.path);
  }

  ProjectDB ro(tmp.path, /*readOnly=*/true);
  REQUIRE(ro.scans().empty());
}

TEST_CASE("Migration v17: idempotent on re-open", "[multiscan][migration]") {
  TempPath tmp;
  build_v16_fixture(tmp.path, 2);
  {
    ProjectDB db(tmp.path);
  } // first open: migrates
  {
    ProjectDB db(tmp.path);
  } // second open: no-op

  ProjectDB ro(tmp.path, /*readOnly=*/true);
  REQUIRE(ro.scans().size() == 1);
}

// ── create_scan ────────────────────────────────────────────────────────────

TEST_CASE("create_scan: first session gets offset 0", "[multiscan]") {
  TempPath tmp;
  ProjectDB db(tmp.path);

  auto rec = db.create_scan("session1.db");
  REQUIRE(rec.id == 1);
  REQUIRE(rec.source_path == "session1.db");
  REQUIRE(rec.id_offset == 0);
  REQUIRE(!rec.imported_at.empty());
}

TEST_CASE("create_scan: offset advances to max node_id", "[multiscan]") {
  TempPath tmp;
  ProjectDB db(tmp.path);

  auto scan1 = db.create_scan("s1.db");
  REQUIRE(scan1.id_offset == 0);

  save_frame(db, 1 + scan1.id_offset, scan1.id); // node_id 1
  save_frame(db, 2 + scan1.id_offset, scan1.id); // node_id 2
  save_frame(db, 3 + scan1.id_offset, scan1.id); // node_id 3

  auto scan2 = db.create_scan("s2.db");
  REQUIRE(scan2.id == 2);
  REQUIRE(scan2.id_offset == 3); // max(node_id) from session 1
}

TEST_CASE("create_scan: duplicate source warns but still creates",
          "[multiscan]") {
  TempPath tmp;
  ProjectDB db(tmp.path);

  db.create_scan("same.db");
  // Second import of same source: should not throw (warning only).
  auto rec2 = db.create_scan("same.db");
  REQUIRE(rec2.id == 2);

  REQUIRE(db.scans().size() == 2);
}

TEST_CASE("create_scan: read-only DB throws", "[multiscan]") {
  TempPath tmp;
  {
    ProjectDB db(tmp.path);
  } // create it

  ProjectDB ro(tmp.path, /*readOnly=*/true);
  REQUIRE_THROWS(ro.create_scan("session.db"));
}

// ── Two-session import: no ID collision ────────────────────────────────────

TEST_CASE("Two sessions: node_ids don't collide", "[multiscan]") {
  TempPath tmp;
  ProjectDB db(tmp.path);

  // Session 1: RTABMap node_ids 1, 2, 3 → stored as 1, 2, 3.
  auto scan1 = db.create_scan("session1.db");
  save_frame(db, 1 + scan1.id_offset, scan1.id);
  save_frame(db, 2 + scan1.id_offset, scan1.id);
  save_frame(db, 3 + scan1.id_offset, scan1.id);

  // Session 2: RTABMap node_ids also start at 1 → stored as 4, 5, 6.
  auto scan2 = db.create_scan("session2.db");
  REQUIRE(scan2.id_offset == 3);
  save_frame(db, 1 + scan2.id_offset, scan2.id); // node_id 4
  save_frame(db, 2 + scan2.id_offset, scan2.id); // node_id 5
  save_frame(db, 3 + scan2.id_offset, scan2.id); // node_id 6

  auto ids = db.sensor_frame_ids();
  REQUIRE(ids.size() == 6);
  std::sort(ids.begin(), ids.end());
  REQUIRE(ids == std::vector<int>{1, 2, 3, 4, 5, 6});
}

TEST_CASE("Two sessions: project_summary per-scan breakdown", "[multiscan]") {
  TempPath tmp;
  ProjectDB db(tmp.path);

  auto scan1 = db.create_scan("/scans/floor1.db");
  save_frame(db, 1 + scan1.id_offset, scan1.id);
  save_frame(db, 2 + scan1.id_offset, scan1.id);

  auto scan2 = db.create_scan("/scans/floor2.db");
  save_frame(db, 1 + scan2.id_offset, scan2.id);
  save_frame(db, 2 + scan2.id_offset, scan2.id);
  save_frame(db, 3 + scan2.id_offset, scan2.id);

  auto summary = db.project_summary();
  REQUIRE(summary.sensor_frames.total_count == 5);
  REQUIRE(summary.sensor_frames.scans.size() == 2);

  const auto &s1 = summary.sensor_frames.scans[0];
  REQUIRE(s1.scan_id == scan1.id);
  REQUIRE(s1.frame_count == 2);
  REQUIRE(s1.source_path == "/scans/floor1.db");

  const auto &s2 = summary.sensor_frames.scans[1];
  REQUIRE(s2.scan_id == scan2.id);
  REQUIRE(s2.frame_count == 3);
  REQUIRE(s2.source_path == "/scans/floor2.db");
}

// ── scans() round-trip ────────────────────────────────────────────────────

TEST_CASE("scans(): lists all records in import order", "[multiscan]") {
  TempPath tmp;
  ProjectDB db(tmp.path);

  db.create_scan("alpha.db");
  save_frame(db, 1, 1);
  db.create_scan("beta.db");

  auto recs = db.scans();
  REQUIRE(recs.size() == 2);
  REQUIRE(recs[0].source_path == "alpha.db");
  REQUIRE(recs[1].source_path == "beta.db");
  REQUIRE(recs[1].id_offset == 1); // one frame from scan 1
}

// ── Back-compat: single-scan workflow unchanged ───────────────────────────

TEST_CASE("Single-scan back-compat: sensor_frame_ids works", "[multiscan]") {
  TempPath tmp;
  ProjectDB db(tmp.path);

  auto scan = db.create_scan("only.db");
  save_frame(db, 1 + scan.id_offset, scan.id);
  save_frame(db, 2 + scan.id_offset, scan.id);
  save_frame(db, 3 + scan.id_offset, scan.id);

  REQUIRE(db.sensor_frame_ids().size() == 3);

  auto summary = db.project_summary();
  REQUIRE(summary.sensor_frames.total_count == 3);
  REQUIRE(summary.sensor_frames.scans.size() == 1);
}
