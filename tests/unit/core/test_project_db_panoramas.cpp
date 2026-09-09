// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>

#include "../../support/temp_path.hpp"

#include <opencv2/core/mat.hpp>
#include <sqlite3.h>

#include <array>
#include <filesystem>
#include <string>

using namespace reusex;
namespace fs = std::filesystem;

namespace {

// Keeps the original "test_pano_db" filename prefix while delegating
// uniqueness/cleanup to the shared helper.
struct TempPath : reusex::test_support::TempPath {
  TempPath() : reusex::test_support::TempPath("test_pano_db") {}
};

void exec(sqlite3 *db, const char *sql) {
  char *err = nullptr;
  int rc = sqlite3_exec(db, sql, nullptr, nullptr, &err);
  std::string msg = err ? err : "";
  sqlite3_free(err);
  REQUIRE(rc == SQLITE_OK);
}

// Build a schema-version-10 database carrying two panoramas in the *pre-v11*
// panoramic_images shape: no pose / pose_source / align_inliers / align_rms
// columns and no panorama_segmentation table.
void buildV10PanoramaFixture(const fs::path &path) {
  sqlite3 *db = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &db) == SQLITE_OK);

  exec(db, "CREATE TABLE schema_version (version INTEGER NOT NULL, "
           "applied_at TEXT, description TEXT);");
  exec(db, "INSERT INTO schema_version (version, description) VALUES "
           "(10, 'test fixture');");

  // ProjectDB::validateSchema() requires the legacy passport tables on every
  // open (read-only included), so a realistic fixture must carry them.
  exec(db, "CREATE TABLE projects (id TEXT PRIMARY KEY);");
  exec(db, "CREATE TABLE property_definitions (id TEXT PRIMARY KEY);");
  exec(db, "CREATE TABLE material_passports (id TEXT PRIMARY KEY, "
           "document_guid TEXT UNIQUE, created_at TEXT);");
  exec(db, "CREATE TABLE passport_property_values (id TEXT PRIMARY KEY);");
  exec(db, "CREATE TABLE passport_log (id TEXT PRIMARY KEY);");

  // v5 shape of panoramic_images (unchanged through v10).
  exec(db, "CREATE TABLE panoramic_images (id INTEGER PRIMARY KEY "
           "AUTOINCREMENT, filename TEXT UNIQUE, image BLOB, "
           "timestamp REAL, node_id INTEGER);");
  exec(db, "INSERT INTO panoramic_images (filename, image, timestamp, node_id) "
           "VALUES ('R0010001.JPG', X'00', 1000.5, 7);");
  exec(db, "INSERT INTO panoramic_images (filename, image, timestamp, node_id) "
           "VALUES ('R0010002.JPG', X'00', NULL, NULL);");

  sqlite3_close(db);
}

} // namespace

// Regression test for the v11 read-only blocker: runMigrations() only runs on
// read-write opens, so a pre-v11 project opened read-only still has the old
// panoramic_images columns. list_panoramic_images() must degrade to the legacy
// column list instead of throwing "no such column: pose" (which took out
// `rux export rhino` / `speckle` and silently dropped panoramas in `rux view`).
TEST_CASE("ListPanoramicImages_PreV11SchemaReadOnlyOpen_"
          "DegradesToLegacyColumnsWithoutThrowing",
          "[projectdb][panorama][migration]") {
  TempPath tmp;
  buildV10PanoramaFixture(tmp.path);

  ProjectDB db(tmp.path, /*readOnly=*/true);

  // No migration was applied — the DB is still on its on-disk version.
  REQUIRE(db.schema_version() == 10);

  std::vector<ProjectDB::PanoramicImage> panos;
  REQUIRE_NOTHROW(panos = db.list_panoramic_images());
  REQUIRE(panos.size() == 2);

  // Ordered by filename; legacy columns are read, v11 fields keep defaults.
  REQUIRE(panos[0].filename == "R0010001.JPG");
  REQUIRE(panos[0].timestamp == Catch::Approx(1000.5));
  REQUIRE(panos[0].node_id == 7);
  REQUIRE(panos[0].has_pose == false);
  REQUIRE(panos[0].pose_source == "timestamp");
  REQUIRE(panos[0].align_inliers == -1);
  REQUIRE(panos[0].align_rms == Catch::Approx(-1.0));

  REQUIRE(panos[1].filename == "R0010002.JPG");
  REQUIRE(panos[1].timestamp == Catch::Approx(-1.0));
  REQUIRE(panos[1].node_id == -1);
  REQUIRE(panos[1].has_pose == false);

  // The v11 panorama_segmentation table is absent too; readers must cope.
  REQUIRE_FALSE(db.has_panorama_segmentation(panos[0].id));
  REQUIRE_NOTHROW(db.panorama_segmentation(panos[0].id));
}

TEST_CASE("ProjectDb_PreV11PanoramaProjectReadWriteOpen_MigratesToV11",
          "[projectdb][panorama][migration]") {
  TempPath tmp;
  buildV10PanoramaFixture(tmp.path);

  {
    ProjectDB db(tmp.path);
    REQUIRE(db.schema_version() == 11);
    auto panos = db.list_panoramic_images();
    REQUIRE(panos.size() == 2);
    // Existing rows survive with NULL pose columns.
    REQUIRE(panos[0].has_pose == false);
  }

  // Read-only re-open now sees the migrated schema.
  ProjectDB ro(tmp.path, /*readOnly=*/true);
  REQUIRE(ro.schema_version() == 11);
  REQUIRE(ro.list_panoramic_images().size() == 2);
}

TEST_CASE("SavePanoramaPose_PoseBlobAndMetrics_RoundTripsAcrossReopen",
          "[projectdb][panorama]") {
  TempPath tmp;
  ProjectDB db(tmp.path);

  // node_id -1: unmatched, so no sensor_frames FK to satisfy here.
  db.save_panoramic_image("R0010003.JPG", std::vector<uint8_t>{0x01, 0x02},
                          2000.25, -1);
  auto panos = db.list_panoramic_images();
  REQUIRE(panos.size() == 1);
  REQUIRE(panos[0].has_pose == false);
  REQUIRE(panos[0].pose_source == "timestamp");
  const int id = panos[0].id;

  // Row-major world pose: identity rotation with a translation.
  const std::array<double, 16> pose = {1, 0, 0, 1.5,  0, 1, 0, -2.25,
                                       0, 0, 1, 0.75, 0, 0, 0, 1};
  db.save_panorama_pose(id, pose, /*inliers=*/123, /*rms=*/0.4375);

  auto after = db.list_panoramic_images();
  REQUIRE(after.size() == 1);
  REQUIRE(after[0].has_pose == true);
  REQUIRE(after[0].pose_source == "aligned");
  REQUIRE(after[0].align_inliers == 123);
  REQUIRE(after[0].align_rms == Catch::Approx(0.4375));
  for (size_t i = 0; i < pose.size(); ++i)
    REQUIRE(after[0].pose[i] == Catch::Approx(pose[i]));

  // Values persist across a fresh read-only connection.
  ProjectDB ro(tmp.path, /*readOnly=*/true);
  auto reread = ro.list_panoramic_images();
  REQUIRE(reread.size() == 1);
  REQUIRE(reread[0].has_pose == true);
  REQUIRE(reread[0].align_inliers == 123);
  REQUIRE(reread[0].pose[3] == Catch::Approx(1.5));
  REQUIRE(reread[0].pose[7] == Catch::Approx(-2.25));
  REQUIRE(reread[0].pose[11] == Catch::Approx(0.75));
}
