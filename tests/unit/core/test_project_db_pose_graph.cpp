// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Pose-graph edge storage in ProjectDB (schema v15, #265).
//
// Three properties are pinned:
//  1. A pre-v15 fixture migrates cleanly: the table appears and is empty.
//  2. Edges survive a round trip (save → list) with correct field values.
//  3. Running save a second time atomically replaces the first write.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>

#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>

using reusex::ProjectDB;
namespace fs = std::filesystem;
using Catch::Approx;
using reusex::test_support::TempPath;

namespace {

void exec(sqlite3 *db, const char *sql) {
  char *err = nullptr;
  int rc = sqlite3_exec(db, sql, nullptr, nullptr, &err);
  std::string msg = err ? err : "";
  sqlite3_free(err);
  REQUIRE(rc == SQLITE_OK);
}

/// Minimal v14 fixture: schema_version table stamped at 14, no
/// pose_graph_edges.
void buildV14Fixture(const fs::path &path) {
  sqlite3 *db = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &db) == SQLITE_OK);

  exec(db, "PRAGMA journal_mode=WAL;");
  exec(db, "CREATE TABLE schema_version (version INTEGER NOT NULL, "
           "applied_at TEXT, description TEXT);");
  exec(db, "INSERT INTO schema_version (version, description) VALUES "
           "(14, 'test fixture v14');");

  // Minimal tables that ProjectDB expects to find when it opens:
  exec(db, "CREATE TABLE point_clouds (id INTEGER PRIMARY KEY AUTOINCREMENT, "
           "name TEXT UNIQUE, point_type TEXT, point_count INTEGER, "
           "point_step INTEGER, width INTEGER, height INTEGER, stage TEXT, "
           "parameters TEXT);");
  exec(db,
       "CREATE TABLE point_cloud_data (id INTEGER PRIMARY KEY AUTOINCREMENT, "
       "cloud_id INTEGER, data BLOB, chunk_index INTEGER);");
  exec(db,
       "CREATE TABLE label_definitions (cloud_id INTEGER, label_id INTEGER, "
       "name TEXT, PRIMARY KEY (cloud_id, label_id));");
  exec(db, "CREATE TABLE meshes (id INTEGER PRIMARY KEY AUTOINCREMENT, "
           "name TEXT UNIQUE, format TEXT, data BLOB, vertex_count INTEGER, "
           "face_count INTEGER, stage TEXT, parameters TEXT);");
  exec(db, "CREATE TABLE sensor_frames (node_id INTEGER PRIMARY KEY, "
           "color_image BLOB, depth_image BLOB, confidence_image BLOB, "
           "transform BLOB, fx REAL, fy REAL, cx REAL, cy REAL, "
           "camera_model TEXT, timestamp REAL, local_transform BLOB);");
  exec(db,
       "CREATE TABLE pipeline_log (id INTEGER PRIMARY KEY AUTOINCREMENT, "
       "stage TEXT, started_at TEXT DEFAULT CURRENT_TIMESTAMP, "
       "finished_at TEXT, success INTEGER, error_msg TEXT, parameters TEXT);");
  exec(db, "CREATE TABLE instances (id INTEGER PRIMARY KEY AUTOINCREMENT, "
           "cloud_id INTEGER REFERENCES point_clouds(id) ON DELETE CASCADE, "
           "label_id INTEGER, point_count INTEGER, centroid_x REAL, "
           "centroid_y REAL, centroid_z REAL, bounding_box BLOB);");
  exec(db, "CREATE TABLE segmentation_images (node_id INTEGER PRIMARY KEY "
           "REFERENCES sensor_frames(node_id) ON DELETE CASCADE, "
           "label_image BLOB NOT NULL);");
  exec(db,
       "CREATE TABLE panoramic_images (id INTEGER PRIMARY KEY AUTOINCREMENT, "
       "filename TEXT, image BLOB, node_id INTEGER, pose BLOB, "
       "timestamp REAL);");
  exec(
      db,
      "CREATE TABLE building_components (id INTEGER PRIMARY KEY AUTOINCREMENT, "
      "name TEXT UNIQUE, type TEXT, parent_id INTEGER, "
      "vertex_data BLOB, confidence REAL, guid TEXT, metadata TEXT, notes "
      "TEXT);");
  exec(db,
       "CREATE TABLE material_passports (id INTEGER PRIMARY KEY AUTOINCREMENT, "
       "document_guid TEXT UNIQUE NOT NULL, created_at TEXT, "
       "schema_version INTEGER DEFAULT 1);");
  exec(db,
       "CREATE TABLE instance_materials (id INTEGER PRIMARY KEY AUTOINCREMENT, "
       "cloud_name TEXT NOT NULL, label_id INTEGER NOT NULL, "
       "material_guid TEXT NOT NULL);");
  exec(db,
       "CREATE TABLE gaussian_splats (id INTEGER PRIMARY KEY AUTOINCREMENT, "
       "name TEXT UNIQUE, format TEXT, gaussian_count INTEGER, "
       "sh_degree INTEGER, byte_size INTEGER, created_at TEXT, "
       "stage TEXT, parameters TEXT);");
  exec(
      db,
      "CREATE TABLE gaussian_splat_data (id INTEGER PRIMARY KEY AUTOINCREMENT, "
      "splat_id INTEGER, chunk_index INTEGER, data BLOB);");
  exec(db,
       "CREATE TABLE mesh_texture_data (id INTEGER PRIMARY KEY AUTOINCREMENT, "
       "mesh_id INTEGER, tex_name TEXT, format TEXT, width INTEGER, "
       "height INTEGER, image_data BLOB);");
  exec(db, "CREATE TABLE projects (id INTEGER PRIMARY KEY AUTOINCREMENT, "
           "name TEXT, address TEXT, year_of_construction INTEGER, "
           "survey_date TEXT, organisation TEXT, description TEXT);");
  exec(db, "CREATE TABLE property_definitions (id INTEGER PRIMARY KEY "
           "AUTOINCREMENT, "
           "document_guid TEXT NOT NULL, property_name TEXT NOT NULL, "
           "property_type TEXT, unit TEXT, description TEXT);");
  exec(db,
       "CREATE TABLE property_values (id INTEGER PRIMARY KEY AUTOINCREMENT, "
       "document_guid TEXT NOT NULL, property_name TEXT NOT NULL, "
       "value TEXT);");
  exec(db,
       "CREATE TABLE transaction_log (id INTEGER PRIMARY KEY AUTOINCREMENT, "
       "document_guid TEXT NOT NULL, transaction_type TEXT NOT NULL, "
       "transaction_date TEXT, quantity REAL, unit TEXT, notes TEXT, "
       "document_reference TEXT);");
  exec(db, "CREATE TABLE glass_confidence_images (node_id INTEGER PRIMARY KEY "
           "REFERENCES sensor_frames(node_id) ON DELETE CASCADE, "
           "confidence_image BLOB NOT NULL);");

  sqlite3_close(db);
}

} // namespace

TEST_CASE("pose_graph migration v14->v15: table created, initially empty",
          "[projectdb][migration][pose_graph]") {
  TempPath tmp("test_pose_graph_migration");
  buildV14Fixture(tmp.path);

  ProjectDB db(tmp.path); // triggers migration
  CHECK(db.list_pose_graph_edges().empty());
  CHECK_FALSE(db.has_pose_graph());
}

TEST_CASE("pose_graph round trip: save and list", "[projectdb][pose_graph]") {
  TempPath tmp("test_pose_graph_roundtrip");
  ProjectDB db(tmp.path);

  const std::vector<ProjectDB::PoseGraphEdge> edges = {
      {1, 2, "odometry", 0.012, 1000.0},
      {2, 3, "odometry", 0.015, 1000.0},
      {1, 10, "loop_closure", 0.45, 400.0},
      {5, 20, "panorama", 0.23, std::numeric_limits<double>::quiet_NaN()},
  };

  db.save_pose_graph_edges(edges);

  REQUIRE(db.has_pose_graph());
  const auto out = db.list_pose_graph_edges();
  REQUIRE(out.size() == 4);

  CHECK(out[0].from_node_id == 1);
  CHECK(out[0].to_node_id == 2);
  CHECK(out[0].edge_type == "odometry");
  CHECK(out[0].residual == Approx(0.012));
  CHECK(out[0].weight == Approx(1000.0));

  CHECK(out[2].edge_type == "loop_closure");
  CHECK(out[2].residual == Approx(0.45));

  CHECK(out[3].edge_type == "panorama");
  CHECK(std::isnan(out[3].weight)); // NaN → NULL → NaN on read-back
}

TEST_CASE("pose_graph save replaces previous run atomically",
          "[projectdb][pose_graph]") {
  TempPath tmp("test_pose_graph_replace");
  ProjectDB db(tmp.path);

  // First optimize run: 3 edges
  db.save_pose_graph_edges({
      {1, 2, "odometry", 0.01, 1000.0},
      {2, 3, "odometry", 0.02, 1000.0},
      {1, 5, "loop_closure", 0.5, 200.0},
  });
  REQUIRE(db.list_pose_graph_edges().size() == 3);

  // Second optimize run: 2 edges (e.g., loop edge was rejected this time)
  db.save_pose_graph_edges({
      {1, 2, "odometry", 0.008, 1000.0},
      {2, 3, "odometry", 0.011, 1000.0},
  });
  const auto out = db.list_pose_graph_edges();
  REQUIRE(out.size() == 2);
  CHECK(out[0].residual == Approx(0.008));

  // Clear: save empty vector
  db.save_pose_graph_edges({});
  CHECK(db.list_pose_graph_edges().empty());
  CHECK_FALSE(db.has_pose_graph());
}
