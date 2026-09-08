// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <geometry/BuildingComponent.hpp>
#include <geometry/component_persistence.hpp>

#include "../../support/temp_path.hpp"

#include <cstdio>
#include <filesystem>
#include <iostream>

using namespace reusex;
using namespace reusex::geometry;
using Catch::Approx;

namespace fs = std::filesystem;

// Helper: create a temp database path that auto-cleans
struct TempDB : reusex::test_support::TempPath {
  TempDB() : TempPath("test_projectdb_comp") {}
};

// Helper: build a rectangular window component
static BuildingComponent make_window(const std::string &name) {
  BuildingComponent c;
  c.name = name;
  c.type = ComponentType::window;
  c.boundary.vertices = {
      {0.0, 0.0, 0.0}, {1.2, 0.0, 0.0}, {1.2, 0.0, 1.5}, {0.0, 0.0, 1.5}};
  c.boundary.plane = Eigen::Vector4d(0, 1, 0, 0); // y=0 plane
  c.parent_id = -1;
  c.confidence = 0.95;
  c.notes = "detected by YOLO";
  c.data = WindowData{"casement", 2, true};
  return c;
}

static BuildingComponent make_door(const std::string &name) {
  BuildingComponent c;
  c.name = name;
  c.type = ComponentType::door;
  c.boundary.vertices = {
      {2.0, 0.0, 0.0}, {3.0, 0.0, 0.0}, {3.0, 0.0, 2.1}, {2.0, 0.0, 2.1}};
  c.boundary.plane = Eigen::Vector4d(0, 1, 0, 0);
  c.confidence = 0.88;
  c.data = DoorData{"single", "left"};
  return c;
}

TEST_CASE("ProjectDB latest schema version on fresh DB",
          "[projectdb][components]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  REQUIRE(db.schema_version() == 11);
}

TEST_CASE("ProjectDB building component save/load round-trip",
          "[projectdb][components]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  auto original = make_window("win1");
  save_building_component(db, original);

  REQUIRE(db.has_building_component("win1"));

  auto loaded = building_component(db, "win1");
  REQUIRE(loaded.name == "win1");
  REQUIRE(loaded.type == ComponentType::window);
  REQUIRE(loaded.parent_id == -1);
  REQUIRE(loaded.confidence == Approx(0.95));
  REQUIRE(loaded.notes == "detected by YOLO");

  // Verify vertices match exactly
  REQUIRE(loaded.boundary.vertices.size() == original.boundary.vertices.size());
  for (size_t i = 0; i < original.boundary.vertices.size(); ++i) {
    REQUIRE(loaded.boundary.vertices[i].x() ==
            Approx(original.boundary.vertices[i].x()));
    REQUIRE(loaded.boundary.vertices[i].y() ==
            Approx(original.boundary.vertices[i].y()));
    REQUIRE(loaded.boundary.vertices[i].z() ==
            Approx(original.boundary.vertices[i].z()));
  }

  // Verify plane
  for (int i = 0; i < 4; ++i) {
    REQUIRE(loaded.boundary.plane[i] == Approx(original.boundary.plane[i]));
  }

  // Verify type-specific data
  auto &wd = std::get<WindowData>(loaded.data);
  REQUIRE(wd.style == "casement");
  REQUIRE(wd.pane_count == 2);
  REQUIRE(wd.operable == true);
}

TEST_CASE("ProjectDB building component UPSERT replaces existing",
          "[projectdb][components]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  auto c1 = make_window("win1");
  save_building_component(db, c1);

  // Overwrite with door
  auto c2 = make_door("win1");
  save_building_component(db, c2);

  auto loaded = building_component(db, "win1");
  REQUIRE(loaded.type == ComponentType::door);
  REQUIRE(std::holds_alternative<DoorData>(loaded.data));
}

TEST_CASE("ProjectDB has_building_component", "[projectdb][components]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  REQUIRE_FALSE(db.has_building_component("nonexistent"));

  save_building_component(db, make_window("win1"));
  REQUIRE(db.has_building_component("win1"));
}

TEST_CASE("ProjectDB delete_building_component", "[projectdb][components]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  save_building_component(db, make_window("win1"));
  REQUIRE(db.has_building_component("win1"));

  db.delete_building_component("win1");
  REQUIRE_FALSE(db.has_building_component("win1"));
}

TEST_CASE("ProjectDB list_building_components", "[projectdb][components]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  REQUIRE(db.list_building_components().empty());

  save_building_component(db, make_window("win1"));
  save_building_component(db, make_window("win2"));
  save_building_component(db, make_door("door1"));

  auto names = db.list_building_components();
  REQUIRE(names.size() == 3);
  REQUIRE(names[0] == "win1");
  REQUIRE(names[1] == "win2");
  REQUIRE(names[2] == "door1");
}

TEST_CASE("ProjectDB list_building_components by type",
          "[projectdb][components]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  save_building_component(db, make_window("win1"));
  save_building_component(db, make_window("win2"));
  save_building_component(db, make_door("door1"));

  auto windows = list_building_components(db, ComponentType::window);
  REQUIRE(windows.size() == 2);
  REQUIRE(windows[0] == "win1");
  REQUIRE(windows[1] == "win2");

  auto doors = list_building_components(db, ComponentType::door);
  REQUIRE(doors.size() == 1);
  REQUIRE(doors[0] == "door1");

  auto walls = list_building_components(db, ComponentType::wall);
  REQUIRE(walls.empty());
}

TEST_CASE("ProjectDB building_component_count", "[projectdb][components]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  REQUIRE(db.building_component_count() == 0);

  save_building_component(db, make_window("win1"));
  save_building_component(db, make_door("door1"));
  REQUIRE(db.building_component_count() == 2);
}

TEST_CASE("ProjectDB building_component throws for missing",
          "[projectdb][components]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  REQUIRE_THROWS_AS(building_component(db, "nonexistent"), std::runtime_error);
}

TEST_CASE("ProjectDB project_summary includes component info",
          "[projectdb][components]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  save_building_component(db, make_window("win1"));
  save_building_component(db, make_window("win2"));
  save_building_component(db, make_door("door1"));

  auto summary = db.project_summary();
  REQUIRE(summary.components.total_count == 3);
  REQUIRE(summary.components.count_by_type["window"] == 2);
  REQUIRE(summary.components.count_by_type["door"] == 1);
}

TEST_CASE("ProjectDB door data round-trip with all fields",
          "[projectdb][components]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  auto original = make_door("door1");
  save_building_component(db, original);

  auto loaded = building_component(db, "door1");
  REQUIRE(loaded.type == ComponentType::door);

  auto &dd = std::get<DoorData>(loaded.data);
  REQUIRE(dd.style == "single");
  REQUIRE(dd.swing == "left");
}

TEST_CASE("ProjectDB auto-generates a guid on save",
          "[projectdb][components][guid]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  auto c = make_window("win1"); // guid empty
  REQUIRE(c.guid.empty());
  save_building_component(db, c);

  auto loaded = building_component(db, "win1");
  REQUIRE_FALSE(loaded.guid.empty());
  // UUID-v4-like: contains hyphens.
  REQUIRE(loaded.guid.find('-') != std::string::npos);
}

TEST_CASE("ProjectDB honours a caller-supplied guid",
          "[projectdb][components][guid]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  auto c = make_window("win1");
  c.guid = "my-fixed-guid";
  save_building_component(db, c);

  REQUIRE(building_component(db, "win1").guid == "my-fixed-guid");
}

TEST_CASE("ProjectDB guid is stable across upsert",
          "[projectdb][components][guid]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  save_building_component(db, make_window("win1"));
  std::string g1 = building_component(db, "win1").guid;

  // Re-save under the same name with different fields.
  auto c2 = make_window("win1");
  c2.confidence = 0.10;
  save_building_component(db, c2);

  std::string g2 = building_component(db, "win1").guid;
  REQUIRE(g1 == g2);
}

TEST_CASE("ProjectDB update_building_component_by_guid renames and edits",
          "[projectdb][components][guid]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  save_building_component(db, make_window("win1"));
  auto c = building_component(db, "win1");
  std::string guid = c.guid;

  // Edit mutable fields, including a rename.
  c.name = "renamed_window";
  c.notes = "edited via csv";
  c.confidence = 0.42;
  c.parent_id = 7;
  std::get<WindowData>(c.data).pane_count = 4;
  update_building_component_by_guid(db, c);

  REQUIRE_FALSE(db.has_building_component("win1")); // old name gone
  REQUIRE(db.has_building_component("renamed_window"));

  auto loaded = building_component(db, "renamed_window");
  REQUIRE(loaded.guid == guid); // guid preserved
  REQUIRE(loaded.notes == "edited via csv");
  REQUIRE(loaded.confidence == Approx(0.42));
  REQUIRE(loaded.parent_id == 7);
  REQUIRE(std::get<WindowData>(loaded.data).pane_count == 4);
}

TEST_CASE("ProjectDB update_building_component_by_guid throws for unknown guid",
          "[projectdb][components][guid]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  auto c = make_window("win1");
  c.guid = "does-not-exist";
  REQUIRE_THROWS_AS(update_building_component_by_guid(db, c),
                    std::runtime_error);
}

TEST_CASE("ProjectDB persists source_instance_guid provenance (issue #211)",
          "[projectdb][components][provenance]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  // Save with provenance set (as create_windows would).
  auto c = make_window("win1");
  c.source_instance_guid = "instances-guid-42";
  save_building_component(db, c);

  auto loaded = building_component(db, "win1");
  REQUIRE(loaded.source_instance_guid == "instances-guid-42");

  // The guid-update path (used by CSV import) must also round-trip it.
  loaded.source_instance_guid = "instances-guid-99";
  update_building_component_by_guid(db, loaded);
  REQUIRE(building_component(db, "win1").source_instance_guid ==
          "instances-guid-99");
}

TEST_CASE("ProjectDB leaves source_instance_guid empty when unset",
          "[projectdb][components][provenance]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  save_building_component(db, make_window("win1")); // no provenance
  REQUIRE(building_component(db, "win1").source_instance_guid.empty());
}

// --- Core-owned persistence contract (#227) ---------------------------------

TEST_CASE("ComponentRecord mapping round-trips a BuildingComponent",
          "[projectdb][components][record]") {
  auto original = make_window("win1");
  original.guid = "fixed-guid";
  original.source_instance_guid = "instances-guid-7";
  original.parent_id = 3;

  auto record = to_component_record(original);

  // The record mirrors the table columns: geometry as blobs, everything
  // type-specific folded into the opaque metadata JSON.
  REQUIRE(record.name == "win1");
  REQUIRE(record.guid == "fixed-guid");
  REQUIRE(record.type == "window");
  REQUIRE(record.vertex_data.size() ==
          original.boundary.vertices.size() * 3 * sizeof(double));
  REQUIRE(record.plane[1] == Approx(1.0));
  REQUIRE(record.parent_id == 3);
  REQUIRE(record.confidence == Approx(0.95));
  REQUIRE(record.notes == "detected by YOLO");
  REQUIRE(record.metadata.find("instances-guid-7") != std::string::npos);

  auto restored = from_component_record(record);
  REQUIRE(restored.name == original.name);
  REQUIRE(restored.guid == original.guid);
  REQUIRE(restored.type == original.type);
  REQUIRE(restored.source_instance_guid == original.source_instance_guid);
  REQUIRE(restored.parent_id == original.parent_id);
  REQUIRE(restored.confidence == Approx(original.confidence));
  REQUIRE(restored.notes == original.notes);
  REQUIRE(restored.boundary.vertices.size() ==
          original.boundary.vertices.size());
  for (size_t i = 0; i < original.boundary.vertices.size(); ++i)
    REQUIRE(
        restored.boundary.vertices[i].isApprox(original.boundary.vertices[i]));
  for (int i = 0; i < 4; ++i)
    REQUIRE(restored.boundary.plane[i] == Approx(original.boundary.plane[i]));
  REQUIRE(std::get<WindowData>(restored.data).pane_count == 2);

  // Mapping is idempotent, so the persisted bytes are stable.
  auto record2 = to_component_record(restored);
  REQUIRE(record2.type == record.type);
  REQUIRE(record2.vertex_data == record.vertex_data);
  REQUIRE(record2.plane == record.plane);
  REQUIRE(record2.metadata == record.metadata);
  REQUIRE(record2.notes == record.notes);
}

TEST_CASE("ProjectDB stores and loads a ComponentRecord verbatim",
          "[projectdb][components][record]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  auto record = to_component_record(make_door("door1"));
  record.guid = "record-guid";
  db.save_component_record(record);

  auto loaded = db.component_record("door1");
  REQUIRE(loaded.name == record.name);
  REQUIRE(loaded.guid == "record-guid");
  REQUIRE(loaded.type == "door");
  REQUIRE(loaded.vertex_data == record.vertex_data);
  REQUIRE(loaded.plane == record.plane);
  REQUIRE(loaded.parent_id == record.parent_id);
  REQUIRE(loaded.confidence == Approx(record.confidence));
  REQUIRE(loaded.metadata == record.metadata);
  REQUIRE(loaded.notes == record.notes);

  REQUIRE(db.list_building_components("door") ==
          std::vector<std::string>{"door1"});
  REQUIRE(db.list_building_components("window").empty());
}
