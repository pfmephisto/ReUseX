// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <geometry/BuildingComponent.hpp>

#include <cstdio>
#include <filesystem>
#include <iostream>

using namespace reusex;
using namespace reusex::geometry;
using Catch::Approx;

namespace fs = std::filesystem;

// Helper: create a temp database path that auto-cleans
struct TempDB {
  fs::path path;
  TempDB()
      : path(fs::temp_directory_path() /
             ("test_projectdb_comp_" +
              std::to_string(reinterpret_cast<uintptr_t>(this)) + ".rux")) {}
  ~TempDB() noexcept {
    std::error_code ec;
    fs::remove(path, ec);
  }
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
  REQUIRE(db.schema_version() == 10);
}

TEST_CASE("ProjectDB building component save/load round-trip",
          "[projectdb][components]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  auto original = make_window("win1");
  db.save_building_component(original);

  REQUIRE(db.has_building_component("win1"));

  auto loaded = db.building_component("win1");
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
  db.save_building_component(c1);

  // Overwrite with door
  auto c2 = make_door("win1");
  db.save_building_component(c2);

  auto loaded = db.building_component("win1");
  REQUIRE(loaded.type == ComponentType::door);
  REQUIRE(std::holds_alternative<DoorData>(loaded.data));
}

TEST_CASE("ProjectDB has_building_component", "[projectdb][components]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  REQUIRE_FALSE(db.has_building_component("nonexistent"));

  db.save_building_component(make_window("win1"));
  REQUIRE(db.has_building_component("win1"));
}

TEST_CASE("ProjectDB delete_building_component", "[projectdb][components]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  db.save_building_component(make_window("win1"));
  REQUIRE(db.has_building_component("win1"));

  db.delete_building_component("win1");
  REQUIRE_FALSE(db.has_building_component("win1"));
}

TEST_CASE("ProjectDB list_building_components", "[projectdb][components]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  REQUIRE(db.list_building_components().empty());

  db.save_building_component(make_window("win1"));
  db.save_building_component(make_window("win2"));
  db.save_building_component(make_door("door1"));

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

  db.save_building_component(make_window("win1"));
  db.save_building_component(make_window("win2"));
  db.save_building_component(make_door("door1"));

  auto windows = db.list_building_components(ComponentType::window);
  REQUIRE(windows.size() == 2);
  REQUIRE(windows[0] == "win1");
  REQUIRE(windows[1] == "win2");

  auto doors = db.list_building_components(ComponentType::door);
  REQUIRE(doors.size() == 1);
  REQUIRE(doors[0] == "door1");

  auto walls = db.list_building_components(ComponentType::wall);
  REQUIRE(walls.empty());
}

TEST_CASE("ProjectDB building_component_count", "[projectdb][components]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  REQUIRE(db.building_component_count() == 0);

  db.save_building_component(make_window("win1"));
  db.save_building_component(make_door("door1"));
  REQUIRE(db.building_component_count() == 2);
}

TEST_CASE("ProjectDB building_component throws for missing",
          "[projectdb][components]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  REQUIRE_THROWS_AS(db.building_component("nonexistent"), std::runtime_error);
}

TEST_CASE("ProjectDB project_summary includes component info",
          "[projectdb][components]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  db.save_building_component(make_window("win1"));
  db.save_building_component(make_window("win2"));
  db.save_building_component(make_door("door1"));

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
  db.save_building_component(original);

  auto loaded = db.building_component("door1");
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
  db.save_building_component(c);

  auto loaded = db.building_component("win1");
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
  db.save_building_component(c);

  REQUIRE(db.building_component("win1").guid == "my-fixed-guid");
}

TEST_CASE("ProjectDB guid is stable across upsert",
          "[projectdb][components][guid]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  db.save_building_component(make_window("win1"));
  std::string g1 = db.building_component("win1").guid;

  // Re-save under the same name with different fields.
  auto c2 = make_window("win1");
  c2.confidence = 0.10;
  db.save_building_component(c2);

  std::string g2 = db.building_component("win1").guid;
  REQUIRE(g1 == g2);
}

TEST_CASE("ProjectDB update_building_component_by_guid renames and edits",
          "[projectdb][components][guid]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  db.save_building_component(make_window("win1"));
  auto c = db.building_component("win1");
  std::string guid = c.guid;

  // Edit mutable fields, including a rename.
  c.name = "renamed_window";
  c.notes = "edited via csv";
  c.confidence = 0.42;
  c.parent_id = 7;
  std::get<WindowData>(c.data).pane_count = 4;
  db.update_building_component_by_guid(c);

  REQUIRE_FALSE(db.has_building_component("win1")); // old name gone
  REQUIRE(db.has_building_component("renamed_window"));

  auto loaded = db.building_component("renamed_window");
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
  REQUIRE_THROWS_AS(db.update_building_component_by_guid(c),
                    std::runtime_error);
}

TEST_CASE("ProjectDB persists source_instance_guid provenance (issue #211)",
          "[projectdb][components][provenance]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  // Save with provenance set (as create_windows would).
  auto c = make_window("win1");
  c.source_instance_guid = "instances-guid-42";
  db.save_building_component(c);

  auto loaded = db.building_component("win1");
  REQUIRE(loaded.source_instance_guid == "instances-guid-42");

  // The guid-update path (used by CSV import) must also round-trip it.
  loaded.source_instance_guid = "instances-guid-99";
  db.update_building_component_by_guid(loaded);
  REQUIRE(db.building_component("win1").source_instance_guid ==
          "instances-guid-99");
}

TEST_CASE("ProjectDB leaves source_instance_guid empty when unset",
          "[projectdb][components][provenance]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  db.save_building_component(make_window("win1")); // no provenance
  REQUIRE(db.building_component("win1").source_instance_guid.empty());
}
