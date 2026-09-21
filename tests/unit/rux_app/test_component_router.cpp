// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Contract tests for the `components` router (#354).
//
// ComponentRouter is read-only: set/del throw. get() supports listing
// (empty components), item-by-name, item-by-index, and property drill-down.
// Tests run against a real (in-process) ProjectDB via the heavy binary
// (tests/unit/rux_app → reusex_unit_tests_vision, links rux_lib).

#include <catch2/catch_test_macros.hpp>

#include <database/component_router.hpp>
#include <database/path_parser.hpp>
#include <database/resource_router.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/component_record.hpp>

#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>

#include <cstring>
#include <memory>
#include <string>
#include <vector>

using namespace rux::database;
using json = nlohmann::json;

namespace {

/// Pack three float64 xyz vertices into a vertex_data blob.
std::vector<uint8_t>
make_vertex_blob(std::initializer_list<std::array<double, 3>> verts) {
  std::vector<uint8_t> blob;
  blob.reserve(verts.size() * 3 * sizeof(double));
  for (const auto &v : verts) {
    for (double coord : v) {
      uint8_t bytes[sizeof(double)];
      std::memcpy(bytes, &coord, sizeof(double));
      blob.insert(blob.end(), bytes, bytes + sizeof(double));
    }
  }
  return blob;
}

reusex::core::ComponentRecord make_window(const std::string &name) {
  reusex::core::ComponentRecord r;
  r.name = name;
  r.guid = "guid-" + name;
  r.type = "window";
  r.vertex_data = make_vertex_blob(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 0.0, 1.2}, {0.0, 0.0, 1.2}});
  r.plane = {0.0, 1.0, 0.0, 0.0};
  r.parent_id = -1;
  r.confidence = 0.9;
  r.notes = "test window";
  r.metadata = R"({"style":"casement"})";
  return r;
}

reusex::core::ComponentRecord make_door(const std::string &name) {
  reusex::core::ComponentRecord r;
  r.name = name;
  r.guid = "guid-" + name;
  r.type = "door";
  r.vertex_data =
      make_vertex_blob({{0.0, 0.0, 0.0}, {0.9, 0.0, 0.0}, {0.9, 0.0, 2.1}});
  r.plane = {0.0, 1.0, 0.0, 0.5};
  r.parent_id = -1;
  r.confidence = -1.0; // manual
  r.notes = "";
  r.metadata = "";
  return r;
}

/// Fixture: a temp ProjectDB with a window and a door component.
struct Fixture {
  reusex::test_support::TempPath tmp{"test_component_router"};
  std::shared_ptr<reusex::ProjectDB> db;
  ComponentRouter router;

  Fixture() : db(std::make_shared<reusex::ProjectDB>(tmp.path)), router(db) {
    db->save_component_record(make_window("win1"));
    db->save_component_record(make_door("door1"));
  }
};

} // namespace

// ===========================================================================
// list()
// ===========================================================================

TEST_CASE("ComponentRouter_List_ReturnsSortedNames",
          "[rux][router][components]") {
  Fixture f;
  auto names = f.router.list();
  REQUIRE(names.size() == 2);
  // Sorted alphabetically: door1 < win1
  CHECK(names[0] == "door1");
  CHECK(names[1] == "win1");
}

// ===========================================================================
// get — collection level
// ===========================================================================

TEST_CASE("ComponentRouter_GetEmpty_ReturnsNameArray",
          "[rux][router][components]") {
  Fixture f;
  auto payload = f.router.get({});
  REQUIRE(std::holds_alternative<json>(payload));
  auto arr = std::get<json>(payload);
  REQUIRE(arr.is_array());
  REQUIRE(arr.size() == 2);
  CHECK(arr[0].get<std::string>() == "door1");
  CHECK(arr[1].get<std::string>() == "win1");
}

// ===========================================================================
// get — item level (by name)
// ===========================================================================

TEST_CASE("ComponentRouter_GetByName_ReturnsFields",
          "[rux][router][components]") {
  Fixture f;
  auto payload = f.router.get({PathComponent(ComponentType::Item, "win1")});
  REQUIRE(std::holds_alternative<json>(payload));
  auto j = std::get<json>(payload);
  CHECK(j["name"].get<std::string>() == "win1");
  CHECK(j["guid"].get<std::string>() == "guid-win1");
  CHECK(j["type"].get<std::string>() == "window");
  CHECK(j["parent_id"].get<int>() == -1);
  CHECK(j.contains("confidence"));
  CHECK(j.contains("vertex_count"));
  CHECK(j["vertex_count"].get<std::size_t>() == 4); // 4 vertices stored
  CHECK(j.contains("plane"));
  CHECK(j["plane"].is_array());
  CHECK(j["plane"].size() == 4);
  // metadata stored as JSON string must come back as a parsed object
  CHECK(j["metadata"].is_object());
  CHECK(j["metadata"]["style"].get<std::string>() == "casement");
}

TEST_CASE("ComponentRouter_GetByName_EmptyMetadataIsNull",
          "[rux][router][components]") {
  Fixture f;
  auto payload = f.router.get({PathComponent(ComponentType::Item, "door1")});
  REQUIRE(std::holds_alternative<json>(payload));
  auto j = std::get<json>(payload);
  CHECK(j["metadata"].is_null());
  CHECK(j["notes"].is_null());
  CHECK(j["vertex_count"].get<std::size_t>() == 3);
}

// ===========================================================================
// get — item level (by index)
// ===========================================================================

TEST_CASE("ComponentRouter_GetByIndex_ResolvesCorrectly",
          "[rux][router][components]") {
  Fixture f;
  // Index 0 → door1 (sorted first)
  auto payload = f.router.get({PathComponent(ComponentType::Index, "", 0)});
  REQUIRE(std::holds_alternative<json>(payload));
  auto j = std::get<json>(payload);
  CHECK(j["name"].get<std::string>() == "door1");
}

TEST_CASE("ComponentRouter_GetByIndexOutOfRange_Throws",
          "[rux][router][components]") {
  Fixture f;
  CHECK_THROWS_AS(f.router.get({PathComponent(ComponentType::Index, "", 99)}),
                  std::runtime_error);
}

// ===========================================================================
// get — property level
// ===========================================================================

TEST_CASE("ComponentRouter_GetProperty_ReturnsScalar",
          "[rux][router][components]") {
  Fixture f;
  auto payload = f.router.get({PathComponent(ComponentType::Item, "win1"),
                               PathComponent(ComponentType::Property, "type")});
  REQUIRE(std::holds_alternative<std::string>(payload));
  CHECK(std::get<std::string>(payload) == "window");
}

TEST_CASE("ComponentRouter_GetVertexCount_ReturnsString",
          "[rux][router][components]") {
  Fixture f;
  auto payload =
      f.router.get({PathComponent(ComponentType::Item, "win1"),
                    PathComponent(ComponentType::Property, "vertex_count")});
  REQUIRE(std::holds_alternative<std::string>(payload));
  CHECK(std::get<std::string>(payload) == "4");
}

TEST_CASE("ComponentRouter_GetUnknownProperty_Throws",
          "[rux][router][components]") {
  Fixture f;
  CHECK_THROWS_AS(
      f.router.get({PathComponent(ComponentType::Item, "win1"),
                    PathComponent(ComponentType::Property, "does_not_exist")}),
      std::runtime_error);
}

// ===========================================================================
// get — error cases
// ===========================================================================

TEST_CASE("ComponentRouter_GetUnknownName_Throws",
          "[rux][router][components]") {
  Fixture f;
  CHECK_THROWS_AS(
      f.router.get({PathComponent(ComponentType::Item, "nonexistent")}),
      std::runtime_error);
}

// ===========================================================================
// set / del — read-only guard
// ===========================================================================

TEST_CASE("ComponentRouter_Set_Throws", "[rux][router][components]") {
  Fixture f;
  DataPayload dummy{std::string("value")};
  CHECK_THROWS_AS(
      f.router.set({PathComponent(ComponentType::Item, "win1")}, dummy),
      std::runtime_error);
}

TEST_CASE("ComponentRouter_Del_Throws", "[rux][router][components]") {
  Fixture f;
  CHECK_THROWS_AS(f.router.del({PathComponent(ComponentType::Item, "win1")}),
                  std::runtime_error);
}

// ===========================================================================
// RouterRegistry — components collection is registered
// ===========================================================================

TEST_CASE("RouterRegistry_ComponentsCollectionIsRegistered",
          "[rux][router][components]") {
  reusex::test_support::TempPath tmp{"test_component_router_registry"};
  auto db = std::make_shared<reusex::ProjectDB>(tmp.path);
  RouterRegistry registry(db);

  auto names = registry.collection_names();
  CHECK(std::find(names.begin(), names.end(), "components") != names.end());
  // "passports" must not appear — the collection is named "materials"
  CHECK(std::find(names.begin(), names.end(), "passports") == names.end());

  // The router must be reachable without throwing
  REQUIRE_NOTHROW(registry.get_router("components"));
}
