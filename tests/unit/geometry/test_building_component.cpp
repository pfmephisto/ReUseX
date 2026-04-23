// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>

#include <geometry/BuildingComponent.hpp>

using namespace reusex::geometry;

TEST_CASE("ComponentType to_string and from_string round-trip",
          "[geometry][building_component]") {
  REQUIRE(to_string(ComponentType::window) == "window");
  REQUIRE(to_string(ComponentType::door) == "door");
  REQUIRE(to_string(ComponentType::wall) == "wall");

  REQUIRE(component_type_from_string("window") == ComponentType::window);
  REQUIRE(component_type_from_string("door") == ComponentType::door);
  REQUIRE(component_type_from_string("wall") == ComponentType::wall);
}

TEST_CASE("ComponentType from_string throws on unknown type",
          "[geometry][building_component]") {
  REQUIRE_THROWS_AS(component_type_from_string("beam"), std::runtime_error);
}

TEST_CASE("WindowData JSON round-trip", "[geometry][building_component]") {
  BuildingComponent c;
  c.name = "win1";
  c.type = ComponentType::window;
  c.data = WindowData{"casement", 2, true};

  std::string json = component_data_to_json(c);
  REQUIRE_FALSE(json.empty());

  BuildingComponent c2;
  c2.type = ComponentType::window;
  component_data_from_json(c2, json);

  auto &wd = std::get<WindowData>(c2.data);
  REQUIRE(wd.style == "casement");
  REQUIRE(wd.pane_count == 2);
  REQUIRE(wd.operable == true);
}

TEST_CASE("DoorData JSON round-trip", "[geometry][building_component]") {
  BuildingComponent c;
  c.name = "door1";
  c.type = ComponentType::door;
  c.data = DoorData{"double", "left"};

  std::string json = component_data_to_json(c);

  BuildingComponent c2;
  c2.type = ComponentType::door;
  component_data_from_json(c2, json);

  auto &dd = std::get<DoorData>(c2.data);
  REQUIRE(dd.style == "double");
  REQUIRE(dd.swing == "left");
}

TEST_CASE("WallData JSON round-trip", "[geometry][building_component]") {
  BuildingComponent c;
  c.name = "wall1";
  c.type = ComponentType::wall;
  c.data = WallData{};

  std::string json = component_data_to_json(c);

  BuildingComponent c2;
  c2.type = ComponentType::wall;
  component_data_from_json(c2, json);

  REQUIRE(std::holds_alternative<WallData>(c2.data));
}

TEST_CASE("JSON discriminator determines type",
          "[geometry][building_component]") {
  // Serialize a door, then deserialize into a fresh component
  BuildingComponent c;
  c.type = ComponentType::door;
  c.data = DoorData{"sliding", "none"};
  std::string json = component_data_to_json(c);

  BuildingComponent c2;
  c2.type = ComponentType::window; // wrong type initially
  component_data_from_json(c2, json);

  // The JSON discriminator should override
  REQUIRE(c2.type == ComponentType::door);
  REQUIRE(std::holds_alternative<DoorData>(c2.data));
}

TEST_CASE("component_data_from_json with empty string is no-op",
          "[geometry][building_component]") {
  BuildingComponent c;
  c.type = ComponentType::window;
  c.data = WindowData{"fixed", 1, false};
  component_data_from_json(c, "");

  // Should be unchanged
  auto &wd = std::get<WindowData>(c.data);
  REQUIRE(wd.style == "fixed");
  REQUIRE(wd.pane_count == 1);
}
