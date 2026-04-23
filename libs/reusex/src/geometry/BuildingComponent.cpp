// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "geometry/BuildingComponent.hpp"

#include <nlohmann/json.hpp>

#include <stdexcept>

namespace reusex::geometry {

std::string_view to_string(ComponentType type) {
  switch (type) {
  case ComponentType::window:
    return "window";
  case ComponentType::door:
    return "door";
  case ComponentType::wall:
    return "wall";
  }
  return "window"; // unreachable, silences warning
}

ComponentType component_type_from_string(std::string_view str) {
  if (str == "window")
    return ComponentType::window;
  if (str == "door")
    return ComponentType::door;
  if (str == "wall")
    return ComponentType::wall;
  throw std::runtime_error("Unknown component type: " + std::string(str));
}

std::string component_data_to_json(const BuildingComponent &c) {
  nlohmann::json j;
  j["component_type"] = to_string(c.type);

  std::visit(
      [&](const auto &d) {
        using T = std::decay_t<decltype(d)>;
        if constexpr (std::is_same_v<T, WindowData>) {
          j["style"] = d.style;
          j["pane_count"] = d.pane_count;
          j["operable"] = d.operable;
        } else if constexpr (std::is_same_v<T, DoorData>) {
          j["style"] = d.style;
          j["swing"] = d.swing;
        } else if constexpr (std::is_same_v<T, WallData>) {
          // empty for now
        }
      },
      c.data);

  return j.dump();
}

void component_data_from_json(BuildingComponent &c, const std::string &json) {
  if (json.empty())
    return;

  auto j = nlohmann::json::parse(json);

  // Determine type from JSON discriminator
  if (j.contains("component_type")) {
    c.type = component_type_from_string(j["component_type"].get<std::string>());
  }

  switch (c.type) {
  case ComponentType::window: {
    WindowData d;
    if (j.contains("style"))
      d.style = j["style"].get<std::string>();
    if (j.contains("pane_count"))
      d.pane_count = j["pane_count"].get<int>();
    if (j.contains("operable"))
      d.operable = j["operable"].get<bool>();
    c.data = d;
    break;
  }
  case ComponentType::door: {
    DoorData d;
    if (j.contains("style"))
      d.style = j["style"].get<std::string>();
    if (j.contains("swing"))
      d.swing = j["swing"].get<std::string>();
    c.data = d;
    break;
  }
  case ComponentType::wall: {
    c.data = WallData{};
    break;
  }
  }
}

} // namespace reusex::geometry
