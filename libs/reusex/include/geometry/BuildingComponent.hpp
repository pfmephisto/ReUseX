// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "reusex/geometry/CoplanarPolygon.hpp"

#include <fmt/format.h>
#include <string>
#include <string_view>
#include <variant>

namespace reusex::geometry {

/// Discriminator for building component types.
enum class ComponentType { window, door, wall };

std::string_view to_string(ComponentType type);
ComponentType component_type_from_string(std::string_view str);

struct WindowData {
  std::string style; // "casement", "sliding", "fixed", etc.
  int pane_count = 0;
  bool operable = true;
};

struct DoorData {
  std::string style; // "single", "double", "sliding"
  std::string swing; // "left", "right", "none"
};

struct WallData {}; // stub for future use

/// A detected or manual building component (window, door, wall, ...).
struct BuildingComponent {
  std::string name;
  ComponentType type = ComponentType::window;
  CoplanarPolygon boundary;
  int parent_id = -1;       // optional link to parent component
  double confidence = -1.0; // detection confidence, -1 if manual
  std::string notes;
  std::variant<WindowData, DoorData, WallData> data = WindowData{};
};

/// Serialize the type-specific variant data to JSON TEXT.
std::string component_data_to_json(const BuildingComponent &c);

/// Deserialize JSON TEXT into the variant data on a BuildingComponent.
void component_data_from_json(BuildingComponent &c, const std::string &json);

} // namespace reusex::geometry

template <>
struct fmt::formatter<reusex::geometry::BuildingComponent>
    : fmt::formatter<std::string_view> {
  auto format(reusex::geometry::BuildingComponent value,
              format_context &ctx) const -> format_context::iterator {
    return fmt::format_to(ctx.out(), "{} ({} vertices)", value.name,
                          value.boundary.vertices.size());
  }
};
