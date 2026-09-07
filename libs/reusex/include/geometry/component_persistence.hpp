// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// Mapping between `geometry::BuildingComponent` and the core-owned
/// persistence POD `core::ComponentRecord`, plus thin `ProjectDB`
/// conveniences (#227).
///
/// `ProjectDB` stores only the POD, so `core` no longer knows about the
/// geometry types. The knowledge of *how* a `BuildingComponent` flattens into
/// a row therefore lives here, on the geometry side of the boundary.
///
/// Layering note: this is a header-only adapter. It is deliberately NOT
/// compiled into `reusex_geometry_common` (there is no matching .cpp), so that
/// module keeps its Layer-1½ position and never links `core`. Consumers that
/// include it — `io`, `apps/rux`, tests — already link both `reusex_core` and
/// `reusex_geometry_common`. See docs/STANDARDS.md §1.

#include "reusex/core/ProjectDB.hpp"
#include "reusex/core/component_record.hpp"
#include "reusex/geometry/BuildingComponent.hpp"
#include "reusex/geometry/CoplanarPolygon.hpp"

#include <string>
#include <string_view>
#include <vector>

namespace reusex::geometry {

/// Flatten a component into the persistence POD.
///
/// The variant payload and the `source_instance_guid` provenance link are
/// folded into the record's opaque `metadata` JSON, exactly as the previous
/// in-`ProjectDB` code did, so the on-disk bytes are unchanged.
inline core::ComponentRecord
to_component_record(const BuildingComponent &component) {
  core::ComponentRecord record;
  record.name = component.name;
  record.guid = component.guid;
  record.type = std::string(to_string(component.type));
  record.vertex_data = component.boundary.serialize_vertices();
  for (int i = 0; i < 4; ++i)
    record.plane[static_cast<size_t>(i)] = component.boundary.plane[i];
  record.parent_id = component.parent_id;
  record.confidence = component.confidence;
  record.metadata = component_data_to_json(component);
  record.notes = component.notes;
  return record;
}

/// Rebuild a component from the persistence POD.
inline BuildingComponent
from_component_record(const core::ComponentRecord &record) {
  BuildingComponent component;
  component.name = record.name;
  component.guid = record.guid;
  if (!record.type.empty())
    component.type = component_type_from_string(record.type);
  if (!record.vertex_data.empty())
    component.boundary.vertices = CoplanarPolygon::deserialize_vertices(
        record.vertex_data.data(), record.vertex_data.size());
  for (int i = 0; i < 4; ++i)
    component.boundary.plane[i] = record.plane[static_cast<size_t>(i)];
  component.parent_id = record.parent_id;
  component.confidence = record.confidence;
  component.notes = record.notes;
  // Overwrites `type` from the JSON discriminator when present, and populates
  // the variant payload plus `source_instance_guid`.
  component_data_from_json(component, record.metadata);
  return component;
}

/// Insert or replace `component`, keyed by its name. A guid is generated on
/// first save and preserved across later saves.
inline void save_building_component(ProjectDB &db,
                                    const BuildingComponent &component) {
  db.save_component_record(to_component_record(component));
}

/// Update an existing component's mutable fields, matched by its immutable
/// guid. Throws if no component has that guid.
inline void
update_building_component_by_guid(ProjectDB &db,
                                  const BuildingComponent &component) {
  db.update_component_record_by_guid(to_component_record(component));
}

/// Load the component stored under `name`. Throws if absent.
inline BuildingComponent building_component(const ProjectDB &db,
                                            std::string_view name) {
  return from_component_record(db.component_record(name));
}

/// Names of all stored components of the given type.
inline std::vector<std::string> list_building_components(const ProjectDB &db,
                                                         ComponentType type) {
  return db.list_building_components(to_string(type));
}

} // namespace reusex::geometry
