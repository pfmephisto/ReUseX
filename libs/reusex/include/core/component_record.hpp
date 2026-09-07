// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace reusex::core {

/// Core-owned persistence contract for a row of the `building_components`
/// table (#227).
///
/// `ProjectDB` (Layer 2) stores and loads only this POD, so persistence needs
/// no knowledge of `geometry::BuildingComponent` / `geometry::CoplanarPolygon`
/// (Layer 1½ geometry types). The mapping in both directions lives in the
/// geometry layer — see `geometry/component_persistence.hpp`.
///
/// Every member corresponds 1:1 to a column of `building_components`; the
/// on-disk representation is unchanged from before the POD was introduced.
/// `vertex_count` is not a member: it is derived from `vertex_data`.
struct ComponentRecord {
  /// `name` column — unique; the UPSERT conflict target.
  std::string name;
  /// `guid` column (schema v8) — stable, immutable identity. Empty means
  /// "generate one on first save"; preserved across upserts.
  std::string guid;
  /// `type` column — component-type discriminator as stored, e.g. "window".
  std::string type;
  /// `vertex_data` blob — boundary vertices as packed little-endian float64
  /// xyz triples (3 * 8 bytes per vertex). The `vertex_count` column is
  /// written as `vertex_data.size() / (3 * sizeof(double))`.
  std::vector<uint8_t> vertex_data;
  /// `plane` blob — Hessian normal form [a,b,c,d] of ax+by+cz+d=0,
  /// always persisted as 4 * 8 bytes.
  std::array<double, 4> plane{0.0, 0.0, 0.0, 0.0};
  /// `parent_id` column — optional link to a parent component, -1 if none.
  int parent_id = -1;
  /// `confidence` column — detection confidence, -1 if manual.
  double confidence = -1.0;
  /// `metadata` column — opaque JSON TEXT owned by the geometry layer. Carries
  /// the type-specific variant payload and the `source_instance_guid`
  /// provenance link (issue #211). Bound as NULL when empty.
  std::string metadata;
  /// `notes` column — free-form text. Bound as NULL when empty.
  std::string notes;
};

} // namespace reusex::core
