// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "database/component_router.hpp"

#include <reusex/core/component_record.hpp>

#include <algorithm>
#include <spdlog/spdlog.h>

namespace rux::database {

namespace {

nlohmann::json record_to_json(const reusex::core::ComponentRecord &r) {
  nlohmann::json j;
  j["name"] = r.name;
  j["guid"] = r.guid;
  j["type"] = r.type;
  j["parent_id"] = r.parent_id;
  j["confidence"] = r.confidence;
  j["notes"] =
      r.notes.empty() ? nlohmann::json(nullptr) : nlohmann::json(r.notes);

  // vertex_count derived from the packed float64-xyz blob
  const std::size_t vertex_count = r.vertex_data.size() / (3 * sizeof(double));
  j["vertex_count"] = vertex_count;

  // plane as [a, b, c, d]
  j["plane"] =
      nlohmann::json::array({r.plane[0], r.plane[1], r.plane[2], r.plane[3]});

  // metadata: store as parsed JSON if possible, raw string otherwise
  if (r.metadata.empty()) {
    j["metadata"] = nlohmann::json(nullptr);
  } else {
    auto parsed = nlohmann::json::parse(r.metadata, nullptr, false);
    j["metadata"] = parsed.is_discarded() ? nlohmann::json(r.metadata) : parsed;
  }

  return j;
}

} // anonymous namespace

std::vector<std::string> ComponentRouter::list() const {
  auto names = db_->list_building_components();
  std::sort(names.begin(), names.end());
  return names;
}

DataPayload ComponentRouter::get(const std::vector<PathComponent> &components) {
  if (components.empty()) {
    return nlohmann::json(list());
  }

  // Resolve item name from name or numeric index
  std::string item_name;
  if (components[0].is_index()) {
    auto resolved = resolve_index(*components[0].index);
    if (!resolved) {
      throw std::runtime_error("Array index out of range: " +
                               std::to_string(*components[0].index));
    }
    item_name = *resolved;
  } else if (components[0].is_item()) {
    item_name = components[0].value;
  } else {
    throw std::runtime_error("Expected item name or index after 'components'");
  }

  if (!db_->has_building_component(item_name)) {
    throw std::runtime_error("Component not found: " + item_name);
  }

  auto record = db_->component_record(item_name);

  if (components.size() == 1) {
    return record_to_json(record);
  }

  // Property access
  const auto &prop = components[1].value;

  if (prop == "name")
    return record.name;
  if (prop == "guid")
    return record.guid;
  if (prop == "type")
    return record.type;
  if (prop == "parent_id")
    return std::to_string(record.parent_id);
  if (prop == "confidence")
    return std::to_string(record.confidence);
  if (prop == "notes")
    return record.notes;
  if (prop == "vertex_count") {
    return std::to_string(record.vertex_data.size() / (3 * sizeof(double)));
  }
  if (prop == "plane") {
    return nlohmann::json::array(
        {record.plane[0], record.plane[1], record.plane[2], record.plane[3]});
  }
  if (prop == "metadata") {
    if (record.metadata.empty())
      return nlohmann::json(nullptr);
    auto parsed = nlohmann::json::parse(record.metadata, nullptr, false);
    return parsed.is_discarded() ? DataPayload{record.metadata}
                                 : DataPayload{parsed};
  }

  throw std::runtime_error(
      "Unknown property: " + prop +
      "\nAvailable properties: name, guid, type, parent_id, confidence, "
      "notes, metadata, vertex_count, plane");
}

void ComponentRouter::set(const std::vector<PathComponent> & /*components*/,
                          const DataPayload & /*data*/) {
  throw std::runtime_error(
      "Building components are created by the pipeline ('rux create ...'). "
      "Use 'rux get components' to read them.");
}

void ComponentRouter::del(const std::vector<PathComponent> & /*components*/) {
  throw std::runtime_error(
      "Building component deletion is not supported via 'rux del'. "
      "Components are managed by the reconstruction pipeline.");
}

} // namespace rux::database
