// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "database/passport_router.hpp"

#include "database/format_handler.hpp"

#include <reusex/core/MaterialPassport.hpp>
#include <reusex/core/materialepas_json_export.hpp>
#include <reusex/core/materialepas_json_import.hpp>

#include <algorithm>
#include <spdlog/spdlog.h>

namespace rux::database {

namespace {

/// Collapse a stored value string for summary display.
/// JSON objects → "{...}", JSON arrays → "[...]", scalars as-is.
std::string collapse_value(const std::string &value) {
  if (value.empty())
    return value;
  // Quick heuristic: check first non-whitespace character
  auto first = value.front();
  if (first == '{') return "{...}";
  if (first == '[') return "[...]";
  return value;
}

/// Resolve GUID from first path component (index or direct access)
std::string resolve_guid(const PathComponent &comp,
                         const ResourceRouter &router) {
  if (comp.is_index()) {
    auto resolved = router.resolve_index(*comp.index);
    if (!resolved) {
      throw std::runtime_error("Array index out of range: " +
                               std::to_string(*comp.index));
    }
    return *resolved;
  }
  if (comp.is_item()) {
    return comp.value;
  }
  throw std::runtime_error("Expected item or index after collection");
}

} // anonymous namespace

std::vector<std::string> PassportRouter::list() const {
  return db_->list_passport_guids();
}

DataPayload PassportRouter::get(const std::vector<PathComponent> &components) {
  if (components.empty()) {
    // Collection level: return list of passport GUIDs
    return nlohmann::json(list());
  }

  auto guid = resolve_guid(components[0], *this);

  if (guid.find('*') != std::string::npos) {
    throw std::runtime_error("Wildcard not expanded: " + guid);
  }

  // Item level: return metadata + stored property name→value pairs
  if (components.size() == 1) {
    auto metadata = db_->passport_metadata(guid);
    auto properties = db_->passport_stored_properties(guid);

    nlohmann::json result;
    result["document_guid"] = metadata.document_guid;
    result["created_at"] = metadata.creation_date;
    result["version_number"] = metadata.version_number;

    nlohmann::json props = nlohmann::json::object();
    for (const auto &[name, value] : properties) {
      props[name] = collapse_value(value);
    }
    result["properties"] = std::move(props);

    return result;
  }

  // Property level: dispatch by field name
  const auto &field_name = components[1].value;

  // Metadata fields
  if (field_name == "document_guid" || field_name == "created_at" ||
      field_name == "version_number") {
    auto metadata = db_->passport_metadata(guid);
    if (field_name == "document_guid") return std::string(metadata.document_guid);
    if (field_name == "created_at")    return std::string(metadata.creation_date);
    return std::string(metadata.version_number);
  }

  // "properties" → return full stored properties map (collapsed or drilled)
  if (field_name == "properties") {
    auto properties = db_->passport_stored_properties(guid);

    // Drill into a specific property: materials[0].properties.contact_email
    if (components.size() >= 3) {
      const auto &prop_name = components[2].value;
      auto it = properties.find(prop_name);
      if (it == properties.end()) {
        throw std::runtime_error("Property '" + prop_name +
                                 "' not found for passport '" + guid + "'");
      }
      return it->second;
    }

    // Return collapsed summary
    nlohmann::json props = nlohmann::json::object();
    for (const auto &[name, value] : properties) {
      props[name] = collapse_value(value);
    }
    return props;
  }

  // Direct property access: materials[0].contact_email
  try {
    auto value = db_->passport_property_value(guid, field_name);
    return value;
  } catch (const std::exception &e) {
    throw std::runtime_error("Failed to get property '" + field_name +
                             "' for passport '" + guid + "': " + e.what());
  }
}

void PassportRouter::set(const std::vector<PathComponent> &components,
                         const DataPayload &data) {
  if (components.empty()) {
    throw std::runtime_error(
        "Cannot set collection directly. Specify a passport GUID: "
        "materials.<guid>");
  }

  auto guid = resolve_guid(components[0], *this);

  // Extract string value from payload
  std::string value;
  if (std::holds_alternative<std::string>(data)) {
    value = std::get<std::string>(data);
  } else if (std::holds_alternative<nlohmann::json>(data)) {
    auto j = std::get<nlohmann::json>(data);
    if (j.is_string()) {
      value = j.get<std::string>();
    } else if (j.is_number()) {
      value = std::to_string(j.get<int>());
    } else {
      value = j.dump();
    }
  } else {
    throw std::runtime_error("Invalid data type for passport property");
  }

  // Case 1: Setting entire passport via JSON import (no property specified)
  if (components.size() == 1) {
    nlohmann::json json_data;
    try {
      json_data = nlohmann::json::parse(value);
    } catch (const std::exception &e) {
      throw std::runtime_error("Failed to parse JSON: " +
                               std::string(e.what()));
    }

    auto passport = reusex::core::json_import::from_json(json_data);
    passport.metadata.document_guid = guid;
    db_->add_material_passport(passport, "");
    spdlog::info("Saved material passport '{}'", guid);
    return;
  }

  // Case 2: Setting individual field
  const auto &field_name = components[1].value;

  // Metadata fields → update material_passports columns directly
  if (field_name == "created_at" || field_name == "version_number" ||
      field_name == "revised_at" || field_name == "version_date") {
    try {
      db_->set_passport_metadata_field(guid, field_name, value);
      spdlog::info("Updated passport '{}' metadata '{}'", guid, field_name);
    } catch (const std::exception &e) {
      throw std::runtime_error("Failed to set metadata '" + field_name +
                               "' for passport '" + guid + "': " + e.what());
    }
    return;
  }

  if (field_name == "document_guid") {
    throw std::runtime_error(
        "Cannot modify document_guid (it's the primary key). "
        "Create a new passport with a different GUID instead.");
  }

  // "properties.X" → resolve the actual property name
  std::string prop_name = field_name;
  if (field_name == "properties" && components.size() >= 3) {
    prop_name = components[2].value;
  } else if (field_name == "properties") {
    throw std::runtime_error(
        "Cannot set 'properties' directly. Specify a property name: "
        "materials[N].properties.<name> or materials[N].<name>");
  }

  // Property value upsert
  try {
    db_->set_passport_property(guid, prop_name, value);
    spdlog::info("Updated passport '{}' property '{}'", guid, prop_name);
  } catch (const std::exception &e) {
    throw std::runtime_error("Failed to set property '" + prop_name +
                             "' for passport '" + guid + "': " + e.what());
  }
}

void PassportRouter::del(const std::vector<PathComponent> &components) {
  if (components.empty()) {
    throw std::runtime_error(
        "Cannot delete collection. Specify a passport GUID: materials.<guid>");
  }

  auto guid = resolve_guid(components[0], *this);

  // Delete entire passport
  if (components.size() == 1) {
    try {
      db_->delete_material_passport(guid);
      spdlog::info("Deleted material passport '{}'", guid);
    } catch (const std::exception &e) {
      throw std::runtime_error("Failed to delete passport '" + guid +
                               "': " + e.what());
    }
    return;
  }

  // Delete single field
  const auto &field_name = components[1].value;

  if (field_name == "document_guid") {
    throw std::runtime_error(
        "Cannot delete document_guid (it's the primary key).");
  }

  // Metadata fields → set to NULL
  if (field_name == "created_at" || field_name == "version_number" ||
      field_name == "revised_at" || field_name == "version_date") {
    try {
      db_->set_passport_metadata_field(guid, field_name, "");
      spdlog::info("Cleared metadata '{}' from passport '{}'", field_name,
                   guid);
    } catch (const std::exception &e) {
      throw std::runtime_error("Failed to clear metadata '" + field_name +
                               "' from passport '" + guid + "': " + e.what());
    }
    return;
  }

  // "properties.X" → resolve the actual property name
  std::string prop_name = field_name;
  if (field_name == "properties" && components.size() >= 3) {
    prop_name = components[2].value;
  } else if (field_name == "properties") {
    throw std::runtime_error(
        "Cannot delete 'properties' directly. Specify a property name: "
        "materials[N].properties.<name> or materials[N].<name>");
  }

  // Delete property value row
  try {
    db_->delete_passport_property(guid, prop_name);
    spdlog::info("Deleted property '{}' from passport '{}'", prop_name, guid);
  } catch (const std::exception &e) {
    throw std::runtime_error("Failed to delete property '" + prop_name +
                             "' from passport '" + guid + "': " + e.what());
  }
}

} // namespace rux::database
