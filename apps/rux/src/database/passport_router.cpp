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

std::vector<std::string> PassportRouter::list() const {
  auto passports = db_->all_material_passports();
  std::vector<std::string> guids;
  guids.reserve(passports.size());

  for (const auto &passport : passports) {
    guids.push_back(passport.metadata.document_guid);
  }

  // Sort alphabetically for deterministic ordering
  std::sort(guids.begin(), guids.end());
  return guids;
}

nlohmann::json
PassportRouter::get_passport_json(std::string_view guid) const {
  auto passport = db_->material_passport(guid);
  return ReUseX::core::json_export::to_json(passport);
}

DataPayload PassportRouter::get(const std::vector<PathComponent> &components) {
  if (components.empty()) {
    // Collection level: return list of passport GUIDs
    auto guids = list();
    return nlohmann::json(guids);
  }

  // Resolve item (GUID)
  std::string guid;

  if (components[0].is_index()) {
    // Array indexing: passports[0]
    auto resolved = resolve_index(*components[0].index);
    if (!resolved) {
      throw std::runtime_error("Array index out of range: " +
                               std::to_string(*components[0].index));
    }
    guid = *resolved;
  } else if (components[0].is_item()) {
    // Direct GUID access: passports.guid-1234
    guid = components[0].value;
  } else {
    throw std::runtime_error("Expected item or index after collection");
  }

  // Check if wildcard (should have been expanded before calling this)
  if (guid.find('*') != std::string::npos) {
    throw std::runtime_error("Wildcard not expanded: " + guid);
  }

  // Get passport as JSON
  nlohmann::json passport_json;
  try {
    passport_json = get_passport_json(guid);
  } catch (const std::exception &e) {
    throw std::runtime_error("Failed to get passport '" + guid +
                             "': " + e.what());
  }

  // If no property specified, return full passport
  if (components.size() == 1) {
    return passport_json;
  }

  // Property access: materials.guid.metadata or materials.guid.sections
  const auto &prop = components[1].value;

  if (!passport_json.contains(prop)) {
    throw std::runtime_error(
        "Unknown property: " + prop +
        "\nAvailable properties: metadata, sections, log");
  }

  auto property_value = passport_json[prop];

  // Handle nested property access (e.g., materials.guid.metadata.document_guid)
  for (size_t i = 2; i < components.size(); ++i) {
    const auto &nested_prop = components[i].value;

    if (!property_value.contains(nested_prop)) {
      // Build path for error message
      std::string path = prop;
      for (size_t j = 2; j <= i; ++j) {
        path += "." + components[j].value;
      }
      throw std::runtime_error("Unknown nested property: " + path);
    }

    property_value = property_value[nested_prop];
  }

  // Return as text if it's a scalar value, otherwise JSON
  if (property_value.is_string() || property_value.is_number() ||
      property_value.is_boolean() || property_value.is_null()) {
    return json_to_text(property_value);
  }

  return property_value;
}

void PassportRouter::set(const std::vector<PathComponent> &components,
                         const DataPayload &data) {
  if (components.empty()) {
    throw std::runtime_error(
        "Cannot set collection directly. Specify a passport GUID: "
        "materials.<guid>");
  }

  // Resolve item (GUID) - support both index and direct access
  std::string guid;
  if (components[0].is_index()) {
    auto resolved = resolve_index(*components[0].index);
    if (!resolved) {
      throw std::runtime_error("Array index out of range: " +
                               std::to_string(*components[0].index));
    }
    guid = *resolved;
  } else if (components[0].is_item()) {
    guid = components[0].value;
  } else {
    throw std::runtime_error("Expected GUID or index after collection");
  }

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
      // For complex JSON, treat as full passport replacement
      value = j.dump();
    }
  } else {
    throw std::runtime_error("Invalid data type for passport property");
  }

  // Case 1: Setting entire passport (no property specified)
  if (components.size() == 1) {
    nlohmann::json json_data;
    try {
      json_data = nlohmann::json::parse(value);
    } catch (const std::exception &e) {
      throw std::runtime_error("Failed to parse JSON: " + std::string(e.what()));
    }

    auto passport = ReUseX::core::json_import::from_json(json_data);
    passport.metadata.document_guid = guid;
    db_->add_material_passport(passport, "default");
    spdlog::info("Saved material passport '{}'", guid);
    return;
  }

  // Case 2: Setting individual property
  // Get existing passport and modify the property in JSON
  nlohmann::json passport_json;
  try {
    passport_json = get_passport_json(guid);
  } catch (const std::exception &e) {
    throw std::runtime_error("Failed to get existing passport '" + guid +
                             "': " + e.what());
  }

  // Navigate to the property and set it
  nlohmann::json *current = &passport_json;

  for (size_t i = 1; i < components.size(); ++i) {
    const auto &prop = components[i].value;

    // Block modification of document_guid (it's the primary key)
    if (i == 2 && components[1].value == "metadata" && prop == "document guid") {
      throw std::runtime_error(
          "Cannot modify document guid (it's the primary key). "
          "Create a new passport with a different GUID instead.");
    }

    if (i == components.size() - 1) {
      // Last component: set the value
      if (!current->contains(prop)) {
        throw std::runtime_error("Unknown property: " + prop);
      }

      // Try to preserve the original type if possible
      auto &target = (*current)[prop];
      if (target.is_number_integer()) {
        try {
          target = std::stoi(value);
        } catch (...) {
          target = value;  // Fallback to string
        }
      } else if (target.is_number_float()) {
        try {
          target = std::stod(value);
        } catch (...) {
          target = value;
        }
      } else if (target.is_boolean()) {
        if (value == "true" || value == "1" || value == "yes") {
          target = true;
        } else if (value == "false" || value == "0" || value == "no") {
          target = false;
        } else {
          target = value;
        }
      } else {
        // String or other type
        target = value;
      }
    } else {
      // Intermediate component: navigate deeper
      if (!current->contains(prop)) {
        throw std::runtime_error("Unknown property path");
      }
      current = &(*current)[prop];
    }
  }

  // Convert back to MaterialPassport and save
  try {
    auto passport = ReUseX::core::json_import::from_json(passport_json);
    db_->add_material_passport(passport, "default");

    // Build property path for log message
    std::string prop_path = components[1].value;
    for (size_t i = 2; i < components.size(); ++i) {
      prop_path += "." + components[i].value;
    }
    spdlog::info("Updated material passport '{}' property '{}'", guid, prop_path);

  } catch (const std::exception &e) {
    throw std::runtime_error("Failed to save updated passport: " +
                             std::string(e.what()));
  }
}

void PassportRouter::del(const std::vector<PathComponent> &components) {
  if (components.empty()) {
    throw std::runtime_error(
        "Cannot delete collection. Specify a passport GUID: materials.<guid>");
  }

  // Resolve item (GUID)
  std::string guid;
  if (components[0].is_index()) {
    auto resolved = resolve_index(*components[0].index);
    if (!resolved) {
      throw std::runtime_error("Array index out of range");
    }
    guid = *resolved;
  } else if (components[0].is_item()) {
    guid = components[0].value;
  } else {
    throw std::runtime_error("Expected GUID or index after collection");
  }

  if (components.size() > 1) {
    throw std::runtime_error(
        "Cannot delete individual properties. Delete the entire passport.");
  }

  // Delete the passport
  try {
    db_->delete_material_passport(guid);
    spdlog::info("Deleted material passport '{}'", guid);
  } catch (const std::exception &e) {
    throw std::runtime_error("Failed to delete passport '" + guid +
                             "': " + e.what());
  }
}

} // namespace rux::database
