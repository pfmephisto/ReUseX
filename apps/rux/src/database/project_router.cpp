// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "database/project_router.hpp"

#include "database/format_handler.hpp"

#include <spdlog/spdlog.h>

namespace rux::database {

std::vector<std::string> ProjectRouter::list() const {
  return db_->list_project_ids();
}

nlohmann::json ProjectRouter::get_project_json(std::string_view project_id) const {
  auto metadata = db_->get_project_metadata(project_id);
  nlohmann::json project;
  project["id"] = metadata.id;
  project["name"] = metadata.name;
  project["building_address"] = metadata.building_address;

  if (metadata.year_of_construction > 0) {
    project["year_of_construction"] = metadata.year_of_construction;
  } else {
    project["year_of_construction"] = nullptr;
  }

  project["survey_date"] = metadata.survey_date;
  project["survey_organisation"] = metadata.survey_organisation;
  project["notes"] = metadata.notes;
  return project;
}

DataPayload ProjectRouter::get(const std::vector<PathComponent> &components) {
  if (components.empty()) {
    // Collection level: return list of projects
    auto projects_list = list();
    nlohmann::json result = nlohmann::json::array();
    for (const auto &id : projects_list) {
      result.push_back(get_project_json(id));
    }
    return result;
  }

  // projects[0] or projects.default
  std::string item_id;
  if (components[0].is_index()) {
    auto resolved = resolve_index(*components[0].index);
    if (!resolved) {
      throw std::runtime_error("Array index out of range");
    }
    item_id = *resolved;
  } else if (components[0].is_item()) {
    item_id = components[0].value;
  } else {
    throw std::runtime_error("Expected item or index after collection");
  }

  if (components.size() == 1) {
    // Return full project object
    return get_project_json(item_id);
  }

  // Property access: projects[0].name
  const auto &prop = components[1].value;
  auto project_json = get_project_json(item_id);

  if (project_json.contains(prop)) {
    return json_to_text(project_json[prop]);
  } else {
    throw std::runtime_error(
        "Unknown property: " + prop +
        "\nAvailable properties: id, name, building_address, "
        "year_of_construction, survey_date, survey_organisation, notes");
  }
}

void ProjectRouter::set(const std::vector<PathComponent> &components,
                        const DataPayload &data) {
  if (components.empty()) {
    throw std::runtime_error(
        "Cannot set collection directly. Specify a project: projects[0].name");
  }

  // Get project ID
  std::string project_id;
  if (components[0].is_index()) {
    auto resolved = resolve_index(*components[0].index);
    if (!resolved) {
      throw std::runtime_error("Array index out of range");
    }
    project_id = *resolved;
  } else if (components[0].is_item()) {
    project_id = components[0].value;
  } else {
    throw std::runtime_error("Expected item or index after collection");
  }

  // Get current metadata (or create new if doesn't exist)
  ReUseX::ProjectDB::ProjectMetadata metadata;
  try {
    metadata = db_->get_project_metadata(project_id);
  } catch (const std::exception &) {
    // Project doesn't exist, create new one
    metadata.id = project_id;
  }

  // Set property
  if (components.size() < 2) {
    throw std::runtime_error(
        "Must specify a property to set: projects[0].name");
  }

  const auto &prop = components[1].value;

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
    throw std::runtime_error("Invalid data type for project property");
  }

  if (prop == "id") {
    throw std::runtime_error("Cannot modify project ID");
  } else if (prop == "name") {
    metadata.name = value;
  } else if (prop == "building_address") {
    metadata.building_address = value;
  } else if (prop == "year_of_construction") {
    if (value.empty()) {
      metadata.year_of_construction = 0;
    } else {
      try {
        metadata.year_of_construction = std::stoi(value);
      } catch (...) {
        throw std::runtime_error("Invalid year_of_construction: " + value);
      }
    }
  } else if (prop == "survey_date") {
    metadata.survey_date = value;
  } else if (prop == "survey_organisation") {
    metadata.survey_organisation = value;
  } else if (prop == "notes") {
    metadata.notes = value;
  } else {
    throw std::runtime_error(
        "Unknown property: " + prop +
        "\nAvailable properties: name, building_address, "
        "year_of_construction, survey_date, survey_organisation, notes");
  }

  // Update database
  db_->update_project_metadata(metadata);
  spdlog::info("Updated project '{}' property '{}'", project_id, prop);
}

void ProjectRouter::del([[maybe_unused]] const std::vector<PathComponent> &components) {
  throw std::runtime_error("Cannot delete project metadata.");
}

} // namespace rux::database
