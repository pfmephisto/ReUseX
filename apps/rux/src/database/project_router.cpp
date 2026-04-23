// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "database/project_router.hpp"

#include "database/format_handler.hpp"

#include <spdlog/spdlog.h>

namespace rux::database {

namespace {

/// Check if value is a known project property (not a project ID)
bool is_known_property(std::string_view value) {
  return value == "id" || value == "name" || value == "building_address" ||
         value == "year_of_construction" || value == "survey_date" ||
         value == "survey_organisation" || value == "notes";
}

} // anonymous namespace

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

  // Resolve project ID and optional property name
  std::string item_id;
  std::string prop;

  if (components[0].is_index()) {
    auto resolved = resolve_index(*components[0].index);
    if (!resolved) {
      throw std::runtime_error("Array index out of range");
    }
    item_id = *resolved;
    if (components.size() >= 2) {
      prop = components[1].value;
    }
  } else if (components[0].is_item()) {
    // Check if this is a property name on the singleton project
    // (e.g., "projects.building_address" instead of "projects[0].building_address")
    if (is_known_property(components[0].value)) {
      auto projects_list = list();
      if (projects_list.empty()) {
        throw std::runtime_error("No projects in database");
      }
      item_id = projects_list[0];
      prop = components[0].value;
    } else {
      item_id = components[0].value;
      if (components.size() >= 2) {
        prop = components[1].value;
      }
    }
  } else {
    throw std::runtime_error("Expected item or index after collection");
  }

  auto project_json = get_project_json(item_id);

  if (prop.empty()) {
    return project_json;
  }

  if (project_json.contains(prop)) {
    return json_to_text(project_json[prop]);
  }

  throw std::runtime_error(
      "Unknown property: " + prop +
      "\nAvailable properties: id, name, building_address, "
      "year_of_construction, survey_date, survey_organisation, notes");
}

void ProjectRouter::set(const std::vector<PathComponent> &components,
                        const DataPayload &data) {
  if (components.empty()) {
    throw std::runtime_error(
        "Cannot set collection directly. Specify a project: projects[0].name");
  }

  // Resolve project ID and property name
  std::string project_id;
  std::string prop;

  if (components[0].is_index()) {
    auto resolved = resolve_index(*components[0].index);
    if (!resolved) {
      throw std::runtime_error("Array index out of range");
    }
    project_id = *resolved;
    if (components.size() >= 2) {
      prop = components[1].value;
    }
  } else if (components[0].is_item()) {
    if (is_known_property(components[0].value)) {
      auto projects_list = list();
      if (projects_list.empty()) {
        throw std::runtime_error("No projects in database");
      }
      project_id = projects_list[0];
      prop = components[0].value;
    } else {
      project_id = components[0].value;
      if (components.size() >= 2) {
        prop = components[1].value;
      }
    }
  } else {
    throw std::runtime_error("Expected item or index after collection");
  }

  if (prop.empty()) {
    throw std::runtime_error(
        "Must specify a property to set: projects[0].name");
  }

  // Get current metadata (or create new if doesn't exist)
  reusex::ProjectDB::ProjectMetadata metadata;
  try {
    metadata = db_->get_project_metadata(project_id);
  } catch (const std::exception &) {
    metadata.id = project_id;
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
