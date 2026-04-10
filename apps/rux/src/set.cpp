// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "set.hpp"

#include "database/input_handler.hpp"
#include "database/path_parser.hpp"
#include "database/resource_router.hpp"

#include <reusex/core/ProjectDB.hpp>

#include <spdlog/spdlog.h>

void setup_subcommand_set(CLI::App &app, std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<DatabaseSetOptions>();

  auto *sub = app.add_subcommand(
      "set", "Set data in project database");

  sub->footer(R"(
DESCRIPTION:
  Writes data to ReUseX project database using hierarchical path notation.
  Update metadata, properties, and configuration values. Accepts inline
  values or JSON from stdin for bulk updates and scripting.

EXAMPLES:
  rux set project.name "Office Building"  # Set project name
  rux set clouds.scan1.source rtabmap     # Set cloud metadata
  echo '{"author":"Jane"}' | rux set project.metadata  # Stdin JSON
  rux set passports.guid123.status "approved"  # Update passport

PATH SYNTAX:
  project.PROPERTY         Set project-level property (name, description)
  clouds.NAME.PROPERTY     Set cloud metadata property
  meshes.NAME.PROPERTY     Set mesh metadata property
  passports.GUID.PROPERTY  Set passport field value

VALUE INPUT:
  - Positional argument: rux set path "value"
  - Stdin (JSON): echo '{"key":"val"}' | rux set path
  - Stdin preferred for complex JSON structures

NOTES:
  - Path must specify exact property to set
  - JSON values auto-detected and parsed
  - Use quotes for string values with spaces
  - Stdin mode expects valid JSON
  - Changes written immediately to database
)");

  sub->add_option("path", opt->path,
                  "Resource path (e.g., clouds.newscan, project.name)")
      ->required();

  sub->add_option("value", opt->value,
                  "Inline value (optional, can also use stdin)")
      ->expected(0, 1); // Optional positional argument

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_set");
    return run_subcommand_set(*opt, *global_opt);
  });
}

int run_subcommand_set(const DatabaseSetOptions &opt, const RuxOptions &global_opt) {
  try {
    fs::path project_path = global_opt.project_db;
    spdlog::info("Opening project: {}", project_path.string());
    auto db = std::make_shared<ReUseX::ProjectDB>(project_path,
                                                  /* readOnly */ false);

    // Parse path
    spdlog::debug("Parsing path: {}", opt.path);
    auto components = rux::database::parse_path(opt.path);

    if (components.empty()) {
      spdlog::error("Invalid empty path");
      return 1;
    }

    // Get collection name
    const auto &collection = components[0].value;
    spdlog::debug("Collection: {}", collection);

    // Read input
    auto input_data = rux::database::InputHandler::read_input(opt.value);

    // Create router registry
    rux::database::RouterRegistry registry(db);
    auto &router = registry.get_router(collection);

    // Remove collection from components
    std::vector<rux::database::PathComponent> relative_components(
        components.begin() + 1, components.end());

    // Check for wildcards (not allowed in set)
    for (const auto &comp : relative_components) {
      if (comp.has_wildcard()) {
        spdlog::error("Wildcards not allowed in set operation: {}", comp.value);
        return 1;
      }
    }

    // Perform set operation
    router.set(relative_components, input_data);

    spdlog::info("Successfully set data at path: {}", opt.path);
    return 0;

  } catch (const rux::database::PathError &e) {
    spdlog::error("Path error: {}", e.what());
    return 1;
  } catch (const std::exception &e) {
    spdlog::error("Failed to set data: {}", e.what());
    return 1;
  }
}
