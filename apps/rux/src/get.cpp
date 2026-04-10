// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "get.hpp"

#include "database/format_handler.hpp"
#include "database/path_parser.hpp"
#include "database/resource_router.hpp"

#include <reusex/core/ProjectDB.hpp>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

void setup_subcommand_get(CLI::App &app, std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<DatabaseGetOptions>();

  auto *sub = app.add_subcommand(
      "get", "Get data from project database");

  sub->footer(R"(
DESCRIPTION:
  Retrieves data from ReUseX project database using hierarchical path
  notation. Query point clouds, meshes, components, metadata, and material
  passports with dot-separated paths. Outputs JSON for piping and scripting.

EXAMPLES:
  rux get                              # List all collections
  rux get clouds                       # List all point clouds
  rux get clouds.scan1                 # Get scan1 metadata
  rux get clouds.scan1.type            # Get specific property
  rux get components --pretty          # Pretty-print JSON
  rux get passports -o data.json       # Save to file

PATH SYNTAX:
  collections              List available collections (clouds, meshes, etc.)
  clouds                   List all cloud names
  clouds.NAME              Get cloud metadata
  clouds.NAME.PROPERTY     Get specific property (type, size, source)
  meshes.NAME              Get mesh metadata
  components               List all building components
  passports                List all material passports

NOTES:
  - Default output: compact JSON to stdout
  - Use --pretty for human-readable formatting
  - Use -o/--output to save to file instead of stdout
  - Empty path lists all top-level collections
  - Paths are case-sensitive
)");

  sub->add_option("path", opt->path,
                  "Resource path (e.g., clouds, clouds.scan1, "
                  "clouds.scan1.metadata)\n"
                  "If omitted, shows all available collections");

  sub->add_option("-o,--output", opt->output_file,
                  "Write output to file instead of stdout");

  sub->add_flag("-p,--pretty", opt->pretty,
                "Pretty-print JSON output (default: auto-detect from TTY)")
      ->default_val(false);

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_get");
    return run_subcommand_get(*opt, *global_opt);
  });
}

int run_subcommand_get(const DatabaseGetOptions &opt, const RuxOptions &global_opt) {
  try {
    fs::path project_path = global_opt.project_db;
    spdlog::info("Opening project: {}", project_path.string());
    auto db = std::make_shared<ReUseX::ProjectDB>(project_path,
                                                  /* readOnly */ true);

    // If no path provided, show available collection names
    if (opt.path.empty()) {
      spdlog::debug("No path provided, listing collection names");

      nlohmann::json collections =
          nlohmann::json::array({"clouds", "materials", "meshes", "projects"});

      rux::database::DataPayload payload = collections;

      if (opt.output_file.empty()) {
        rux::database::write_output(payload, opt.pretty);
      } else {
        rux::database::write_to_file(payload, opt.output_file);
      }

      return 0;
    }

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

    // Create router registry
    rux::database::RouterRegistry registry(db);
    auto &router = registry.get_router(collection);

    // Remove collection from components (routers work with relative paths)
    std::vector<rux::database::PathComponent> relative_components(
        components.begin() + 1, components.end());

    // Expand wildcards if present
    auto expanded_paths =
        rux::database::expand_wildcards(relative_components, router);

    if (expanded_paths.empty()) {
      spdlog::warn("Path matched no resources");
      return 0;
    }

    if (expanded_paths.size() == 1) {
      // Single path: get and output directly
      auto payload = router.get(expanded_paths[0]);

      if (opt.output_file.empty()) {
        rux::database::write_output(payload, opt.pretty);
      } else {
        rux::database::write_to_file(payload, opt.output_file);
      }
    } else {
      // Multiple paths (wildcard expansion): collect metadata as JSON array
      spdlog::debug("Wildcard matched {} items, collecting metadata",
                    expanded_paths.size());

      nlohmann::json results = nlohmann::json::array();

      for (const auto &path : expanded_paths) {
        try {
          auto payload = router.get(path);

          // Convert payload to JSON
          if (std::holds_alternative<nlohmann::json>(payload)) {
            results.push_back(std::get<nlohmann::json>(payload));
          } else if (std::holds_alternative<std::string>(payload)) {
            results.push_back(std::get<std::string>(payload));
          } else {
            // Binary data: skip for wildcard results
            spdlog::warn("Skipping binary data in wildcard expansion");
          }
        } catch (const std::exception &e) {
          spdlog::error("Failed to get {}: {}", path[0].value, e.what());
        }
      }

      rux::database::DataPayload result_payload = results;

      if (opt.output_file.empty()) {
        rux::database::write_output(result_payload, opt.pretty);
      } else {
        rux::database::write_to_file(result_payload, opt.output_file);
      }
    }

    return 0;

  } catch (const rux::database::PathError &e) {
    spdlog::error("Path error: {}", e.what());
    return 1;
  } catch (const std::exception &e) {
    spdlog::error("Failed to get data: {}", e.what());
    return 1;
  }
}
