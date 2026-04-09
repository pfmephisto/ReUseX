// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "del.hpp"

#include "database/path_parser.hpp"
#include "database/resource_router.hpp"

#include <reusex/core/ProjectDB.hpp>

#include <fmt/format.h>
#include <iostream>
#include <spdlog/spdlog.h>

namespace {
constexpr int kMaxWildcardExpansion = 1000; // Safety limit

/**
 * @brief Prompt user for confirmation
 * @return true if user confirms (y/Y)
 */
bool prompt_confirmation() {
  std::cout << "Continue? [y/N] " << std::flush;
  std::string response;
  std::getline(std::cin, response);

  return !response.empty() && (response[0] == 'y' || response[0] == 'Y');
}

} // namespace

void setup_subcommand_del(CLI::App &app) {
  auto opt = std::make_shared<DatabaseDelOptions>();

  auto *sub = app.add_subcommand(
      "del", "Delete data from project database using path-based access");

  sub->add_option("project_file", opt->project_file,
                  "Path to the ReUseX project database (.rux)")
      ->required()
      ->check(CLI::ExistingFile);

  sub->add_option("path", opt->path,
                  "Resource path (e.g., clouds.oldcloud, clouds.test_*)")
      ->required();

  sub->add_flag("-f,--force", opt->force,
                "Force deletion (required for wildcards, shows preview + "
                "confirmation)")
      ->default_val(false);

  sub->add_flag("-y,--yes", opt->yes,
                "Skip confirmation prompt (use with --force for scripts)")
      ->default_val(false);

  sub->callback([opt]() {
    spdlog::trace("calling run_subcommand_del");
    return run_subcommand_del(*opt);
  });
}

int run_subcommand_del(const DatabaseDelOptions &opt) {
  try {
    spdlog::info("Opening project: {}", opt.project_file.string());
    auto db = std::make_shared<ReUseX::ProjectDB>(opt.project_file,
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

    // Create router registry
    rux::database::RouterRegistry registry(db);
    auto &router = registry.get_router(collection);

    // Remove collection from components
    std::vector<rux::database::PathComponent> relative_components(
        components.begin() + 1, components.end());

    // Check for wildcards
    bool has_wildcard = false;
    for (const auto &comp : relative_components) {
      if (comp.has_wildcard()) {
        has_wildcard = true;
        break;
      }
    }

    if (has_wildcard && !opt.force) {
      spdlog::error(
          "Wildcard deletion requires --force flag to prevent accidents");
      fmt::print("Hint: Use 'rux del {} {} --force' to proceed\n",
                 opt.project_file.string(), opt.path);
      return 1;
    }

    // Expand wildcards
    auto expanded_paths =
        rux::database::expand_wildcards(relative_components, router);

    if (expanded_paths.empty()) {
      spdlog::warn("Path matched no resources to delete");
      return 0;
    }

    // Safety check: limit wildcard expansion
    if (expanded_paths.size() > kMaxWildcardExpansion) {
      spdlog::error("Wildcard expansion exceeds safety limit ({} > {})",
                    expanded_paths.size(), kMaxWildcardExpansion);
      return 1;
    }

    // Show preview for multiple items
    if (expanded_paths.size() > 1) {
      fmt::print("This will delete {} item(s) from {}:\n",
                 expanded_paths.size(), opt.project_file.string());

      for (const auto &path : expanded_paths) {
        if (!path.empty() && path[0].is_item()) {
          fmt::print("  - {}\n", path[0].value);
        }
      }

      // Prompt for confirmation unless --yes
      if (!opt.yes) {
        if (!prompt_confirmation()) {
          spdlog::info("Deletion cancelled by user");
          return 0;
        }
      }
    }

    // Perform deletions
    int deleted_count = 0;
    int failed_count = 0;

    for (const auto &path : expanded_paths) {
      try {
        router.del(path);
        deleted_count++;

        if (!path.empty() && path[0].is_item()) {
          spdlog::info("Deleted: {}", path[0].value);
        }
      } catch (const std::exception &e) {
        failed_count++;
        if (!path.empty() && path[0].is_item()) {
          spdlog::error("Failed to delete {}: {}", path[0].value, e.what());
        }
      }
    }

    if (deleted_count > 0) {
      fmt::print("Successfully deleted {} item(s)\n", deleted_count);
    }

    if (failed_count > 0) {
      spdlog::error("Failed to delete {} item(s)", failed_count);
      return 1;
    }

    return 0;

  } catch (const rux::database::PathError &e) {
    spdlog::error("Path error: {}", e.what());
    return 1;
  } catch (const std::exception &e) {
    spdlog::error("Failed to delete data: {}", e.what());
    return 1;
  }
}
