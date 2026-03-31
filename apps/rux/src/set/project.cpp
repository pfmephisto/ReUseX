// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "set/project.hpp"

#include <reusex/core/ProjectDB.hpp>

#include <spdlog/spdlog.h>

void setup_subcommand_set_project(CLI::App &app) {
  auto opt = std::make_shared<SubcommandSetProjectOptions>();
  auto *sub = app.add_subcommand(
      "project", "Update project metadata properties");

  sub->add_option("property", opt->property,
                  "Property name (e.g., 'address', 'year', 'building_name')")
      ->required();

  sub->add_option("value", opt->value,
                  "Property value")
      ->required();

  sub->add_option("--project", opt->project,
                  "Path to the ReUseX project database (.rux)")
      ->default_val(opt->project)
      ->check(CLI::ExistingFile);

  sub->callback([opt]() {
    spdlog::trace("calling run_subcommand_set_project");
    return run_subcommand_set_project(*opt);
  });
}

int run_subcommand_set_project(SubcommandSetProjectOptions const &opt) {
  spdlog::info("Setting project property '{}' = '{}'",
               opt.property, opt.value);

  // TODO: Implement project metadata updates
  // 1. Validate property name against schema
  // 2. Update projects table in ProjectDB
  // 3. May need to add project metadata API to ProjectDB first
  //    (currently no direct access to projects table)

  spdlog::error("Project property updates not yet implemented");
  spdlog::info("To implement: add ProjectDB API for projects table");

  return RuxError::NOT_IMPLEMENTED;
}
