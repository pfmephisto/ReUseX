// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/windows.hpp"
#include "validation.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/geometry/texture_mesh.hpp>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <fmt/format.h>

#include <Eigen/Dense>

void setup_subcommand_create_windows(CLI::App &app) {
  auto opt = std::make_shared<SubcommandWindowOptions>();
  auto *sub = app.add_subcommand(
      "windows", "Create windows based on a semantic lable and instances.");

  sub->add_option("project", opt->project, "Path to the .rux project file.")
      ->required()
      ->check(CLI::ExistingFile);

  // sub->add_option("-m, --mesh-name", opt->mesh_name,
  //                 "Name of the input mesh in ProjectDB")
  //     ->default_val(opt->mesh_name);

  // sub->add_option("-o, --output-name", opt->output_name,
  //                 "Name for the output textured mesh in ProjectDB")
  //     ->default_val(opt->output_name);

  sub->callback([opt]() {
    spdlog::trace("calling run_subcommand_texture");
    return run_subcommand_create_windows(*opt);
  });
}
int run_subcommand_create_windows(SubcommandWindowOptions const &opt) {
  spdlog::info("Create windows in project: {}", opt.project.string());

  ReUseX::ProjectDB db(opt.project);

  // Pre-flight validation: check for mesh and sensor frames
  auto validation =
      rux::validation::validate_window_prerequisites(db, "labels");
  if (!validation) {
    spdlog::error("{}", validation.error_message);
    spdlog::info("Resolution: {}", validation.resolution_hint);
    return RuxError::INVALID_ARGUMENT;
  }

  return RuxError::NOT_IMPLEMENTED;
}
