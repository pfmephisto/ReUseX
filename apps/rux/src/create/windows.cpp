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

void setup_subcommand_create_windows(CLI::App &app, std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandWindowOptions>();
  auto *sub = app.add_subcommand(
      "windows", "Create window building components");

  sub->footer(R"(
DESCRIPTION:
  Generates window building components from semantic segmentation labels
  and instance data. Identifies window instances, computes bounding boxes,
  and creates structured component records suitable for BIM export and
  material passport generation.

EXAMPLES:
  rux create windows                   # Create from 'labels' cloud
  rux -p scan.rux create windows       # Custom project path

WORKFLOW:
  1. rux create annotate --net model   # Semantic segmentation
  2. rux create clouds                 # Reconstruct labeled cloud
  3. rux create instances -l 5         # Instance segment windows (label 5)
  4. rux create windows                # Generate window components
  5. rux get components                # View component data

NOTES:
  - Requires 'labels' semantic cloud with window instances
  - Automatically computes component geometry and metadata
  - Output stored in project database components table
  - Integrates with material passport system
  - Use 'rux create instances' first to separate individual windows
)");

  // sub->add_option("-m, --mesh-name", opt->mesh_name,
  //                 "Name of the input mesh in ProjectDB")
  //     ->default_val(opt->mesh_name);

  // sub->add_option("-o, --output-name", opt->output_name,
  //                 "Name for the output textured mesh in ProjectDB")
  //     ->default_val(opt->output_name);

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_texture");
    return run_subcommand_create_windows(*opt, *global_opt);
  });
}
int run_subcommand_create_windows([[maybe_unused]] SubcommandWindowOptions const &opt, const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;
  spdlog::info("Create windows in project: {}", project_path.string());

  ReUseX::ProjectDB db(project_path);

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
