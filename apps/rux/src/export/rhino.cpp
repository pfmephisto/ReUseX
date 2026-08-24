// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "export/rhino.hpp"
#include "global-params.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/export_scene.hpp>
#include <reusex/io/rhino.hpp>
#include <spdlog/spdlog.h>

#include <filesystem>
namespace fs = std::filesystem;
using namespace reusex;

void setup_subcommand_export_rhino(CLI::App &parent,
                                   std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandExportRhinoOptions>();
  auto *sub =
      parent.add_subcommand("rhino", "Export project data to Rhino 3DM");

  sub->footer(R"(
DESCRIPTION:
  Exports all available project data from ProjectDB to Rhino 3DM format
  using OpenNURBS. Automatically includes point clouds, semantic labels,
  meshes, 360 panoramas, and material passports in a structured layer
  hierarchy.

LAYER STRUCTURE:
  ReUseX-<timestamp>/
  ├── cloud          # XYZRGB point cloud + normals if available
  ├── semantic/      # Per-category sublayers with Glasbey colors
  │   ├── wall       # One cloud per instance (or single blob)
  │   ├── ceiling
  │   └── ...
  ├── mesh/          # All meshes as individual ON_Mesh objects
  ├── 360/           # Sphere per panoramic image with user strings
  └── materials/     # TextDots per passport with user strings

EXAMPLES:
  rux export rhino -o output.3dm       # Export everything (latest version)
  rux export rhino -o output.3dm -r 6  # Save for Rhino 6
  rux -p project.rux export rhino      # Custom project path

WORKFLOW:
  1. rux import rtabmap scan.db        # Import sensor data
  2. rux create clouds                 # Generate point cloud
  3. rux export rhino -o scan.3dm      # Export to Rhino

NOTES:
  - Only layers with data are created (empty layers are omitted)
  - Use -r/--rhino-version to target an older Rhino (6, 7, 8, ...)
  - Output .3dm file compatible with Rhino 7+ and OpenNURBS readers by default
  - Point colors preserved from RGB channels
  - Semantic labels use Glasbey LUT for distinct category colors
  - 360 panoramas stored as sphere meshes with metadata in user strings
  - Material passport properties stored as user strings on TextDots
)");

  sub->add_option("-o, --output", opt->path_out,
                  "Path to the output Rhino 3DM file")
      ->default_val(opt->path_out);

  sub->add_option("-r, --rhino-version", opt->rhino_version,
                  "Target Rhino major version to save for (e.g. 6, 7, 8). "
                  "0 uses the latest version supported by OpenNURBS.")
      ->default_val(opt->rhino_version)
      ->check(CLI::IsMember({0, 2, 3, 4, 5, 6, 7, 8}));

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling export rhino subcommand");
    return run_subcommand_export_rhino(*opt, *global_opt);
  });
}

int run_subcommand_export_rhino(SubcommandExportRhinoOptions const &opt,
                                const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;
  spdlog::info("Exporting to Rhino from project: {}", project_path.string());

  try {
    ProjectDB db(project_path, true);

    spdlog::info("Gathering export data...");
    auto scene = io::gather_export_scene(db);

    spdlog::info("Building Rhino model...");
    auto model = io::export_to_rhino(scene);

    // OpenNURBS archive versions are 10x the Rhino major version for V6+
    // (Rhino 6 -> 60, 7 -> 70, 8 -> 80), while pre-V6 use the single-digit
    // version directly (2, 3, 4, 5). 0 lets OpenNURBS pick the latest.
    int version = opt.rhino_version;
    if (version >= 6)
      version *= 10;

    if (version == 0)
      spdlog::trace("Writing Rhino model to: {} (latest version)",
                    opt.path_out.string());
    else
      spdlog::trace("Writing Rhino model to: {} (Rhino {}, archive v{})",
                    opt.path_out.string(), opt.rhino_version, version);

    if (!model->Write(opt.path_out.c_str(), version)) {
      spdlog::error("Failed to write Rhino model to: {}",
                    opt.path_out.string());
      return RuxError::IO;
    }

    spdlog::info("Rhino model successfully written to: {}",
                 opt.path_out.string());
    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Rhino export failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
