// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "import/gsplat.hpp"
#include "exit_status.hpp"

#include <reusex/core/ProjectDB.hpp>

#include <spdlog/spdlog.h>

#include <cstdint>
#include <fstream>
#include <vector>

void setup_subcommand_import_gsplat(CLI::App &app,
                                    std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandImportGsplatOptions>();
  auto *sub = app.add_subcommand(
      "gsplat", "Import a 3D Gaussian Splatting .ply into the project");

  sub->footer(R"(
DESCRIPTION:
  Stores an INRIA-format 3D Gaussian Splatting .ply in the project, where
  'rux gui' renders it as a viewport layer.

  'rux create gsplat' already stores what it trains, so this is for splats
  that live outside a project: runs made before schema v12, and models trained
  by another 3DGS implementation against the same scan.

  The PLY header is validated on the way in. A point cloud exported with
  'rux export ply' is a perfectly valid PLY and is NOT a Gaussian splat; it is
  refused here rather than stored and rendered as nothing.

EXAMPLES:
  rux -p scan.rux import gsplat splat.ply
  rux -p scan.rux import gsplat detailed.ply --name detailed

NOTES:
  - Expected layout: binary_little_endian with x/y/z, f_dc_*, f_rest_*,
    opacity, scale_* and rot_* vertex properties
  - Re-importing under an existing name replaces that splat
  - The splat must be in the same world frame as the project's clouds, or it
    will render offset from them
)");

  sub->add_option("file", opt->input_path, "Path to the splat .ply file.")
      ->required()
      ->check(CLI::ExistingFile);

  sub->add_option("--name", opt->splat_name,
                  "Name to store the splat under in the project")
      ->default_val(opt->splat_name);

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_import_gsplat");
    rux::finish(run_subcommand_import_gsplat(*opt, *global_opt));
  });
}

int run_subcommand_import_gsplat(SubcommandImportGsplatOptions const &opt,
                                 const RuxOptions &global_opt) {
  try {
    std::ifstream in(opt.input_path, std::ios::binary | std::ios::ate);
    if (!in)
      throw std::runtime_error("cannot open '" + opt.input_path.string() + "'");

    const auto size = in.tellg();
    if (size <= 0)
      throw std::runtime_error("'" + opt.input_path.string() + "' is empty");
    in.seekg(0);

    std::vector<uint8_t> ply(static_cast<size_t>(size));
    in.read(reinterpret_cast<char *>(ply.data()),
            static_cast<std::streamsize>(ply.size()));
    if (!in)
      throw std::runtime_error("could not read '" + opt.input_path.string() +
                               "'");

    reusex::ProjectDB db(global_opt.project_db);
    // ProjectDB parses the header, derives the metadata and refuses a PLY that
    // is not a splat — one implementation of that rule, shared with the
    // trainer (docs/STANDARDS.md §1: the CLI validates and delegates).
    db.save_gaussian_splat(opt.splat_name, ply, "import gsplat",
                           R"({"source":")" +
                               opt.input_path.filename().string() + R"("})");

    const auto meta = db.gaussian_splat_metadata(opt.splat_name);
    spdlog::info("Imported {} Gaussians (SH degree {}, {:.1f} MB) as '{}'",
                 meta.gaussian_count, meta.sh_degree,
                 static_cast<double>(meta.byte_size) / (1024.0 * 1024.0),
                 opt.splat_name);
    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Gaussian splat import failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
