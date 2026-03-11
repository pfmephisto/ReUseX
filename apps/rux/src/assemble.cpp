// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "assemble.hpp"
#include "spdmon/spdmon.hpp"

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <fmt/std.h>

#include <spdlog/spdlog.h>

#include <filesystem>
#include <ranges>
namespace fs = std::filesystem;
// using namespace ReUseX;

/**
 * @brief Setup CLI options for the assemble subcommand.
 * 
 * Configures command-line arguments for assembling multiple scans into
 * a single database.
 * 
 * @param app CLI application to add the subcommand to.
 */
void setup_subcommand_assemble(CLI::App &app) {

  auto opt = std::make_shared<SubcommandAssembleOptions>();
  auto *sub =
      app.add_subcommand("assemble", "Assemble multiple scans into a single RTAB-Map database.");

  sub->add_option("path", opt->paths_in, "Path(s) to the input scan file(s).")
      ->required()
      ->check(CLI::ExistingFile);

  sub->add_option("-o, --out", opt->db_path_out,
                  "Path to save the assembled database to.")
      ->default_val(opt->db_path_out);

  sub->callback([opt]() {
    spdlog::trace("calling assemble subcommand");
    return run_subcommand_assemble(*opt);
  });
}

/**
 * @brief Execute the assemble operation on multiple scan files.
 * 
 * Assembles multiple scan files into a single RTABMap-compatible database.
 * Currently not fully implemented.
 * 
 * @param opt Options containing input file paths and output database path.
 * @return Exit code (RuxError::NOT_IMPLEMENTED currently).
 */
int run_subcommand_assemble(SubcommandAssembleOptions const &opt) {

  spdlog::warn("The assemble command is not yet implemented.");
  spdlog::debug("Input files: {}", fmt::join(opt.paths_in, ", "));
  spdlog::debug("Output database: {}", opt.db_path_out);

  std::vector<fs::path> input_files = opt.paths_in;
  std::ranges::sort(input_files);

  spdlog::debug("Input files: {}", fmt::join(input_files, ", "));

  // TODO: Copy the implementation from rtabmap-reprocess
  // Sett the defualt parameters that work well for rux
  // https://github.com/introlab/rtabmap/blob/master/tools/Reprocess/main.cpp

  return RuxError::NOT_IMPLEMENTED;
}
