// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "assemble.hpp"
#include "spdmon.hpp"

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <fmt/std.h>

#include <spdlog/spdlog.h>

#include <filesystem>
#include <ranges>
namespace fs = std::filesystem;
// using namespace reusex;

/**
 * @brief Setup CLI options for the assemble subcommand.
 *
 * Configures command-line arguments for assembling multiple scans into
 * a single database.
 *
 * @param app CLI application to add the subcommand to.
 */
void setup_subcommand_assemble(CLI::App &app, std::shared_ptr<RuxOptions> global_opt) {

  auto opt = std::make_shared<SubcommandAssembleOptions>();
  auto *sub = app.add_subcommand(
      "assemble", "Assemble multiple scans into one database");

  sub->footer(R"(
DESCRIPTION:
  Merges multiple RTABMap scan databases into a single unified database
  for large-scale reconstruction projects. Combines sensor frames, poses,
  and map data while preserving spatial relationships. Useful for multi-
  session scanning or collaborative data collection.

EXAMPLES:
  rux assemble scan1.db scan2.db       # Merge to assembled.db
  rux assemble *.db -o building.db     # Merge all scans
  rux assemble floor1.db floor2.db floor3.db  # Multi-floor

WORKFLOW:
  1. Capture multiple RTABMap scans (separate sessions)
  2. rux assemble scan*.db -o unified.db  # Merge databases
  3. rux import rtabmap unified.db     # Import merged data
  4. rux create clouds                 # Process unified scan

NOTES:
  - Input: multiple RTABMap .db files
  - Output: single merged RTABMap database (default: assembled.db)
  - Preserves all sensor data and SLAM graph structure
  - Use for large buildings scanned in multiple sessions
  - Requires RTABMap-compatible databases as input
  - After assembly, use 'rux import rtabmap' for further processing
)");

  sub->add_option("path", opt->paths_in, "Path(s) to the input scan file(s).")
      ->required()
      ->check(CLI::ExistingFile);

  sub->add_option("-o, --out", opt->db_path_out,
                  "Path to save the assembled database to.")
      ->default_val(opt->db_path_out);

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling assemble subcommand");
    return run_subcommand_assemble(*opt, *global_opt);
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
int run_subcommand_assemble(SubcommandAssembleOptions const &opt, [[maybe_unused]] const RuxOptions &global_opt) {

  spdlog::warn("The assemble command is not yet implemented.");
  spdlog::debug("Input files: {}", fmt::join(opt.paths_in, ", "));
  spdlog::debug("Output database: {}", opt.db_path_out);

  std::vector<fs::path> input_files = opt.paths_in;
  std::ranges::sort(input_files);

  spdlog::debug("Input files: {}", fmt::join(input_files, ", "));

  // TODO: Implement multi-scan assembly from rtabmap-reprocess reference
  // category=CLI estimate=1w
  // Need to port logic from RTABMap's reprocess tool to assemble multiple
  // scans: Reference:
  // https://github.com/introlab/rtabmap/blob/master/tools/Reprocess/main.cpp
  // Key steps:
  // 1. Load multiple .db files and extract pose graphs
  // 2. Perform global registration/alignment between scans
  // 3. Set default parameters optimized for building interiors (rux use case)
  // 4. Merge point clouds and update node links in output database
  // 5. Run graph optimization to refine alignment
  // This unblocks the multi-scan workflow for large building projects

  return RuxError::NOT_IMPLEMENTED;
}
