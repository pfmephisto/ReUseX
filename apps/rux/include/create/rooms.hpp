// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"
#include <CLI/CLI.hpp>
#include <memory>

#include <reusex/geometry/segment_rooms.hpp>

namespace fs = std::filesystem;

/// Collection of all options for room segmentation subcommand.
///
/// Defaults are derived from reusex::geometry::SegmentRoomsOptions so the CLI
/// and library never disagree on a parameter's default (docs/STANDARDS.md §4).
struct SubcommandSegRoomsOptions {
  /// Leiden resolution parameter (cluster granularity).
  float resolution = reusex::geometry::SegmentRoomsOptions{}.resolution;
  /// Leiden beta (refinement randomness).
  float beta = reusex::geometry::SegmentRoomsOptions{}.beta;
  /// Maximum Leiden iterations (finite bound, see library default).
  int max_iter = reusex::geometry::SegmentRoomsOptions{}.max_iter;

  float grid_size = reusex::geometry::SegmentRoomsOptions{}.grid_size;

  /// Distance-bounded k-NN label propagation to non-sampled points.
  float propagate_max_radius =
      reusex::geometry::SegmentRoomsOptions{}.propagate_max_radius;

  std::string filter_expr; ///< Filter expression to limit processing
};

/**
 * @brief Setup the segment rooms subcommand in the CLI application.
 * @param app CLI application to add the subcommand to.
 */
void setup_subcommand_create_rooms(CLI::App &app,
                                   std::shared_ptr<RuxOptions> global_opt);

/**
 * @brief Run the segment rooms subcommand with given options.
 * @param opt Options for room segmentation.
 * @return Exit code (0 for success).
 */
int run_subcommand_segment_rooms(SubcommandSegRoomsOptions const &opt,
                                 const RuxOptions &global_opt);
