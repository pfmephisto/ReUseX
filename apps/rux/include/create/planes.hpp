// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"
#include <CLI/CLI.hpp>
#include <memory>

#include <reusex/geometry/segment_planes.hpp>

namespace fs = std::filesystem;

/// Collection of all options for plane segmentation subcommand.
///
/// Defaults are derived from reusex::geometry::SegmentPlanesOptions so the CLI
/// and library never disagree on a parameter's default (docs/STANDARDS.md §4).
struct SubcommandSegPlanesOptions {

  /// Angular threshold for plane detection (degrees).
  float angle_threshold =
      reusex::geometry::SegmentPlanesOptions{}.angle_threshold;
  /// Distance threshold for plane detection.
  float plane_dist_threshold =
      reusex::geometry::SegmentPlanesOptions{}.plane_dist_threshold;
  /// Minimum number of inliers for a valid plane.
  int minInliers = reusex::geometry::SegmentPlanesOptions{}.min_inliers;
  float radius = reusex::geometry::SegmentPlanesOptions{}.radius;
  float interval_0 = reusex::geometry::SegmentPlanesOptions{}.interval_0;
  float interval_factor =
      reusex::geometry::SegmentPlanesOptions{}.interval_factor;

  bool adaptive = true; ///< Derive thresholds from measured cloud noise (#214)
  unsigned noise_seed = 42;   ///< Deterministic seed for the noise estimator
  bool dist_explicit = false; ///< User passed -d (pins distance threshold)
  bool min_explicit = false;  ///< User passed -m (pins min_inliers)

  std::string filter_expr; ///< Filter expression to limit processing
};

/**
 * @brief Setup the segment planes subcommand in the CLI application.
 * @param app CLI application to add the subcommand to.
 */
void setup_subcommand_create_planes(CLI::App &app,
                                    std::shared_ptr<RuxOptions> global_opt);

/**
 * @brief Run the segment planes subcommand with given options.
 * @param opt Options for plane segmentation.
 * @return Exit code (0 for success).
 */
int run_subcommand_segment_planes(SubcommandSegPlanesOptions const &opt,
                                  const RuxOptions &global_opt);
