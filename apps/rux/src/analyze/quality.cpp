// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "analyze/quality.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/geometry/quality_metrics.hpp>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <filesystem>
#include <fstream>
#include <iostream>

namespace fs = std::filesystem;

void setup_subcommand_analyze_quality(CLI::App &app,
                                      std::shared_ptr<RuxOptions> global_opt) {

  auto opt = std::make_shared<SubcommandAnalyzeQualityOptions>();
  auto *sub = app.add_subcommand(
      "quality", "Plane flatness / surface thickness report (JSON)");

  sub->footer(R"(
DESCRIPTION:
  Computes ground-truth-free reconstruction quality metrics from a point
  cloud and its plane segmentation:

  - flatness_rms:   point-weighted mean RMS point-to-plane residual [m].
                    Pose drift smears wall points, inflating this value.
  - thickness_p90:  point-weighted mean 90th-percentile |residual| [m].
                    Misaligned passes double surfaces; p90 grows much
                    faster than RMS when walls double.

  Planes are refit by least squares from the labeled points, independent
  of stored plane parameters.

EXAMPLES:
  rux analyze quality                       # print JSON to stdout
  rux analyze quality -o baseline.json      # write to file
  rux analyze quality --cloud cloud_5cm     # analyze a different cloud

NOTES:
  - Requires '<cloud>' and '<planes>' clouds of equal size in the project
  - Lower is better for all metrics; compare across pipeline changes
)");

  sub->add_option("--cloud", opt->cloud_name, "Point cloud name to analyze")
      ->default_val(opt->cloud_name);

  sub->add_option("--planes", opt->planes_name, "Plane label cloud name")
      ->default_val(opt->planes_name);

  sub->add_option("--min-points", opt->min_points,
                  "Minimum supporting points for a plane to be reported")
      ->default_val(opt->min_points)
      ->check(CLI::Range(static_cast<std::size_t>(3),
                         static_cast<std::size_t>(10'000'000)));

  sub->add_option("-o, --output", opt->output_path,
                  "Write JSON report to this file (default: stdout)")
      ->default_val("");

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling analyze quality subcommand");
    return run_subcommand_analyze_quality(*opt, *global_opt);
  });
}

int run_subcommand_analyze_quality(SubcommandAnalyzeQualityOptions const &opt,
                                   const RuxOptions &global_opt) {
  try {
    reusex::ProjectDB db(global_opt.project_db);

    if (!db.has_point_cloud(opt.cloud_name)) {
      spdlog::error("Point cloud '{}' not found in project", opt.cloud_name);
      return RuxError::INVALID_ARGUMENT;
    }
    if (!db.has_point_cloud(opt.planes_name)) {
      spdlog::error("Plane label cloud '{}' not found in project; run "
                    "'rux create planes' first",
                    opt.planes_name);
      return RuxError::INVALID_ARGUMENT;
    }

    auto cloud = db.point_cloud_xyzrgb(opt.cloud_name);
    auto planes = db.point_cloud_label(opt.planes_name);

    reusex::geometry::QualityMetricsOptions metrics_options;
    metrics_options.min_points = opt.min_points;

    const auto report = reusex::geometry::compute_plane_quality(
        *cloud, *planes, metrics_options);

    spdlog::info("Quality: {} planes, flatness_rms={:.4f}m (max {:.4f}m), "
                 "thickness_p90={:.4f}m, {}/{} points labeled",
                 report.planes.size(), report.flatness_rms,
                 report.flatness_rms_max, report.thickness_p90,
                 report.labeled_points, report.total_points);

    const std::string json = report.to_json();
    if (opt.output_path.empty()) {
      std::cout << json << '\n';
    } else {
      std::ofstream out(opt.output_path);
      if (!out) {
        spdlog::error("Cannot write to '{}'", opt.output_path);
        return RuxError::IO;
      }
      out << json << '\n';
      spdlog::info("Report written to {}", opt.output_path);
    }

    return RuxError::SUCCESS;
  } catch (const std::exception &e) {
    spdlog::error("Quality analysis failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
