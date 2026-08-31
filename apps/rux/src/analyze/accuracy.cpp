// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "analyze/accuracy.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/geometry/accuracy_metrics.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>

namespace fs = std::filesystem;

void setup_subcommand_analyze_accuracy(CLI::App &app,
                                       std::shared_ptr<RuxOptions> global_opt) {

  auto opt = std::make_shared<SubcommandAnalyzeAccuracyOptions>();
  auto *sub = app.add_subcommand(
      "accuracy", "Ground-truth accuracy / completeness / F-score (JSON)");

  sub->footer(R"(
DESCRIPTION:
  Scores a reconstructed point cloud against an external ground-truth point
  cloud (e.g. a Faro survey, or MuSHRoom gt_pd.ply). Both clouds must share a
  common coordinate frame. Distances are in meters:

  - accuracy:     mean/median recon->GT nearest distance (how far the
                  reconstruction strays from the reference).
  - completeness: mean/median GT->recon nearest distance (how much of the
                  reference was reconstructed).
  - chamfer:      mean of accuracy_mean and completeness_mean.
  - precision:    fraction of recon points within --threshold of GT.
  - recall:       fraction of GT points within --threshold of recon.
  - fscore:       2*P*R/(P+R).

EXAMPLES:
  rux analyze accuracy gt_pd.ply                 # print JSON to stdout
  rux analyze accuracy --gt gt_pd.ply -o acc.json
  rux analyze accuracy gt_pd.ply --threshold 0.02 --gt-voxel 0.005

NOTES:
  - The reconstruction cloud must already be aligned to the ground truth.
  - accuracy_mean should be centimeters; meters indicates a misalignment.
  - Lower is better for accuracy/completeness/chamfer; higher for F-score.
)");

  sub->add_option("gt,--gt", opt->gt_path,
                  "Ground-truth PLY point cloud to score against")
      ->required()
      ->check(CLI::ExistingFile);

  sub->add_option("--cloud", opt->cloud_name,
                  "Reconstruction cloud name to score")
      ->default_val(opt->cloud_name);

  sub->add_option("--threshold", opt->threshold,
                  "Inlier distance threshold for precision/recall/F-score [m]")
      ->default_val(opt->threshold)
      ->check(CLI::PositiveNumber);

  sub->add_option("--gt-voxel", opt->gt_voxel,
                  "Ground-truth downsample leaf size [m] (<=0 disables)")
      ->default_val(opt->gt_voxel);

  sub->add_option("-o, --output", opt->output_path,
                  "Write JSON report to this file (default: stdout)")
      ->default_val("");

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling analyze accuracy subcommand");
    return run_subcommand_analyze_accuracy(*opt, *global_opt);
  });
}

int run_subcommand_analyze_accuracy(SubcommandAnalyzeAccuracyOptions const &opt,
                                    const RuxOptions &global_opt) {
  try {
    reusex::ProjectDB db(global_opt.project_db);

    if (!db.has_point_cloud(opt.cloud_name)) {
      spdlog::error("Reconstruction cloud '{}' not found in project",
                    opt.cloud_name);
      return RuxError::INVALID_ARGUMENT;
    }

    auto gt = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (pcl::io::loadPLYFile(opt.gt_path, *gt) != 0) {
      spdlog::error("Failed to load ground-truth PLY '{}'", opt.gt_path);
      return RuxError::IO;
    }
    if (gt->empty()) {
      spdlog::error("Ground-truth PLY '{}' contains no points", opt.gt_path);
      return RuxError::INVALID_ARGUMENT;
    }

    auto cloud = db.point_cloud_xyzrgb(opt.cloud_name);

    reusex::geometry::AccuracyMetricsOptions metrics_options;
    metrics_options.threshold = opt.threshold;
    metrics_options.gt_voxel = opt.gt_voxel;

    const auto report =
        reusex::geometry::compute_accuracy(*cloud, *gt, metrics_options);

    spdlog::info(
        "Accuracy: fscore={:.3f} @ {:.0f}mm, accuracy_mean={:.1f}mm, "
        "completeness_mean={:.1f}mm, precision={:.1f}%, recall={:.1f}% "
        "({} recon vs {} gt points)",
        report.fscore, report.threshold * 1000.0, report.accuracy_mean * 1000.0,
        report.completeness_mean * 1000.0, report.precision * 100.0,
        report.recall * 100.0, report.cloud_points, report.gt_points);

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
    spdlog::error("Accuracy analysis failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
