// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/instances.hpp"
#include "global-params.hpp"
#include "validation.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/geometry/segment_instances.hpp>
#include <spdlog/spdlog.h>

#include <fmt/ranges.h>
#include <algorithm>
#include <chrono>
#include <set>

void setup_subcommand_create_instances(CLI::App &app, std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandSegInstancesOptions>();

  auto *sub = app.add_subcommand(
      "instances",
      "Separate labels into spatial instances");

  sub->footer(R"(
DESCRIPTION:
  Separates semantic segmentation labels into distinct spatial instances
  using Euclidean distance clustering. For example, multiple 'window'
  labels become 'window_1', 'window_2', etc. based on spatial separation.
  Essential for distinguishing individual objects of the same class.

EXAMPLES:
  rux create instances                 # Process all labels
  rux create instances -t 0.3          # 30cm clustering tolerance
  rux create instances -l 3,5,7        # Only labels 3, 5, 7
  rux create instances -s labels -o inst  # Custom input/output

WORKFLOW:
  1. rux import rtabmap scan.db        # Import sensor data
  2. rux create annotate --net model   # Semantic segmentation
  3. rux create clouds                 # Reconstruct labeled cloud
  4. rux create instances              # Instance segmentation
  5. rux view                          # Visualize instances

NOTES:
  - Requires 'cloud' and semantic label cloud (default: 'labels')
  - Tolerance: Euclidean distance threshold for clustering (meters)
  - Min/max size filters noise and oversized clusters
  - Use --labels to process specific semantic classes only
  - Output includes per-instance metadata and statistics
  - Saved as new label cloud with instance IDs
)");

  sub->add_option("-t,--tolerance", opt->cluster_tolerance,
                  "Euclidean distance threshold for clustering (meters)")
      ->default_val(0.5F)
      ->check(CLI::Range(0.01, 5.0));

  sub->add_option("-m,--min-size", opt->min_cluster_size,
                  "Minimum points per instance cluster")
      ->default_val(50)
      ->check(CLI::Range(1, 100000));

  sub->add_option("-M,--max-size", opt->max_cluster_size,
                  "Maximum points per instance cluster")
      ->default_val(1000000)
      ->check(CLI::Range(10, 10000000));

  sub->add_option("-s,--semantic", opt->semantic_cloud_name,
                  "Input semantic labels cloud name")
      ->default_val("labels");

  sub->add_option("-o,--output", opt->output_cloud_name,
                  "Output instance labels cloud name")
      ->default_val("instances");

  sub->add_option("-l,--labels", opt->labels_to_process,
                  "Comma-separated list of semantic labels to process (empty = all)")
      ->delimiter(',');

  sub->callback([opt, global_opt]() {
    int exit_code = run_subcommand_segment_instances(*opt, *global_opt);
    if (exit_code != RuxError::SUCCESS) {
      throw CLI::RuntimeError(exit_code);
    }
  });
}

int run_subcommand_segment_instances(const SubcommandSegInstancesOptions &opt, const RuxOptions &global_opt) {
  try {
    fs::path project_path = global_opt.project_db;
    spdlog::info("Opening project database: {}", project_path.string());
    ReUseX::ProjectDB db(project_path);

    // Validate prerequisites
    spdlog::info("Validating prerequisites...");
    auto validation = rux::validation::validate_instances_prerequisites(
        db, opt.semantic_cloud_name);
    if (!validation.success) {
      spdlog::error("Validation failed: {}", validation.error_message);
      if (!validation.resolution_hint.empty()) {
        spdlog::info("Suggestion: {}", validation.resolution_hint);
      }
      return RuxError::INVALID_ARGUMENT;
    }

    // Log pipeline start
    int logId = db.log_pipeline_start(
        "segment_instances",
        fmt::format(R"({{"tolerance":{},"min_size":{},"max_size":{}}})",
                    opt.cluster_tolerance, opt.min_cluster_size,
                    opt.max_cluster_size));

    // Load input data
    spdlog::info("Loading point cloud and semantic labels...");
    auto cloud = db.point_cloud_xyzrgb("cloud");
    auto semantic_labels = db.point_cloud_label(opt.semantic_cloud_name);

    if (!cloud || !semantic_labels) {
      spdlog::error("Failed to load required data");
      return RuxError::IO;
    }

    spdlog::info("Loaded {} points with semantic labels", cloud->size());

    // Build request
    ReUseX::geometry::SegmentInstancesRequest request;
    request.cloud = cloud;
    request.semantic_labels = semantic_labels;
    request.cluster_tolerance = opt.cluster_tolerance;
    request.min_cluster_size = opt.min_cluster_size;
    request.max_cluster_size = opt.max_cluster_size;

    // Convert label vector to set
    if (!opt.labels_to_process.empty()) {
      request.labels_to_process = std::set<uint32_t>(
          opt.labels_to_process.begin(), opt.labels_to_process.end());
      spdlog::info("Processing only labels: {}",
                   fmt::join(opt.labels_to_process, ", "));
    }

    // Run segmentation
    auto start_time = std::chrono::steady_clock::now();
    spdlog::info("Starting instance segmentation...");
    auto result = ReUseX::geometry::segment_instances(request);
    auto end_time = std::chrono::steady_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                        end_time - start_time)
                        .count();
    spdlog::info("Instance segmentation completed in {:.2f}s",
                 duration / 1000.0);

    // Save results
    spdlog::info("Saving instance labels as '{}'...", opt.output_cloud_name);
    db.save_point_cloud(opt.output_cloud_name, *result.instance_labels,
                        "segment_instances");

    // Save metadata as label definitions
    // Format: instance_id -> "semantic_class_N_instance_M (P points)"
    std::map<int, std::string> label_definitions;
    for (const auto &[instance_id, semantic_class] :
         result.instance_to_semantic) {
      size_t size = result.instance_sizes.at(instance_id);
      label_definitions[static_cast<int>(instance_id)] = fmt::format(
          "semantic_class_{}_instance_{} ({} points)", semantic_class,
          instance_id, size);
    }
    db.save_label_definitions(opt.output_cloud_name, label_definitions);

    spdlog::info("Saved {} instance definitions", label_definitions.size());

    // Log statistics
    size_t labeled_points = 0;
    for (const auto &label : *result.instance_labels) {
      if (label.label > 0) {
        ++labeled_points;
      }
    }

    spdlog::info("Summary:");
    spdlog::info("  Total instances: {}", result.instance_to_semantic.size());
    spdlog::info("  Labeled points: {}/{} ({:.1f}%)", labeled_points,
                 cloud->size(), 100.0 * labeled_points / cloud->size());

    // Print per-semantic-class breakdown
    std::map<uint32_t, std::vector<uint32_t>> semantic_to_instances;
    for (const auto &[instance_id, semantic_class] :
         result.instance_to_semantic) {
      semantic_to_instances[semantic_class].push_back(instance_id);
    }

    spdlog::info("Per-class breakdown:");
    for (const auto &[semantic_class, instances] : semantic_to_instances) {
      size_t total_points = 0;
      for (uint32_t instance_id : instances) {
        total_points += result.instance_sizes.at(instance_id);
      }
      spdlog::info("  Semantic class {}: {} instances, {} points",
                   semantic_class, instances.size(), total_points);
    }

    // Log pipeline end
    db.log_pipeline_end(logId, true);

    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Instance segmentation failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
