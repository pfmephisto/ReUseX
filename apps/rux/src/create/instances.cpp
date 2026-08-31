// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/instances.hpp"
#include "global-params.hpp"
#include "validation.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/guid.hpp>
#include <reusex/geometry/reconcile_instances.hpp>
#include <reusex/geometry/segment_instances.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <fmt/ranges.h>
#include <set>

void setup_subcommand_create_instances(CLI::App &app,
                                       std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandSegInstancesOptions>();

  auto *sub =
      app.add_subcommand("instances", "Separate labels into spatial instances");

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
      ->default_val(opt->cluster_tolerance)
      ->check(CLI::Range(0.01, 5.0));

  sub->add_option("-m,--min-size", opt->min_cluster_size,
                  "Minimum points per instance cluster")
      ->default_val(opt->min_cluster_size)
      ->check(CLI::Range(1, 100000));

  sub->add_option("-M,--max-size", opt->max_cluster_size,
                  "Maximum points per instance cluster")
      ->default_val(opt->max_cluster_size)
      ->check(CLI::Range(10, 10000000));

  sub->add_option("-s,--semantic", opt->semantic_cloud_name,
                  "Input semantic labels cloud name")
      ->default_val("labels");

  sub->add_option("-o,--output", opt->output_cloud_name,
                  "Output instance labels cloud name")
      ->default_val("instances");

  sub->add_option(
         "-l,--labels", opt->labels_to_process,
         "Comma-separated list of semantic labels to process (empty = all)")
      ->delimiter(',');

  sub->callback([opt, global_opt]() {
    int exit_code = run_subcommand_segment_instances(*opt, *global_opt);
    if (exit_code != RuxError::SUCCESS) {
      throw CLI::RuntimeError(exit_code);
    }
  });
}

int run_subcommand_segment_instances(const SubcommandSegInstancesOptions &opt,
                                     const RuxOptions &global_opt) {
  try {
    fs::path project_path = global_opt.project_db;
    spdlog::info("Opening project database: {}", project_path.string());
    reusex::ProjectDB db(project_path);

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
    reusex::geometry::SegmentInstancesRequest request;
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
    auto result = reusex::geometry::segment_instances(request);
    auto end_time = std::chrono::steady_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                        end_time - start_time)
                        .count();
    spdlog::info("Instance segmentation completed in {:.2f}s",
                 duration / 1000.0);

    // --- Stable identity: carry GUIDs across regeneration (issue #207) ---
    // Load the previous instance state (label cloud + instances rows) BEFORE
    // overwriting the output cloud, so we can match new instances to old ones
    // and preserve their GUIDs and material links.
    reusex::CloudLPtr old_labels;
    std::vector<reusex::ProjectDB::InstanceRecord> old_instances;
    std::map<int, std::string> old_links; // old instance_id -> material_guid
    if (db.has_point_cloud(opt.output_cloud_name)) {
      try {
        old_labels = db.point_cloud_label(opt.output_cloud_name);
        old_instances = db.instances(opt.output_cloud_name);
        old_links = db.instance_materials(opt.output_cloud_name);
      } catch (const std::exception &e) {
        spdlog::warn("Could not load previous instance state for '{}': {}",
                     opt.output_cloud_name, e.what());
      }
    }

    std::vector<reusex::geometry::PriorInstance> prior;
    prior.reserve(old_instances.size());
    for (const auto &oi : old_instances)
      prior.push_back({oi.instance_id, oi.semantic_class, oi.guid});

    auto reconcile = reusex::geometry::reconcile_instance_identities(
        old_labels.get(), *result.instance_labels, prior,
        result.instance_to_semantic, result.instance_sizes,
        []() { return reusex::core::generate_guid(); });

    // Save results
    spdlog::info("Saving instance labels as '{}'...", opt.output_cloud_name);
    db.save_point_cloud(opt.output_cloud_name, *result.instance_labels,
                        "segment_instances");

    // Persist stable instance identities (instances table).
    std::vector<reusex::ProjectDB::InstanceRecord> records;
    records.reserve(reconcile.instances.size());
    for (const auto &ri : reconcile.instances)
      records.push_back(
          {ri.instance_id, ri.guid, ri.semantic_class, ri.point_count});
    db.save_instances(opt.output_cloud_name, records);

    // Save metadata as label definitions
    // Format: instance_id -> "SM{semantic_class}-{instance_id} ({points}p)"
    std::map<int, std::string> label_definitions;
    for (const auto &[instance_id, semantic_class] :
         result.instance_to_semantic) {
      size_t size = result.instance_sizes.at(instance_id);
      label_definitions[static_cast<int>(instance_id)] =
          fmt::format("SM{}-{} ({}p)", semantic_class, instance_id, size);
    }
    db.save_label_definitions(opt.output_cloud_name, label_definitions);

    spdlog::info("Saved {} instance definitions", label_definitions.size());

    // --- Migrate material links across regeneration ---
    // Build a reverse map: old instance_id -> new instance_id (matched only).
    std::map<uint32_t, uint32_t> old_to_new;
    for (const auto &ri : reconcile.instances)
      if (ri.carried_over)
        old_to_new[ri.matched_old_id] = ri.instance_id;

    size_t links_carried = 0, links_dropped = 0;
    if (!old_links.empty()) {
      // The DELETE of instances rows above cascaded away the old
      // instance_materials rows; re-create them for matched instances.
      for (const auto &[old_id, material_guid] : old_links) {
        auto it = old_to_new.find(static_cast<uint32_t>(old_id));
        if (it != old_to_new.end()) {
          try {
            db.set_instance_material(opt.output_cloud_name,
                                     static_cast<int>(it->second),
                                     material_guid);
            ++links_carried;
          } catch (const std::exception &e) {
            spdlog::warn("Could not re-link material {} to instance {}: {}",
                         material_guid, it->second, e.what());
            ++links_dropped;
          }
        } else {
          ++links_dropped;
          spdlog::warn(
              "Dropping material link: old instance {} has no match in "
              "the regenerated cloud; passport {} kept but unlinked",
              old_id, material_guid);
        }
      }
    }

    // Loud reconciliation report (STANDARDS §3.2, §5).
    if (old_labels && !old_instances.empty()) {
      spdlog::info("Instance identity: {} carried over, {} fresh GUID(s)",
                   reconcile.matched_count, reconcile.fresh_count);
      if (!reconcile.orphaned_old.empty()) {
        spdlog::warn("{} previous instance(s) had no match in the regenerated "
                     "cloud:",
                     reconcile.orphaned_old.size());
        for (const auto &o : reconcile.orphaned_old)
          spdlog::warn("  - old instance {} (class {}, guid {}, best overlap "
                       "{:.0f}%)",
                       o.instance_id, o.semantic_class, o.guid,
                       o.best_overlap * 100.0);
      }
      if (!old_links.empty())
        spdlog::info("Material links: {} carried over, {} dropped",
                     links_carried, links_dropped);
    }

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
