// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "register.hpp"
#include "validation.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/geometry/registration/JointPairwiseRegistration.hpp>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

void setup_subcommand_register(CLI::App &app,
                               std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandRegisterOptions>();
  auto *sub = app.add_subcommand(
      "register", "Refine per-frame sensor poses (joint pairwise registration)");

  sub->footer(R"(
DESCRIPTION:
  Jointly refines the stored per-frame sensor poses within a single scan by
  minimizing point-to-plane distances between overlapping frames (Joint
  Pairwise Registration). Each frame's depth is back-projected to a surfel set
  seeded from its RTABMap world pose; all poses are then optimized together.

  A soft prior keeps the solution anchored to the original RTABMap poses, so
  the result refines rather than replaces them. Refined poses overwrite the
  stored transforms in place — re-import the RTABMap database to recover the
  originals, or use --dry-run to preview the residual first.

EXAMPLES:
  rux register                       # Defaults (Welsch kernel, 20 iters)
  rux register --dry-run             # Report before/after residual only
  rux register --iterations 40 --max-corr-distance 0.05
  rux register --kernel huber --prior-weight 2.0

WORKFLOW:
  1. rux import rtabmap scan.db      # Import sensor frames
  2. rux register                    # Refine poses (this command)
  3. rux create clouds               # Regenerate clouds with refined poses

NOTES:
  - Requires at least 2 sensor frames with depth (run 'rux import' first)
  - Run 'rux create clouds' afterwards to rebuild the merged point cloud
)");

  sub->add_option("--iterations", opt->iterations,
                  "Maximum Gauss-Newton outer iterations")
      ->default_val(opt->iterations);
  sub->add_option("--neighbor-window", opt->neighbor_window,
                  "Temporal pairing half-window (frames each side)")
      ->default_val(opt->neighbor_window);
  sub->add_option("--max-corr-distance", opt->max_corr_distance,
                  "Reject correspondences beyond this distance (m)")
      ->default_val(opt->max_corr_distance);
  sub->add_option("--normal-angle", opt->normal_angle,
                  "Reject correspondences whose normals disagree beyond (deg)")
      ->default_val(opt->normal_angle);
  sub->add_option("--robust-width", opt->robust_width,
                  "Robust kernel width (m): Welsch sigma / Huber delta")
      ->default_val(opt->robust_width);
  sub->add_option("--kernel", opt->kernel, "Robust kernel: welsch or huber")
      ->check(CLI::IsMember({"welsch", "huber"}))
      ->default_val(opt->kernel);
  sub->add_option("--prior-weight", opt->prior_weight,
                  "Soft prior pulling poses toward seeds (0 disables)")
      ->default_val(opt->prior_weight);
  sub->add_option("--anchor-frame", opt->anchor_frame,
                  "Frame index to hard-fix (-1 = none, rely on prior)")
      ->default_val(opt->anchor_frame);
  sub->add_option("--surfel-voxel", opt->surfel_voxel,
                  "Per-frame voxel downsample for surfels (m, <=0 disables)")
      ->default_val(opt->surfel_voxel);
  sub->add_option("--min-distance", opt->min_distance, "Minimum depth (m)")
      ->default_val(opt->min_distance);
  sub->add_option("--max-distance", opt->max_distance, "Maximum depth (m)")
      ->default_val(opt->max_distance);
  sub->add_option("--sampling-factor", opt->sampling_factor,
                  "Per-frame pixel subsampling factor")
      ->default_val(opt->sampling_factor);
  sub->add_option("--confidence", opt->confidence_threshold,
                  "Minimum confidence threshold")
      ->default_val(opt->confidence_threshold);
  sub->add_flag("--dry-run", opt->dry_run,
                "Compute and report residual without writing poses back");

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_register");
    return run_subcommand_register(*opt, *global_opt);
  });
}

int run_subcommand_register(SubcommandRegisterOptions const &opt,
                            const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;
  spdlog::info("Refining sensor poses in: {}", project_path.string());

  try {
    reusex::ProjectDB db(project_path);

    auto validation = rux::validation::validate_register_prerequisites(db);
    if (!validation) {
      spdlog::error("{}", validation.error_message);
      spdlog::info("Resolution: {}", validation.resolution_hint);
      return RuxError::INVALID_ARGUMENT;
    }

    reusex::geometry::JprParams params;
    params.max_iterations = opt.iterations;
    params.neighbor_window = opt.neighbor_window;
    params.max_corr_distance = opt.max_corr_distance;
    params.normal_angle_threshold = opt.normal_angle;
    params.robust_width = opt.robust_width;
    params.kernel = (opt.kernel == "huber")
                        ? reusex::geometry::JprParams::Kernel::huber
                        : reusex::geometry::JprParams::Kernel::welsch;
    params.prior_weight = opt.prior_weight;
    params.anchor_frame = opt.anchor_frame;
    params.surfel.min_distance = opt.min_distance;
    params.surfel.max_distance = opt.max_distance;
    params.surfel.sampling_factor = opt.sampling_factor;
    params.surfel.confidence_threshold = opt.confidence_threshold;
    params.surfel.voxel_size = opt.surfel_voxel;

    int logId = db.log_pipeline_start(
        "pose_refinement_jpr",
        fmt::format(
            R"({{"iterations":{},"neighbor_window":{},"max_corr_distance":{},"normal_angle":{},"robust_width":{},"kernel":"{}","prior_weight":{},"anchor_frame":{},"dry_run":{}}})",
            opt.iterations, opt.neighbor_window, opt.max_corr_distance,
            opt.normal_angle, opt.robust_width, opt.kernel, opt.prior_weight,
            opt.anchor_frame, opt.dry_run));

    auto result =
        reusex::geometry::refine_sensor_poses(db, params, opt.dry_run);

    db.log_pipeline_end(logId, true);

    spdlog::info("Pose refinement: {} frames, {} iterations, "
                 "point-to-plane RMS {:.5f} -> {:.5f} m",
                 result.frames, result.iterations, result.initial_rms,
                 result.final_rms);
    if (opt.dry_run)
      spdlog::info("Dry run: poses were not modified");
    else
      spdlog::info("Refined poses written. Run 'rux create clouds' to "
                   "regenerate the merged point cloud.");

    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Pose refinement failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
