// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "optimize.hpp"
#include "validation.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/geometry/registration/PlaneGraphOptimizer.hpp>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

void setup_subcommand_optimize(CLI::App &app,
                               std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandOptimizeOptions>();
  auto *sub = app.add_subcommand(
      "optimize",
      "Globally optimize per-frame poses via a plane-landmark pose graph");

  sub->footer(R"(
DESCRIPTION:
  Globally optimizes the stored per-frame sensor poses of a single scan using a
  plane-landmark pose graph (issue #225, P1). Unlike 'rux register' — which
  polishes poses locally with frame-to-frame point-to-plane residuals and
  provably saturates because pairwise energy cannot express global consistency —
  this back-end changes the objective to *shared global plane landmarks*:

    1. detect the dominant planes each frame sees (RANSAC on its surfels),
    2. associate those observations across frames into persistent
       OrientedPlane3 landmarks (normal-angle + offset gating, min observations),
    3. build a GTSAM factor graph (per-frame Pose3 + odometry BetweenFactors from
       the seed poses + one OrientedPlane3Factor per observation + a soft prior
       on the first pose for gauge), and
    4. solve it with Graduated Non-Convexity (GNC) so a handful of bad plane
       associations are down-weighted rather than corrupting the solution.

  Optimized poses overwrite the stored transforms in place — re-import the
  RTABMap database to recover the originals, or use --dry-run to preview.

EXAMPLES:
  rux optimize                        # Defaults (GNC on, 100 LM iters)
  rux optimize --dry-run              # Report statistics without writing
  rux optimize --min-observations 3 --assoc-distance 0.15
  rux optimize --no-gnc               # Plain Levenberg-Marquardt (no robustness)

WORKFLOW:
  1. rux import rtabmap scan.db       # Import sensor frames
  2. rux optimize                     # Global pose optimization (this command)
  3. rux create clouds                # Regenerate clouds with optimized poses

NOTES:
  - Requires at least 2 sensor frames with depth (run 'rux import' first)
  - Run 'rux create clouds' afterwards to rebuild the merged point cloud
  - When too few plane landmarks are found the poses are left unchanged
)");

  // --- Per-frame plane detection ---
  sub->add_option("--max-planes-per-frame", opt->max_planes_per_frame,
                  "Keep at most this many planes per frame")
      ->default_val(opt->max_planes_per_frame);
  sub->add_option("--min-plane-inliers", opt->min_plane_inliers,
                  "Reject planes with fewer supporting surfels")
      ->default_val(opt->min_plane_inliers);
  sub->add_option("--ransac-distance", opt->ransac_distance,
                  "Inlier point-to-plane distance (m)")
      ->default_val(opt->ransac_distance);
  sub->add_option("--ransac-normal-angle", opt->ransac_normal_angle,
                  "Inlier normal agreement threshold (deg)")
      ->default_val(opt->ransac_normal_angle);
  sub->add_option("--ransac-iterations", opt->ransac_iterations,
                  "RANSAC hypotheses per plane")
      ->default_val(opt->ransac_iterations);

  // --- Cross-frame association ---
  sub->add_option(
         "--assoc-normal-angle", opt->assoc_normal_angle,
         "Max normal angle to merge observations into a landmark (deg)")
      ->default_val(opt->assoc_normal_angle);
  sub->add_option("--assoc-distance", opt->assoc_distance,
                  "Max plane-offset difference to merge (m)")
      ->default_val(opt->assoc_distance);
  sub->add_option("--min-observations", opt->min_observations,
                  "Landmark must be seen by at least this many frames")
      ->default_val(opt->min_observations);

  // --- Factor-graph noise ---
  sub->add_option("--odometry-sigma-rot", opt->odometry_sigma_rot,
                  "Odometry rotation std (rad)")
      ->default_val(opt->odometry_sigma_rot);
  sub->add_option("--odometry-sigma-trans", opt->odometry_sigma_trans,
                  "Odometry translation std (m)")
      ->default_val(opt->odometry_sigma_trans);
  sub->add_option("--plane-sigma-normal", opt->plane_sigma_normal,
                  "Plane-normal measurement std (rad)")
      ->default_val(opt->plane_sigma_normal);
  sub->add_option("--plane-sigma-distance", opt->plane_sigma_distance,
                  "Plane-distance measurement std (m)")
      ->default_val(opt->plane_sigma_distance);
  sub->add_option("--prior-sigma-rot", opt->prior_sigma_rot,
                  "First-pose gauge prior rotation std (rad)")
      ->default_val(opt->prior_sigma_rot);
  sub->add_option("--prior-sigma-trans", opt->prior_sigma_trans,
                  "First-pose gauge prior translation std (m)")
      ->default_val(opt->prior_sigma_trans);

  // --- Solver ---
  sub->add_flag("--no-gnc", opt->no_gnc,
                "Disable Graduated Non-Convexity (use plain LM)");
  sub->add_option("--gnc-inlier-cost", opt->gnc_inlier_cost,
                  "GNC TLS inlier threshold on factor error (0.5*chi^2)")
      ->default_val(opt->gnc_inlier_cost);
  sub->add_option("--iterations", opt->iterations,
                  "Levenberg-Marquardt (inner) iteration cap")
      ->default_val(opt->iterations);
  sub->add_option("--seed", opt->seed, "RANSAC seed (determinism)")
      ->default_val(opt->seed);

  // --- Surfel extraction ---
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
                "Compute and report statistics without writing poses back");

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_optimize");
    return run_subcommand_optimize(*opt, *global_opt);
  });
}

int run_subcommand_optimize(SubcommandOptimizeOptions const &opt,
                            const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;
  spdlog::info("Optimizing sensor poses in: {}", project_path.string());

  try {
    reusex::ProjectDB db(project_path);

    // Reuse the register prerequisites (>= 2 sensor frames carrying depth).
    auto validation = rux::validation::validate_register_prerequisites(db);
    if (!validation) {
      spdlog::error("{}", validation.error_message);
      spdlog::info("Resolution: {}", validation.resolution_hint);
      return RuxError::INVALID_ARGUMENT;
    }

    reusex::geometry::PlaneGraphOptions options;
    options.max_planes_per_frame = opt.max_planes_per_frame;
    options.min_plane_inliers = opt.min_plane_inliers;
    options.ransac_distance = opt.ransac_distance;
    options.ransac_normal_angle = opt.ransac_normal_angle;
    options.ransac_iterations = opt.ransac_iterations;
    options.assoc_normal_angle = opt.assoc_normal_angle;
    options.assoc_distance = opt.assoc_distance;
    options.min_landmark_observations = opt.min_observations;
    options.odometry_sigma_rot = opt.odometry_sigma_rot;
    options.odometry_sigma_trans = opt.odometry_sigma_trans;
    options.plane_sigma_normal = opt.plane_sigma_normal;
    options.plane_sigma_distance = opt.plane_sigma_distance;
    options.prior_sigma_rot = opt.prior_sigma_rot;
    options.prior_sigma_trans = opt.prior_sigma_trans;
    options.use_gnc = !opt.no_gnc;
    options.gnc_inlier_cost = opt.gnc_inlier_cost;
    options.max_iterations = opt.iterations;
    options.seed = opt.seed;
    options.surfel.min_distance = opt.min_distance;
    options.surfel.max_distance = opt.max_distance;
    options.surfel.sampling_factor = opt.sampling_factor;
    options.surfel.confidence_threshold = opt.confidence_threshold;
    options.surfel.voxel_size = opt.surfel_voxel;

    int logId = db.log_pipeline_start(
        "pose_optimization_plane_graph",
        fmt::format(
            R"({{"min_observations":{},"assoc_normal_angle":{},"assoc_distance":{},"max_planes_per_frame":{},"min_plane_inliers":{},"use_gnc":{},"iterations":{},"seed":{},"dry_run":{}}})",
            opt.min_observations, opt.assoc_normal_angle, opt.assoc_distance,
            opt.max_planes_per_frame, opt.min_plane_inliers, !opt.no_gnc,
            opt.iterations, opt.seed, opt.dry_run));

    auto result =
        reusex::geometry::optimize_sensor_poses(db, options, opt.dry_run);

    db.log_pipeline_end(logId, true);

    spdlog::info("Plane-graph optimization: {} frames, {} plane detections, "
                 "{} landmarks, {} plane factors",
                 result.frames, result.planes_detected, result.landmarks,
                 result.plane_factors);
    spdlog::info("Factor-graph error {:.4f} -> {:.4f}, max pose shift {:.4f} m",
                 result.initial_error, result.final_error,
                 result.max_pose_shift);

    if (result.landmarks == 0) {
      spdlog::warn("No plane landmarks reached the minimum observation count; "
                   "poses were left unchanged. Try lowering --min-observations "
                   "or relaxing --assoc-distance / --assoc-normal-angle.");
      return RuxError::SUCCESS;
    }

    if (!result.converged) {
      spdlog::error("Factor-graph optimizer failed to produce a solution; "
                    "poses were left unchanged (see log for details)");
      return RuxError::GENERIC;
    }

    if (opt.dry_run)
      spdlog::info("Dry run: poses were not modified");
    else
      spdlog::info("Optimized poses written. Run 'rux create clouds' to "
                   "regenerate the merged point cloud.");

    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Pose optimization failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
