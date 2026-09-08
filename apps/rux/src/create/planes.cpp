// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/planes.hpp"
#include "create/stage_bridge.hpp"
#include "exit_status.hpp"
#include "stage_prerequisites.hpp"
#include <reusex/core/ProjectDB.hpp>
#include <reusex/pipeline/stages.hpp>

#include <spdlog/spdlog.h>

namespace fs = std::filesystem;

/**
 * @brief Setup CLI options for the plane segmentation subcommand.
 *
 * Configures command-line arguments including cloud paths, thresholds,
 * and segmentation options for plane segmentation.
 *
 * @param app CLI application to add the subcommand to.
 */
void setup_subcommand_create_planes(CLI::App &app,
                                    std::shared_ptr<RuxOptions> global_opt) {

  auto opt = std::make_shared<SubcommandSegPlanesOptions>();
  auto *sub =
      app.add_subcommand("planes", "Detect and segment planar surfaces");

  sub->footer(R"(
DESCRIPTION:
  Detects and segments planar surfaces (walls, floors, ceilings) in 3D
  point clouds using multi-scale region growing. Identifies major structural
  planes and assigns a unique label to each detected surface. Essential
  preprocessing step for room segmentation and mesh generation.

EXAMPLES:
  rux create planes                    # Default settings
  rux create planes -a 15 -d 0.05      # Tighter angle/distance
  rux create planes -m 1000            # Larger minimum cluster
  rux create planes -r 0.15            # Larger search radius

WORKFLOW:
  1. rux import rtabmap scan.db        # Import sensor data
  2. rux create clouds                 # Reconstruct point clouds
  3. rux create planes                 # Segment planar surfaces
  4. rux create rooms                  # Segment rooms from planes
  5. rux view                          # Visualize results

NOTES:
  - Requires 'cloud' and 'normals' in project database
  - Angle threshold: surface normal similarity (degrees, default: 25°)
  - Distance threshold: max point-to-plane distance (meters, default: 0.1m)
  - Minimum cluster size affects noise filtering (default: ~2 sqm @ 2cm res)
  - Output: 'planes', 'plane_centroids', 'plane_normals' saved to project
  - Filter syntax supports expressions like 'rooms == 3'
)");

  sub->get_formatter()->column_width(40);

  sub->add_option("-a, --angle-threshold", opt->angle_threshold,
                  "Angle threshold for plane fitting "
                  "(default: 25° or cos(25°) = 0.96592583)")
      ->default_val(opt->angle_threshold)
      ->check(CLI::Range(0.0, 365.0));

  auto *dist_opt =
      sub->add_option("-d, --plane-dist-threshold", opt->plane_dist_threshold,
                      "Distance threshold for plane fitting [m]. When set "
                      "explicitly, bypasses adaptive derivation for this "
                      "parameter (default: adaptive ~3*sigma).")
          ->default_val(opt->plane_dist_threshold)
          ->check(CLI::Range(0.0, 1.0));

  auto *min_opt =
      sub->add_option("-m, --min-cluster-size", opt->minInliers,
                      "Minimum cluster size for plane fitting. When set "
                      "explicitly, bypasses adaptive derivation for this "
                      "parameter (default: adaptive by density).")
          ->default_val(opt->minInliers)
          ->check(CLI::Range(3, 1000000));

  sub->add_flag("--adaptive,!--no-adaptive", opt->adaptive,
                "Derive plane_dist_threshold (~3*sigma) and min_inliers (by "
                "point density) from measured cloud noise (default: on). "
                "Explicit -d/-m still win per-parameter.");

  sub->add_option("--noise-seed", opt->noise_seed,
                  "Deterministic seed for the noise estimator")
      ->default_val(opt->noise_seed);

  sub->add_option("-r, --radius", opt->radius, "Radius for region growing")
      ->default_val(opt->radius)
      ->check(CLI::Range(0.0, 5.0));

  sub->add_option("-i, --interval-0", opt->interval_0,
                  "Initial interval for plane update")
      ->default_val(opt->interval_0)
      ->check(CLI::Range(1.0, 10000.0));

  sub->add_option("--interval-factor", opt->interval_factor,
                  "Factor for interval update")
      ->default_val(opt->interval_factor)
      ->check(CLI::Range(1.0, 10.0));

  sub->add_option(
         "-f, --filter", opt->filter_expr,
         "Filter expression to limit processing to specific labeled points.\n"
         "Syntax: <cloud_name> <op> <value(s)>\n"
         "Examples:\n"
         "  -f 'planes in [1, 2, 5]'        # Filter to labels 1, 2, 5 from "
         "planes cloud\n"
         "  -f 'rooms == 3'                 # Only process room 3\n"
         "  -f 'planes in [1,2] || rooms == 5'  # Combine multiple clouds\n"
         "  -f 'planes >= 10 && planes <= 20'   # Range filter")
      ->default_val("");

  sub->callback([opt, global_opt, dist_opt, min_opt]() {
    spdlog::trace("calling seg-planes subcommand");
    // Record which thresholds the user pinned so adaptivity is bypassed only
    // for those (issue #214).
    opt->dist_explicit = dist_opt->count() > 0;
    opt->min_explicit = min_opt->count() > 0;
    rux::finish(run_subcommand_segment_planes(*opt, *global_opt));
  });
}

/**
 * @brief Execute plane segmentation on a point cloud.
 *
 * Validates the CLI's prerequisites, then hands the run to
 * reusex::pipeline::run_stage, which owns loading, segmenting, saving and the
 * pipeline_log record (#284).
 *
 * @param opt Options containing project path and segmentation parameters.
 * @return Exit code (RuxError::SUCCESS on success).
 */
int run_subcommand_segment_planes(SubcommandSegPlanesOptions const &opt,
                                  const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;
  spdlog::info("Segmenting planes in project: {}", project_path.string());

  try {
    reusex::ProjectDB db(project_path);

    // Pre-flight validation: check for cloud and normals
    if (int rc = rux::check_stage_prerequisites(
            db, reusex::core::PipelineStage::planes);
        rc != RuxError::SUCCESS)
      return rc;

    reusex::pipeline::StageContext ctx;
    ctx.project = project_path;
    ctx.stage = reusex::pipeline::JobStage::planes;
    // The noise-adaptive thresholds of #214 are pinned by *presence*: emitting
    // `plane_dist_threshold` / `min_inliers` only when the user actually passed
    // -d/-m is what tells the stage to bypass adaptive derivation for exactly
    // those parameters.
    ctx.parameters =
        rux::StageParams()
            .set("angle_threshold", opt.angle_threshold)
            .set_if(opt.dist_explicit, "plane_dist_threshold",
                    opt.plane_dist_threshold)
            .set_if(opt.min_explicit, "min_inliers", opt.minInliers)
            .set("radius", opt.radius)
            .set("interval_0", opt.interval_0)
            .set("interval_factor", opt.interval_factor)
            .set("adaptive", opt.adaptive)
            .set("noise_seed", opt.noise_seed)
            .set_if(!opt.filter_expr.empty(), "filter", opt.filter_expr)
            .dump();

    const auto result = reusex::pipeline::run_stage(db, ctx);
    if (result.ok)
      spdlog::info("Plane segmentation complete: {}", result.message);
    return rux::exit_code_for(result);

  } catch (const std::exception &e) {
    spdlog::error("Plane segmentation failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
