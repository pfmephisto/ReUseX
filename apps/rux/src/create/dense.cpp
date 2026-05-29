// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/dense.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/geometry/densify.hpp>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

void setup_subcommand_create_dense(CLI::App &app,
                                   std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandCreateDenseOptions>();
  auto *sub = app.add_subcommand(
      "dense", "Generate a dense point cloud via OpenMVS Multi-View Stereo");

  sub->footer(R"(
DESCRIPTION:
  Runs OpenMVS PatchMatch Multi-View Stereo on the project's sensor frames
  to produce a dense point cloud that extends beyond the LiDAR depth range.
  Uses SLAM-supplied poses (no SfM step). OpenMVS is linked directly into
  libreusex — no subprocess, no external workspace files.

PIPELINE (all in-process):
  1. Build MVS::Scene from sensor frames (Platform per intrinsics group,
     Pose per frame, optional LiDAR seed point cloud).
  2. Scene::SelectNeighborViews()
  3. Scene::DenseReconstruction()
  4. Convert resulting MVS::PointCloud → pcl::PointCloud<PointXYZRGB>
  5. Save under --output (default 'dense') alongside the LiDAR cloud.

EXAMPLES:
  rux create dense                               # default settings
  rux create dense --max-resolution 1280         # faster, lower-res depth
  rux create dense --no-geom-consistency         # quick first-pass quality
  rux create dense --frame-stride 2              # skip every other frame
  rux create dense --no-lidar-seed               # no seed; pure MVS

RAM USAGE:
  Peak memory is dominated by the fusion stage, which loads ALL depth +
  normal + confidence maps simultaneously AND aggregates one candidate
  per pixel per image into a single working set before clustering.
  Practically that's ~N_frames * W * H * ~60 bytes; for 238 frames at
  640×480 that exceeds 16 GB. Fusion is sequential, so --max-threads
  does NOT help here.

  Memory-bounded path (recommended on OOM):
    --chunk-size N        process N frames per dense-reconstruction call,
                          then merge per-chunk point clouds. THIS is the
                          only way to keep full image resolution without
                          OOM. 640×480 indoor: start 30–80; halve again
                          if a single chunk still OOMs.

  Other levers, lower impact:
    --max-resolution N    cap on max(width, height); halves dmap memory
    --frame-stride N      halves/thirds frame count
    --max-fuse-points N   caps total output size (truncates fusion result)
    --max-threads N       bounds *peak* RAM during depth-map estimation
                          (each PatchMatch worker holds a pyramid)
    --no-geom-consistency skip the geometric pass (saves a depth-map set)

NOTES:
  - JPEGs are decoded once into a temp directory because OpenMVS loads
    pixels through OpenCV by path. The temp directory is cleaned up on
    success.
  - The LiDAR cloud stays put under its current name; choose which to
    consume downstream by passing --cloud to other commands.
  - OpenMVS is AGPL-3.0-or-later — combining it with libreusex makes the
    combined binary effectively AGPL.
)");

  sub->add_option("--output", opt->output_name,
                  "Cloud name to save in ProjectDB")
      ->default_val(opt->output_name);

  sub->add_option("--cloud", opt->seed_cloud_name,
                  "Name of the LiDAR cloud used to seed PatchMatch")
      ->default_val(opt->seed_cloud_name);

  sub->add_option("--seed-max", opt->seed_max_points,
                  "Cap on the number of LiDAR seed points (uniform stride)")
      ->default_val(opt->seed_max_points);

  sub->add_option("--resolution-level", opt->resolution_level,
                  "OPTDENSE.nResolutionLevel: 0=full, 1=half, 2=quarter, ...")
      ->default_val(opt->resolution_level);

  sub->add_option("--max-resolution", opt->max_resolution,
                  "Upper bound on max(width,height) of input images")
      ->default_val(opt->max_resolution);

  sub->add_option("--min-resolution", opt->min_resolution,
                  "Lower bound on max(width,height) of input images")
      ->default_val(opt->min_resolution);

  sub->add_option("--num-views", opt->num_views,
                  "Neighbor views per image during depth estimation")
      ->default_val(opt->num_views);

  sub->add_option("--frame-stride", opt->frame_stride,
                  "Use every Nth sensor frame (1 = all)")
      ->default_val(opt->frame_stride)
      ->check(CLI::PositiveNumber);

  sub->add_option("--gpu-index", opt->gpu_index,
                  "CUDA device for PatchMatch (-2=CPU, -1=best GPU, "
                  ">=0=device index). Default -2 because CUDA init is "
                  "unreliable in some Nix envs; pass 0 on a working GPU.")
      ->default_val(opt->gpu_index);

  sub->add_option("--max-threads", opt->max_threads,
                  "Maximum concurrent OpenMP threads for depth-map "
                  "estimation. 0 = all cores. Lower to bound peak RAM "
                  "(each worker holds an image pyramid + buffers).")
      ->default_val(opt->max_threads)
      ->check(CLI::NonNegativeNumber);

  sub->add_option(
         "--chunk-size", opt->chunk_size,
         "Process at most N frames per dense-reconstruction call, then "
         "merge the resulting point clouds. The only way to bound peak "
         "fusion RAM without lowering image resolution. 0 disables. For "
         "640×480 indoor scans, start with 30–80; lower if a single "
         "chunk still OOMs.")
      ->default_val(opt->chunk_size)
      ->check(CLI::NonNegativeNumber);

  sub->add_option("--max-fuse-points", opt->max_fuse_points,
                  "Cap on the number of points kept after fusion "
                  "(OPTDENSE.nMaxPointsFuse). 0 = unlimited.")
      ->default_val(opt->max_fuse_points)
      ->check(CLI::NonNegativeNumber);

  sub->add_flag("--no-geom-consistency", opt->no_geom_consistency,
                "Disable geometric consistency between neighbor depth maps");
  sub->add_flag("--no-lidar-seed", opt->no_lidar_seed,
                "Do not seed PatchMatch from the LiDAR point cloud");
  sub->add_flag("--no-outlier-filter", opt->no_outlier_filter,
                "Skip the post-fusion statistical outlier removal pass.");
  sub->add_flag("--no-bbox-clip", opt->no_bbox_clip,
                "Skip clipping dense output to the LiDAR cloud's "
                "percentile bbox (expanded by 25%). The bbox clip is "
                "the most effective filter against MVS triangulation "
                "patches that land far outside the real scene.");

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_create_dense");
    return run_subcommand_create_dense(*opt, *global_opt);
  });
}

int run_subcommand_create_dense(SubcommandCreateDenseOptions const &opt,
                                const RuxOptions &global_opt) {
  try {
    spdlog::info("Running OpenMVS densify on project: {}",
                 global_opt.project_db.string());

    reusex::ProjectDB db(global_opt.project_db);

    reusex::geometry::DensifyParams params;
    params.output_name = opt.output_name;
    params.seed_cloud_name = opt.no_lidar_seed ? std::string{} : opt.seed_cloud_name;
    params.seed_max_points = opt.seed_max_points;
    params.resolution_level = opt.resolution_level;
    params.max_resolution = opt.max_resolution;
    params.min_resolution = opt.min_resolution;
    params.num_views = opt.num_views;
    params.frame_stride = opt.frame_stride;
    params.gpu_index = opt.gpu_index;
    params.max_threads = opt.max_threads;
    params.chunk_size = opt.chunk_size;
    params.max_fuse_points = opt.max_fuse_points;
    params.geometric_consistency = !opt.no_geom_consistency;
    params.outlier_filter = !opt.no_outlier_filter;
    params.bbox_clip = !opt.no_bbox_clip;

    int logId = db.log_pipeline_start(
        "create_dense",
        fmt::format(R"({{"resolution_level":{},"max_resolution":{},"geom":{},"stride":{}}})",
                    params.resolution_level, params.max_resolution,
                    params.geometric_consistency, params.frame_stride));
    try {
      reusex::geometry::densify_from_images(db, params);
      db.log_pipeline_end(logId, true);
    } catch (...) {
      db.log_pipeline_end(logId, false, "densify failed");
      throw;
    }

    spdlog::info("Dense reconstruction complete; saved as '{}'",
                 opt.output_name);
    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Dense reconstruction failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
