// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "export/colmap.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/colmap.hpp>

#include <spdlog/spdlog.h>

void setup_subcommand_export_colmap(CLI::App &parent,
                                    std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandExportColmapOptions>();
  auto *sub = parent.add_subcommand(
      "colmap", "Export sensor frames as a COLMAP sparse model");

  sub->footer(R"(
DESCRIPTION:
  Writes a COLMAP sparse model directory from the project's sensor frames.
  Camera poses are taken directly from SLAM (no SfM is run). The output is a
  ready-to-use input for COLMAP's dense reconstruction (patch_match_stereo +
  stereo_fusion) and for Gaussian Splatting trainers (gsplat, nerfstudio,
  Brush, original Inria 3DGS) which all consume COLMAP format natively.

OUTPUT LAYOUT:
  <out_dir>/
    images/<frame_id>.jpg
    sparse/0/cameras.txt
    sparse/0/images.txt
    sparse/0/points3D.txt   (optionally seeded from LiDAR cloud)

EXAMPLES:
  rux export colmap ./scene                       # Default seed from 'cloud'
  rux export colmap ./scene --no-lidar-seed       # Empty points3D
  rux export colmap ./scene --max-lidar 50000     # Smaller seed
  rux export colmap ./scene --cloud filtered      # Use different seed cloud

NOTES:
  - Requires sensor frames in the project (run 'rux import rtabmap' first).
  - Identical intrinsics across frames are merged into a single PINHOLE
    camera entry. Distortion is not modelled (intrinsics are already pinhole).
  - The LiDAR seed accelerates PatchMatch convergence and gives 3DGS a
    head start; subsample is uniform across the cloud.
)");

  sub->add_option("output_dir", opt->output_dir, "COLMAP scene output directory")
      ->required();

  sub->add_flag("--no-lidar-seed", opt->no_lidar_seed,
                "Do not seed points3D.txt from the LiDAR point cloud");

  sub->add_option("--max-lidar", opt->max_lidar_points,
                  "Maximum number of LiDAR points to include as seed")
      ->default_val(opt->max_lidar_points);

  sub->add_option("--cloud", opt->lidar_cloud_name,
                  "Name of the point cloud in ProjectDB to use as seed")
      ->default_val(opt->lidar_cloud_name);

  sub->add_option("--jpeg-quality", opt->jpeg_quality,
                  "JPEG quality for exported images (1..100)")
      ->default_val(opt->jpeg_quality)
      ->check(CLI::Range(1, 100));

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_export_colmap");
    return run_subcommand_export_colmap(*opt, *global_opt);
  });
}

int run_subcommand_export_colmap(SubcommandExportColmapOptions const &opt,
                                 const RuxOptions &global_opt) {
  try {
    spdlog::info("Exporting COLMAP scene from project: {}",
                 global_opt.project_db.string());

    // Open writable so any pending schema migrations run before we read.
    reusex::ProjectDB db(global_opt.project_db);

    reusex::io::ColmapExportOptions cmopt;
    cmopt.include_lidar_points = !opt.no_lidar_seed;
    cmopt.max_lidar_points = opt.max_lidar_points;
    cmopt.lidar_cloud_name = opt.lidar_cloud_name;
    cmopt.jpeg_quality = opt.jpeg_quality;

    reusex::io::export_colmap_scene(db, opt.output_dir, cmopt);

    spdlog::info("COLMAP scene written to {}", opt.output_dir.string());
    return RuxError::SUCCESS;
  } catch (const std::exception &e) {
    spdlog::error("COLMAP export failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
