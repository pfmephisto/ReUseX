// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/texture.hpp"
#include "validation.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/geometry/texture_mesh.hpp>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <fmt/format.h>

#include <Eigen/Dense>

void setup_subcommand_create_texture(CLI::App &app, std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandTextureOptions>();
  auto *sub = app.add_subcommand(
      "texture",
      "Apply textures to mesh from sensor frames");

  sub->footer(R"(
DESCRIPTION:
  Maps RGB textures from sensor frame images onto 3D mesh surfaces using
  camera poses and intrinsics. Projects mesh vertices into image space,
  selects best-view images per face, and generates UV-mapped textured mesh
  suitable for photorealistic visualization and CAD export.

EXAMPLES:
  rux create texture                   # Default: mesh -> mesh_textured
  rux create texture -m room1 -o room1_tex  # Custom names
  rux -p scan.rux create texture       # Custom project path

WORKFLOW:
  1. rux import rtabmap scan.db        # Import sensor frames with RGB
  2. rux create clouds                 # Reconstruct geometry
  3. rux create planes && rooms        # Segment architecture
  4. rux create mesh                   # Generate 3D mesh
  5. rux create texture                # Apply textures to mesh
  6. rux export rhino textured.3dm     # Export textured model

NOTES:
  - Requires mesh and sensor frames with RGB images in project database
  - Best-view selection based on viewing angle and distance
  - UV coordinates automatically generated for texture mapping
  - Output mesh includes embedded texture atlas
  - Default input: 'mesh', default output: 'mesh_textured'
)");

  sub->add_option("-m, --mesh-name", opt->mesh_name,
                  "Name of the input mesh in ProjectDB")
      ->default_val(opt->mesh_name);

  sub->add_option("-o, --output-name", opt->output_name,
                  "Name for the output textured mesh in ProjectDB")
      ->default_val(opt->output_name);

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_texture");
    return run_subcommand_texture(*opt, *global_opt);
  });
}
int run_subcommand_texture(SubcommandTextureOptions const &opt, const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;
  spdlog::info("Texturing mesh from project: {}", project_path.string());

  try {
    reusex::ProjectDB db(project_path);

    // Pre-flight validation: check for mesh and sensor frames
    auto validation = rux::validation::validate_texture_prerequisites(db);
    if (!validation) {
      spdlog::error("{}", validation.error_message);
      spdlog::info("Resolution: {}", validation.resolution_hint);
      return RuxError::INVALID_ARGUMENT;
    }

    int logId = db.log_pipeline_start("texture_mesh",
        fmt::format(R"({{"mesh_name":"{}","output_name":"{}"}})",
                    opt.mesh_name, opt.output_name));

    spdlog::trace("Loading mesh '{}' from ProjectDB", opt.mesh_name);
    auto mesh = db.mesh(opt.mesh_name);

    spdlog::trace("Loading sensor frames from ProjectDB");
    auto frame_ids = db.sensor_frame_ids();
    spdlog::info("Found {} sensor frames for texturing", frame_ids.size());

    // Build camera data map from ProjectDB
    std::map<int, reusex::geometry::CameraData> cameras;
    for (int nodeId : frame_ids) {
      reusex::geometry::CameraData cam_data;

      cam_data.image = db.sensor_frame_image(nodeId);
      if (cam_data.image.empty()) {
        spdlog::warn("Skipping node {} - empty image", nodeId);
        continue;
      }

      cam_data.intrinsics = db.sensor_frame_intrinsics(nodeId);

      // Get pose as 4x4 SE(3) matrix
      auto pose_array = db.sensor_frame_pose(nodeId);
      for (int i = 0; i < 16; ++i) {
        cam_data.pose(i / 4, i % 4) = pose_array[i];
      }

      cameras[nodeId] = std::move(cam_data);
    }

    spdlog::info("Loaded {} camera frames for texturing", cameras.size());

    spdlog::trace("Generating textured mesh");
    auto textured_mesh = reusex::geometry::texture_mesh(mesh, cameras);

    spdlog::trace("Saving textured mesh to ProjectDB as '{}'", opt.output_name);
    db.save_mesh(opt.output_name, *textured_mesh, "texture_mesh");

    spdlog::info("Textured mesh '{}' saved to ProjectDB", opt.output_name);

    db.log_pipeline_end(logId, true);
    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Mesh texturing failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
