// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "texture.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/geometry/texture_mesh.hpp>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <fmt/format.h>

#include <Eigen/Dense>

void setup_subcommand_texture(CLI::App &app) {
  auto opt = std::make_shared<SubcommandTextureOptions>();
  auto *sub = app.add_subcommand(
      "texture",
      "Apply textures to a mesh using sensor frames from ProjectDB.");

  sub->add_option("project", opt->project,
                  "Path to the .rux project file.")
      ->required()
      ->check(CLI::ExistingFile);

  sub->add_option("-m, --mesh-name", opt->mesh_name,
                  "Name of the input mesh in ProjectDB")
      ->default_val(opt->mesh_name);

  sub->add_option("-o, --output-name", opt->output_name,
                  "Name for the output textured mesh in ProjectDB")
      ->default_val(opt->output_name);

  sub->callback([opt]() {
    spdlog::trace("calling run_subcommand_texture");
    return run_subcommand_texture(*opt);
  });
}
int run_subcommand_texture(SubcommandTextureOptions const &opt) {
  spdlog::info("Texturing mesh from project: {}", opt.project.string());

  try {
    ReUseX::ProjectDB db(opt.project);

    int logId = db.log_pipeline_start("texture_mesh",
        fmt::format(R"({{"mesh_name":"{}","output_name":"{}"}})",
                    opt.mesh_name, opt.output_name));

    spdlog::trace("Loading mesh '{}' from ProjectDB", opt.mesh_name);
    auto mesh = db.mesh(opt.mesh_name);

    spdlog::trace("Loading sensor frames from ProjectDB");
    auto frame_ids = db.sensor_frame_ids();
    spdlog::info("Found {} sensor frames for texturing", frame_ids.size());

    // Build camera data map from ProjectDB
    std::map<int, ReUseX::geometry::CameraData> cameras;
    for (int nodeId : frame_ids) {
      ReUseX::geometry::CameraData cam_data;

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
    auto textured_mesh = ReUseX::geometry::texture_mesh(mesh, cameras);

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
