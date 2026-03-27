// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "export/speckle.hpp"
#include "global-params.hpp"

#include <fmt/format.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <reusex/geometry/unweld.hpp>
#include <reusex/io/speckle.hpp>
#include <reusex/types.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cstdlib>
#include <memory>
#include <stdexcept>

namespace fs = std::filesystem;
using namespace ReUseX;

void setup_subcommand_export_speckle(CLI::App &parent) {
  auto opt = std::make_shared<SubcommandExportSpeckleOptions>();
  auto *sub = parent.add_subcommand(
      "speckle",
      "Export a point cloud or mesh to Speckle (app.speckle.systems).\n\n"
      "Supported formats:\n"
      "  Point clouds: .pcd\n"
      "  Meshes:       .ply, .obj, .stl, .vtk\n\n"
      "Authentication: Set SPECKLE_TOKEN environment variable.\n"
      "Find your token at: https://app.speckle.systems/profile\n\n"
      "How to find your Project ID:\n"
      "  1. Open your project in Speckle web UI\n"
      "  2. Copy the project ID from the URL: /projects/{PROJECT_ID}\n\n"
      "Examples:\n"
      "  export SPECKLE_TOKEN=your_token_here\n"
      "  rux export speckle cloud.pcd \\\n"
      "    -s https://app.speckle.systems \\\n"
      "    -p abc123def456 \\\n"
      "    -M \"Point cloud from ReUseX\"\n"
      "  rux export speckle mesh.ply \\\n"
      "    -s https://app.speckle.systems \\\n"
      "    -p abc123def456 \\\n"
      "    -M \"Mesh from ReUseX\"");

  sub->add_option("input", opt->input_path,
                  "Input file (PCD point cloud, or PLY/OBJ/STL/VTK mesh).")
      ->required()
      ->check(CLI::ExistingFile);

  sub->add_option("-s,--server", opt->server_url,
                  "Speckle server URL (e.g., https://app.speckle.systems)")
      ->required();

  sub->add_option("-p,--project-id", opt->project_id,
                  "Project/stream ID (find in URL: /projects/{PROJECT_ID})")
      ->required();

  sub->add_option("-m,--model", opt->model_name,
                  "Model/branch name (what users call 'model' in Speckle UI)")
      ->default_val(opt->model_name);

  sub->add_option("-M,--message", opt->commit_message, "Version commit message")
      ->default_val(opt->commit_message);

  sub->add_option("--max-batch", opt->max_batch_bytes,
                  "HTTP batch size in bytes (default: 25 MB)")
      ->default_val(opt->max_batch_bytes);

  sub->callback([opt]() {
    spdlog::trace("calling export speckle subcommand");
    return run_subcommand_export_speckle(*opt);
  });
}

int run_subcommand_export_speckle(SubcommandExportSpeckleOptions const &opt) {
  // 1. Validate environment
  const char *token_env = std::getenv("SPECKLE_TOKEN");
  if (!token_env || std::string(token_env).empty()) {
    spdlog::error("SPECKLE_TOKEN environment variable is not set.");
    spdlog::error("Please set your Speckle token:");
    spdlog::error("  export SPECKLE_TOKEN=your_token_here");
    spdlog::error("Find your token at: https://app.speckle.systems/profile");
    return RuxError::INVALID_ARGUMENT;
  }
  std::string token(token_env);

  // 2. Detect file type and load
  std::string ext = opt.input_path.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

  std::unique_ptr<io::speckle::Base> speckle_obj;

  if (ext == ".pcd") {
    // Point cloud
    spdlog::info("Loading point cloud from: {}", opt.input_path.string());
    CloudPtr cloud(new Cloud);
    if (pcl::io::loadPCDFile<PointT>(opt.input_path.c_str(), *cloud) < 0) {
      spdlog::error("Failed to load point cloud from: {}",
                    opt.input_path.string());
      return RuxError::INVALID_ARGUMENT;
    }
    if (cloud->empty()) {
      spdlog::error("Point cloud is empty");
      return RuxError::INVALID_ARGUMENT;
    }
    spdlog::info("Loaded {} points", cloud->size());
    speckle_obj =
        std::make_unique<io::speckle::Pointcloud>(io::speckle::to_speckle(cloud));

  } else if (ext == ".ply" || ext == ".obj" || ext == ".stl" || ext == ".vtk") {
    // Mesh
    spdlog::info("Loading mesh from: {}", opt.input_path.string());
    pcl::PolygonMesh mesh;
    int result = -1;
    if (ext == ".ply") {
      result = pcl::io::loadPLYFile(opt.input_path.string(), mesh);
    } else if (ext == ".obj") {
      result = pcl::io::loadOBJFile(opt.input_path.string(), mesh);
    } else if (ext == ".stl") {
      result = pcl::io::loadPolygonFileSTL(opt.input_path.string(), mesh);
    } else if (ext == ".vtk") {
      result = pcl::io::loadPolygonFileVTK(opt.input_path.string(), mesh);
    }
    if (result < 0) {
      spdlog::error("Failed to load mesh from: {}", opt.input_path.string());
      return RuxError::INVALID_ARGUMENT;
    }
    if (mesh.polygons.empty()) {
      spdlog::error("Mesh has no polygons");
      return RuxError::INVALID_ARGUMENT;
    }
    spdlog::info("Loaded {} polygons", mesh.polygons.size());

    // Unweld mesh vertices at sharp edges for correct shading
    auto unwelded = geometry::unweld_mesh(mesh, 0.0f);
    spdlog::info("Unwelded mesh: {} vertices",
                 unwelded->cloud.data.size() / unwelded->cloud.point_step);

    speckle_obj =
        std::make_unique<io::speckle::Mesh>(io::speckle::to_speckle(*unwelded));

  } else {
    spdlog::error("Unsupported file format: {}", ext);
    spdlog::error("Supported formats: .pcd (point cloud), .ply .obj .stl .vtk (mesh)");
    return RuxError::INVALID_ARGUMENT;
  }

  // 3. Create Speckle client
  spdlog::debug("Creating Speckle client for server: {}", opt.server_url);
  spdlog::debug("Project ID: {}", opt.project_id);
  io::speckle::SpeckleClient client(opt.server_url, opt.project_id, token);

  // 4. Configure batching
  client.set_max_batch_size(opt.max_batch_bytes);

  if (opt.max_batch_bytes < 1024 * 1024) {
    spdlog::warn("Very small batch size (< 1MB) may impact performance");
  }

  spdlog::debug("Batch size: {} bytes", opt.max_batch_bytes);

  // 5. Upload
  try {
    spdlog::info("Uploading to model '{}' with message: '{}'", opt.model_name,
                 opt.commit_message);

    std::string commit_id =
        client.upload(*speckle_obj, opt.model_name, opt.commit_message);

    spdlog::info("Upload successful!");
    spdlog::info("Commit ID: {}", commit_id);

    // Construct viewable URL
    std::string view_url =
        fmt::format("{}/projects/{}/models/{}", opt.server_url, opt.project_id,
                    opt.model_name);
    spdlog::info("View at: {}", view_url);

    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Upload failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
