// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "export/speckle.hpp"
#include "global-params.hpp"

#include <fmt/format.h>
#include <pcl/PolygonMesh.h>
#include <reusex/core/ProjectDB.hpp>
#include <reusex/geometry/unweld.hpp>
#include <reusex/io/speckle.hpp>
#include <reusex/types.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cstdlib>
#include <memory>
#include <stdexcept>

namespace fs = std::filesystem;
using namespace reusex;

void setup_subcommand_export_speckle(CLI::App &parent,
                                     std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandExportSpeckleOptions>();
  auto *sub =
      parent.add_subcommand("speckle", "Export point cloud or mesh to Speckle");

  sub->footer(R"(
DESCRIPTION:
  Uploads point clouds or meshes from ReUseX ProjectDB to Speckle platform
  for web-based 3D visualization, collaboration, and BIM integration.
  Supports both cloud and mesh data with metadata preservation.

AUTHENTICATION:
  Set SPECKLE_TOKEN environment variable with your personal access token.
  Get token at: https://app.speckle.systems/profile

PROJECT ID:
  Find in Speckle web UI URL: /projects/{PROJECT_ID}

EXAMPLES:
  export SPECKLE_TOKEN=your_token_here
  rux export speckle \
    -s https://app.speckle.systems \
    -p abc123def456 \
    -d cloud \
    -M "Point cloud from ReUseX"

  rux export speckle \
    -s https://app.speckle.systems \
    -p abc123def456 \
    -d mesh --mesh \
    -M "Textured mesh export"

WORKFLOW:
  1. Create Speckle project at app.speckle.systems
  2. Copy project ID from URL
  3. export SPECKLE_TOKEN=your_token
  4. rux export speckle -p PROJECT_ID -d data_name

NOTES:
  - Requires network connection to Speckle server
  - Default server: https://app.speckle.systems
  - Supports point clouds (XYZRGB) and polygon meshes
  - Use -M/--message to annotate commit with description
  - Data name must exist in ProjectDB (e.g., 'cloud', 'mesh')
)");

  sub->add_option("-d,--data-name", opt->data_name,
                  "Name of the cloud or mesh in ProjectDB")
      ->default_val(opt->data_name);

  sub->add_flag("--mesh", opt->is_mesh, "Load as mesh instead of point cloud")
      ->default_val(opt->is_mesh);

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

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling export speckle subcommand");
    return run_subcommand_export_speckle(*opt, *global_opt);
  });
}

int run_subcommand_export_speckle(SubcommandExportSpeckleOptions const &opt,
                                  const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;
  spdlog::info("Exporting to Speckle from project: {}", project_path.string());

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

  try {
    // 2. Load data from ProjectDB
    ProjectDB db(project_path);

    std::unique_ptr<io::speckle::Base> speckle_obj;

    if (!opt.is_mesh) {
      // Point cloud
      spdlog::info("Loading point cloud '{}' from ProjectDB", opt.data_name);
      CloudPtr cloud = db.point_cloud_xyzrgb(opt.data_name);

      if (cloud->empty()) {
        spdlog::error("Point cloud is empty");
        return RuxError::INVALID_ARGUMENT;
      }
      spdlog::info("Loaded {} points", cloud->size());
      speckle_obj = std::make_unique<io::speckle::Pointcloud>(
          io::speckle::to_speckle(cloud));

    } else {
      // Mesh
      spdlog::info("Loading mesh '{}' from ProjectDB", opt.data_name);
      auto mesh = db.mesh(opt.data_name);

      if (mesh->polygons.empty()) {
        spdlog::error("Mesh has no polygons");
        return RuxError::INVALID_ARGUMENT;
      }
      spdlog::info("Loaded {} polygons", mesh->polygons.size());

      // Unweld mesh vertices at sharp edges for correct shading
      auto unwelded = geometry::unweld_mesh(*mesh, 0.0f);
      spdlog::info("Unwelded mesh: {} vertices",
                   unwelded->cloud.data.size() / unwelded->cloud.point_step);

      speckle_obj = std::make_unique<io::speckle::Mesh>(
          io::speckle::to_speckle(*unwelded));
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
    spdlog::error("Speckle export failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
