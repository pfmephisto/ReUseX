// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "export/speckle.hpp"
#include "global-params.hpp"

#include <fmt/format.h>
#include <pcl/io/pcd_io.h>
#include <reusex/io/speckle.hpp>
#include <reusex/types.hpp>
#include <spdlog/spdlog.h>

#include <cstdlib>
#include <stdexcept>

namespace fs = std::filesystem;
using namespace ReUseX;

void setup_subcommand_export_speckle(CLI::App &parent) {
  auto opt = std::make_shared<SubcommandExportSpeckleOptions>();
  auto *sub = parent.add_subcommand(
      "speckle",
      "Export a point cloud to Speckle (app.speckle.systems).\n\n"
      "Authentication: Set SPECKLE_TOKEN environment variable.\n"
      "Find your token at: https://app.speckle.systems/profile\n\n"
      "How to find your Project ID:\n"
      "  1. Open your project in Speckle web UI\n"
      "  2. Copy the project ID from the URL: /projects/{PROJECT_ID}\n\n"
      "Example:\n"
      "  export SPECKLE_TOKEN=your_token_here\n"
      "  rux export speckle cloud.pcd \\\n"
      "    -s https://app.speckle.systems \\\n"
      "    -p abc123def456 \\\n"
      "    -m main \\\n"
      "    -M \"Point cloud from ReUseX\"");

  sub->add_option("cloud", opt->cloud_path_in,
                  "Path to the input point cloud file (PCD format).")
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

  sub->add_option("--chunk-size", opt->chunk_size,
                  "DataChunk size (default: 50000 points)")
      ->default_val(opt->chunk_size);

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

  // 2. Load point cloud
  spdlog::info("Loading point cloud from: {}", opt.cloud_path_in.string());
  CloudPtr cloud(new Cloud);
  if (pcl::io::loadPCDFile<PointT>(opt.cloud_path_in.c_str(), *cloud) < 0) {
    spdlog::error("Failed to load point cloud from: {}",
                  opt.cloud_path_in.string());
    return RuxError::INVALID_ARGUMENT;
  }

  if (cloud->empty()) {
    spdlog::error("Point cloud is empty");
    return RuxError::INVALID_ARGUMENT;
  }

  spdlog::info("Loaded {} points", cloud->size());

  // 3. Convert to Speckle format
  spdlog::debug("Converting point cloud to Speckle format");
  auto speckle_cloud = io::speckle::to_speckle(cloud);

  // 4. Create Speckle client
  spdlog::debug("Creating Speckle client for server: {}", opt.server_url);
  spdlog::debug("Project ID: {}", opt.project_id);
  io::speckle::SpeckleClient client(opt.server_url, opt.project_id, token);

  // 5. Configure batching
  client.set_max_batch_size(opt.max_batch_bytes);
  client.set_chunk_size(opt.chunk_size);

  if (opt.max_batch_bytes < 1024 * 1024) {
    spdlog::warn("Very small batch size (< 1MB) may impact performance");
  }
  if (opt.chunk_size > 500000) {
    spdlog::warn("Very large chunk size (> 500k) may cause memory issues");
  }

  spdlog::debug("Batch size: {} bytes", opt.max_batch_bytes);
  spdlog::debug("Chunk size: {} points", opt.chunk_size);

  // 6. Upload
  try {
    spdlog::info("Uploading to model '{}' with message: '{}'", opt.model_name,
                 opt.commit_message);

    std::string commit_id =
        client.upload(speckle_cloud, opt.model_name, opt.commit_message);

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
