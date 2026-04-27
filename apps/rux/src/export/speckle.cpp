// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "export/speckle.hpp"
#include "global-params.hpp"

#include <fmt/format.h>
#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/export_scene.hpp>
#include <reusex/io/speckle.hpp>
#include <spdlog/spdlog.h>

#include <cstdlib>
#include <stdexcept>

namespace fs = std::filesystem;
using namespace reusex;

void setup_subcommand_export_speckle(CLI::App &parent,
                                     std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandExportSpeckleOptions>();
  auto *sub =
      parent.add_subcommand("speckle", "Export project data to Speckle");

  sub->footer(R"(
DESCRIPTION:
  Uploads all available project data from ReUseX ProjectDB to Speckle
  platform for web-based 3D visualization, collaboration, and BIM
  integration. Each data category (cloud, semantic, mesh, 360, materials)
  is uploaded as a separate model/branch.

AUTHENTICATION:
  Set SPECKLE_TOKEN environment variable with your personal access token.
  Get token at: https://app.speckle.systems/profile

PROJECT ID:
  Find in Speckle web UI URL: /projects/{PROJECT_ID}

MODEL STRUCTURE:
  Project/
  ├── Model "cloud"       # XYZRGB point cloud
  ├── Model "semantic"    # Collection with per-category sub-collections
  ├── Model "mesh"        # Collection with individual meshes
  ├── Model "360"         # Collection with sphere meshes + properties
  ├── Model "materials"   # Collection with Points + properties
  └── Model "components"  # Collection with building component boundaries

  With --root scan1:
  Project/
  ├── Model "scan1/cloud"
  ├── Model "scan1/semantic"
  └── ...

EXAMPLES:
  export SPECKLE_TOKEN=your_token_here
  rux export speckle \
    -s https://app.speckle.systems \
    -p abc123def456

  rux -p project.rux export speckle \
    -s https://app.speckle.systems \
    -p abc123def456 \
    -M "Full project export"

WORKFLOW:
  1. Create Speckle project at app.speckle.systems
  2. Copy project ID from URL
  3. export SPECKLE_TOKEN=your_token
  4. rux export speckle -s URL -p PROJECT_ID

NOTES:
  - Requires network connection to Speckle server
  - Only models with data are uploaded (empty categories are skipped)
  - Each category creates its own model/branch in the Speckle project
  - Use -M/--message to annotate versions with description
)");

  sub->add_option("-s,--server", opt->server_url,
                  "Speckle server URL (e.g., https://app.speckle.systems)")
      ->required();

  sub->add_option("-p,--project-id", opt->project_id,
                  "Project/stream ID (find in URL: /projects/{PROJECT_ID})")
      ->required();

  sub->add_option("-M,--message", opt->commit_message, "Version commit message")
      ->default_val(opt->commit_message);

  sub->add_option("-r,--root", opt->root_folder,
                  "Root folder prefix for model names (e.g., 'scan1' -> scan1/cloud)");

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

  // Validate environment
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
    ProjectDB db(project_path, true);

    spdlog::info("Gathering export data...");
    auto scene = io::gather_export_scene(db);

    spdlog::info("Building Speckle objects...");
    auto models = io::speckle::export_to_speckle(scene);

    if (models.empty()) {
      spdlog::warn("No data to export — project database is empty");
      return RuxError::SUCCESS;
    }

    // Create client
    spdlog::debug("Creating Speckle client for server: {}", opt.server_url);
    io::speckle::SpeckleClient client(opt.server_url, opt.project_id, token);
    client.set_max_batch_size(opt.max_batch_bytes);

    if (opt.max_batch_bytes < 1024 * 1024)
      spdlog::warn("Very small batch size (< 1MB) may impact performance");

    // Upload each model to its own branch
    for (const auto &m : models) {
      std::string branch = opt.root_folder.empty()
                               ? m.model_name
                               : fmt::format("{}/{}", opt.root_folder, m.model_name);
      spdlog::info("Uploading model '{}'...", branch);
      std::string commit_id =
          client.upload(*m.root, branch, opt.commit_message);
      spdlog::info("Uploaded model '{}' (commit: {})", branch, commit_id);
    }

    spdlog::info("Upload complete! {} models exported.", models.size());

    // Construct viewable URL
    std::string view_url =
        fmt::format("{}/projects/{}", opt.server_url, opt.project_id);
    spdlog::info("View at: {}", view_url);

    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Speckle export failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
