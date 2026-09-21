// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/attributes.hpp"
#include "stage_prerequisites.hpp"

#include "exit_status.hpp"
#include <reusex/core/ProjectDB.hpp>
#include <reusex/vision/describe.hpp>

#include <spdlog/spdlog.h>

#include <cstdlib>
#include <filesystem>

namespace fs = std::filesystem;

void setup_subcommand_create_attributes(
    CLI::App &app, std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandAttributesOptions>();
  auto *sub = app.add_subcommand(
      "attributes",
      "Describe each material with a vision-language model (VLM)");

  // Initialize CLI defaults FROM the library options struct (STANDARDS §4).
  const reusex::vision::DescribeConfig defaults;
  opt->api_url = defaults.api_url;
  opt->model = defaults.model;
  opt->instances_cloud = defaults.instances_cloud;
  opt->point_cloud = defaults.point_cloud;
  opt->crop_padding = defaults.crop_padding;
  opt->min_view_points = defaults.min_view_points;
  opt->skip_existing = defaults.skip_existing;

  sub->footer(R"(
DESCRIPTION:
  For each MATERIAL passport in the project, resolves the instance(s) it is
  linked to, crops their best sensor view, sends the crop to an OpenAI-compatible
  chat/completions endpoint (a local Ollama by default, or any cloud provider),
  and stores the answer — a free-text description plus arbitrary key/value
  attributes — keyed by the material GUID for use in a MaterialPassport. The
  prompt fully drives which attribute keys come back, so you can ask for any
  properties you need with --prompt.

EXAMPLES:
  rux create attributes                                   # local Ollama
  rux create attributes --model llava:13b
  rux create attributes --api-url https://api.openai.com/v1 \
                        --model gpt-4o --api-key sk-...
  REUSEX_VLM_API_KEY=sk-... rux create attributes --api-url ...

WORKFLOW:
  1. rux create clouds        # fused point cloud
  2. rux create annotate ...  # 2D semantic labels
  3. rux create project       # 3D labels
  4. rux create instances     # spatial instances
  5. rux create materials     # one material passport per instance
  6. rux create attributes    # <-- this stage

NOTES:
  - A material with no linked instance (or no usable view) is logged and
    skipped; descriptions/attributes are never fabricated.
  - --api-key falls back to the REUSEX_VLM_API_KEY environment variable.
)");

  sub->add_option("--api-url", opt->api_url,
                  "OpenAI-compatible base URL (a '/chat/completions' suffix is "
                  "appended automatically)")
      ->default_val(opt->api_url);
  sub->add_option("--model", opt->model, "Model name the endpoint routes to")
      ->default_val(opt->model);
  sub->add_option("--api-key", opt->api_key,
                  "Bearer token for cloud providers (falls back to "
                  "$REUSEX_VLM_API_KEY)");
  sub->add_option("--instances", opt->instances_cloud,
                  "Name of the instance-label cloud materials resolve to")
      ->default_val(opt->instances_cloud);
  sub->add_option("--cloud", opt->point_cloud,
                  "Name of the XYZRGB cloud aligned with --instances")
      ->default_val(opt->point_cloud);
  sub->add_option(
      "--prompt", opt->prompt,
      "Override the instruction sent with each crop (empty asks for "
      "a description + attributes object; ask for any keys here)");
  sub->add_flag("--skip-existing", opt->skip_existing,
                "Skip materials that already have a stored annotation (resume)")
      ->default_val(opt->skip_existing);
  sub->add_option("--crop-padding", opt->crop_padding,
                  "Pixels of padding around each instance's projected bbox")
      ->check(CLI::Range(0, 512))
      ->default_val(opt->crop_padding);
  sub->add_option("--min-view-points", opt->min_view_points,
                  "Minimum in-bounds projected points for a frame to be a "
                  "usable view")
      ->check(CLI::Range(1, 1000000))
      ->default_val(opt->min_view_points);

  sub->callback([opt, global_opt]() {
    rux::finish(run_subcommand_attributes(*opt, *global_opt));
  });
}

int run_subcommand_attributes(SubcommandAttributesOptions const &opt,
                              const RuxOptions &global_opt) {
  try {
    fs::path project_path = global_opt.project_db;

    // Scope the prerequisite handle so it is closed before describe() opens its
    // own read-write connection (sqlite handles are not shared for writes).
    {
      reusex::ProjectDB db(project_path);
      if (int rc = rux::check_stage_prerequisites(
              db, reusex::core::PipelineStage::attributes);
          rc != RuxError::SUCCESS)
        return rc;
    }

    reusex::vision::DescribeConfig config;
    config.api_url = opt.api_url;
    config.model = opt.model;
    config.api_key = opt.api_key;
    if (config.api_key.empty()) {
      if (const char *env = std::getenv("REUSEX_VLM_API_KEY"); env)
        config.api_key = env;
    }
    config.instances_cloud = opt.instances_cloud;
    config.point_cloud = opt.point_cloud;
    config.prompt = opt.prompt;
    config.skip_existing = opt.skip_existing;
    config.crop_padding = opt.crop_padding;
    config.min_view_points = opt.min_view_points;

    return reusex::vision::describe(project_path, config);
  } catch (const std::exception &e) {
    spdlog::error("Attribute extraction failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
