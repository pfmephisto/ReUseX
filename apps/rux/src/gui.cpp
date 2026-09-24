// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "gui.hpp"
#include "exit_status.hpp"
#include "gui/FrameSegmenter.hpp"
#include "gui/Server.hpp"

#include <reusex/vision/model_factory.hpp>
#include <reusex/vision/sam3_prompt.hpp>
#include <reusex/vision/segment_image.hpp>
#include <reusex/vision/segment_panorama.hpp>

#include <opencv2/core.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

/// Concrete SAM3 segmenter registered with the GUI server by the rux app layer
/// (#409). Lives in rux_lib (which links the full reusex umbrella including
/// reusex_vision) so the GUI library itself stays free of libtorch/TensorRT.
///
/// Thread safety: model is created on first call and cached by path. A mutex
/// ensures only one inference runs at a time (SAM3 engines are not re-entrant).
class DefaultFrameSegmenter : public rux::gui::IFrameSegmenter {
    public:
  rux::gui::SegmentFrameResult
  segment(const cv::Mat &image_bgr,
          const std::vector<reusex::vision::Sam3Prompt> &prompts,
          float confidence, const std::string &model_path) override {
    std::lock_guard<std::mutex> lock(mutex_);

    // Reload when the model path changes (supports runtime model switching).
    if (!model_ || model_path != current_path_) {
      spdlog::info("GUI segmenter: loading SAM3 model from {}", model_path);
      try {
        model_ = reusex::vision::create_model_from_path(model_path,
                                                        /*use_cuda=*/true);
        current_path_ = model_path;
      } catch (const std::exception &e) {
        throw std::runtime_error(std::string("failed to load SAM3 model: ") +
                                 e.what());
      }
    }

    cv::Mat label_map =
        reusex::vision::segment_image(*model_, image_bgr, prompts, confidence);

    std::vector<std::string> class_names;
    class_names.reserve(prompts.size());
    for (const auto &p : prompts)
      class_names.push_back(p.text);

    return {std::move(label_map), std::move(class_names)};
  }

    private:
  std::mutex mutex_;
  std::unique_ptr<reusex::vision::IModel> model_;
  std::string current_path_;
};

/// Concrete panorama segmenter registered with the GUI server (#448).
///
/// Reuses the same model cache as DefaultFrameSegmenter (both are kept alive
/// for the server's lifetime and share nothing else). A mutex ensures only one
/// SAM3 inference runs at a time.
class DefaultPanoramaSegmenter : public rux::gui::IPanoramaSegmenter {
    public:
  rux::gui::SegmentPanoramaResult
  segment(const cv::Mat &equirect_bgr,
          const std::vector<reusex::vision::Sam3Prompt> &prompts,
          float confidence, int n_yaw, double fov_deg,
          const std::string &model_path) override {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!model_ || model_path != current_path_) {
      spdlog::info("GUI panorama segmenter: loading SAM3 model from {}",
                   model_path);
      try {
        model_ = reusex::vision::create_model_from_path(model_path,
                                                        /*use_cuda=*/true);
        current_path_ = model_path;
      } catch (const std::exception &e) {
        throw std::runtime_error(std::string("failed to load SAM3 model: ") +
                                 e.what());
      }
    }

    reusex::vision::SegmentPanoramaOptions opts;
    opts.n_yaw = n_yaw;
    opts.fov_deg = fov_deg;
    opts.confidence = confidence;
    opts.prompts.reserve(prompts.size());
    for (const auto &p : prompts)
      opts.prompts.push_back(p.text);

    cv::Mat label_map =
        reusex::vision::segment_panorama(*model_, equirect_bgr, opts);

    std::vector<std::string> class_names;
    class_names.reserve(prompts.size());
    for (const auto &p : prompts)
      class_names.push_back(p.text);

    return {std::move(label_map), std::move(class_names)};
  }

    private:
  std::mutex mutex_;
  std::unique_ptr<reusex::vision::IModel> model_;
  std::string current_path_;
};

} // namespace

void setup_subcommand_gui(CLI::App &app,
                          std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandGuiOptions>();

  auto *sub = app.add_subcommand("gui", "Serve the web interface locally");

  sub->footer(R"(
DESCRIPTION:
  Starts a local web server for the project selected with -p/--project and
  opens it in a browser. The server implements the REST + WebSocket contract in
  docs/gui/openapi.yaml: project summary, clouds, meshes, sensor frames,
  panoramas, building components, material passports, instances, the pipeline
  log, and job submit/status/cancel with live progress.

  Pipeline stages submitted through the API run IN-PROCESS, one at a time, on a
  single worker thread — progress is reported through the same observer the CLI
  progress bar uses.

EXAMPLES:
  rux -p scan.rux gui                  # Serve and open a browser
  rux -p scan.rux gui --no-browser     # Headless (CI, remote shell)
  rux -p scan.rux gui --port 9000      # Custom port
  rux -p scan.rux gui --assets ./dist  # Serve a locally built frontend

NOTES:
  - Binds to 127.0.0.1 by default. There is NO authentication and the API can
    execute pipeline stages, so only change --bind if you know what that means.
  - Cross-origin requests are refused unless they come from loopback or an
    origin named with --allow-origin. WebSocket upgrades are checked the same
    way, since CORS does not apply to them.
  - Without a frontend bundle a built-in placeholder page is served that lists
    the live API. The real frontend is Phase 2 of issue #265.
  - Asset lookup order: --assets, then $RUX_GUI_ASSETS, then
    <install prefix>/share/reusex/gui.
  - The viewport renders the Gaussian splats stored in the project by
    'rux create gsplat' (schema v12) as a layer of their own. An existing .ply
    from an older run can be brought in with 'rux import gsplat'.
  - The project is created if it does not exist, exactly like other
    project-writing subcommands.
)");

  sub->add_option("--port", opt->server.port, "TCP port to listen on")
      ->default_val(opt->server.port)
      ->check(CLI::Range(1, 65535));

  sub->add_option("--bind", opt->server.bind_address,
                  "Interface to bind (default: loopback only)")
      ->default_val(opt->server.bind_address);

  sub->add_option("--threads", opt->server.threads,
                  "HTTP worker threads (0 = hardware concurrency)")
      ->default_val(opt->server.threads);

  sub->add_option("--assets", opt->server.asset_dir,
                  "Directory holding the frontend bundle")
      ->check(CLI::ExistingDirectory);

  sub->add_option("--allow-origin", opt->server.allowed_origins,
                  "Additional browser origin allowed to call the API "
                  "(repeatable). Loopback is always allowed.");

  sub->add_flag("--no-browser", opt->no_browser,
                "Do not open a browser on startup");

  sub->callback([opt, global_opt]() {
    rux::finish(run_subcommand_gui(*opt, *global_opt));
  });
}

int run_subcommand_gui(SubcommandGuiOptions const &opt,
                       const RuxOptions &global_opt) {
  try {
    rux::gui::ServerOptions server_options = opt.server;
    server_options.project = global_opt.project_db;
    server_options.open_browser = !opt.no_browser;

    rux::gui::Server server(std::move(server_options));

    // Register segmenters so the SAM3 endpoints work (#409, #448).
    DefaultFrameSegmenter segmenter;
    server.set_segmenter(&segmenter);

    DefaultPanoramaSegmenter panorama_segmenter;
    server.set_panorama_segmenter(&panorama_segmenter);

    spdlog::info("Serving {} at {}", global_opt.project_db.string(),
                 server.url());
    return server.run();

  } catch (const std::exception &e) {
    spdlog::error("Could not start the GUI server: {}", e.what());
    return RuxError::GENERIC;
  }
}
