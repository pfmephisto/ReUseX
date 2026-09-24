// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "gui.hpp"
#include "exit_status.hpp"
#include "gui/FrameSegmenter.hpp"
#include "gui/Server.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/pipeline/stages.hpp>
#include <reusex/slam/PlaneGraphOptimizer.hpp>
#include <reusex/vision/model_factory.hpp>
#include <reusex/vision/sam3_prompt.hpp>
#include <reusex/vision/segment_image.hpp>
#include <reusex/vision/segment_panorama.hpp>

#include <fmt/format.h>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

/// Concrete SAM3 segmenter registered with the GUI server by the rux app layer
/// (#409, #467). Lives in rux_lib (which links the full reusex umbrella
/// including reusex_vision) so the GUI library itself stays free of
/// libtorch/TensorRT.
///
/// Thread safety: model is created on first call and cached by (path,
/// use_cuda). A mutex ensures only one inference runs at a time (SAM3 engines
/// are not re-entrant).
class DefaultFrameSegmenter : public rux::gui::IFrameSegmenter {
    public:
  rux::gui::SegmentFrameResult
  segment(const cv::Mat &image_bgr,
          const std::vector<reusex::vision::Sam3Prompt> &prompts,
          float confidence, const std::string &model_path,
          bool use_cuda) override {
    std::lock_guard<std::mutex> lock(mutex_);

    // Reload when the model path or cuda preference changes.
    if (!model_ || model_path != current_path_ || use_cuda != current_cuda_) {
      spdlog::info("GUI segmenter: loading SAM3 model from {} (cuda={})",
                   model_path, use_cuda);
      try {
        model_ = reusex::vision::create_model_from_path(model_path, use_cuda);
        current_path_ = model_path;
        current_cuda_ = use_cuda;
      } catch (const std::exception &e) {
        if (use_cuda) {
          // Graceful CPU fallback: if CUDA loading fails, try ONNX/CPU (#467).
          spdlog::warn("GUI segmenter: CUDA model load failed ({}); retrying "
                       "on CPU",
                       e.what());
          try {
            model_ = reusex::vision::create_model_from_path(model_path,
                                                            /*use_cuda=*/false);
            current_path_ = model_path;
            current_cuda_ = false;
          } catch (const std::exception &e2) {
            throw std::runtime_error(
                std::string("failed to load SAM3 model: ") + e2.what());
          }
        } else {
          throw std::runtime_error(std::string("failed to load SAM3 model: ") +
                                   e.what());
        }
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
  bool current_cuda_ = false;
};

/// Concrete panorama segmenter registered with the GUI server (#448, #467).
///
/// Reuses the same model-cache approach as DefaultFrameSegmenter. A mutex
/// ensures only one SAM3 inference runs at a time.
class DefaultPanoramaSegmenter : public rux::gui::IPanoramaSegmenter {
    public:
  rux::gui::SegmentPanoramaResult
  segment(const cv::Mat &equirect_bgr,
          const std::vector<reusex::vision::Sam3Prompt> &prompts,
          float confidence, int n_yaw, double fov_deg,
          const std::string &model_path, bool use_cuda) override {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!model_ || model_path != current_path_ || use_cuda != current_cuda_) {
      spdlog::info(
          "GUI panorama segmenter: loading SAM3 model from {} (cuda={})",
          model_path, use_cuda);
      try {
        model_ = reusex::vision::create_model_from_path(model_path, use_cuda);
        current_path_ = model_path;
        current_cuda_ = use_cuda;
      } catch (const std::exception &e) {
        if (use_cuda) {
          spdlog::warn("GUI panorama segmenter: CUDA model load failed ({}); "
                       "retrying on CPU",
                       e.what());
          try {
            model_ = reusex::vision::create_model_from_path(model_path,
                                                            /*use_cuda=*/false);
            current_path_ = model_path;
            current_cuda_ = false;
          } catch (const std::exception &e2) {
            throw std::runtime_error(
                std::string("failed to load SAM3 model: ") + e2.what());
          }
        } else {
          throw std::runtime_error(std::string("failed to load SAM3 model: ") +
                                   e.what());
        }
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
  bool current_cuda_ = false;
};

// ---------------------------------------------------------------------------
// Optimize executor (#464)
// ---------------------------------------------------------------------------
//
// LAYERING: reusex_pipeline does not link reusex_slam (GTSAM). This function
// lives in rux_lib (which links the full `reusex` umbrella) and is injected
// into the Server via ServerOptions::stage_executor, keeping rux_gui_lib free
// of the slam/GTSAM closure and the light test binary unaffected.

namespace {

template <typename T>
T param_or(const nlohmann::json &params, const char *key, T fallback) {
  auto it = params.find(key);
  if (it == params.end() || it->is_null())
    return fallback;
  return it->get<T>();
}

} // anonymous namespace

reusex::pipeline::StageResult
run_optimize_stage(reusex::ProjectDB &db,
                   const reusex::pipeline::StageContext &ctx) {
  nlohmann::json params;
  if (!ctx.parameters.empty()) {
    params = nlohmann::json::parse(ctx.parameters, nullptr,
                                   /*allow_exceptions=*/false);
    if (params.is_discarded() || !params.is_object())
      return reusex::pipeline::StageResult::invalid(
          "optimize parameters must be a JSON object");
  }

  reusex::geometry::PlaneGraphOptions options;
  options.min_landmark_observations =
      param_or(params, "min_observations", options.min_landmark_observations);
  options.assoc_rounds = param_or(params, "assoc_rounds", options.assoc_rounds);
  if (param_or(params, "no_gnc", false))
    options.use_gnc = false;

  const bool dry_run = param_or(params, "dry_run", false);

  try {
    auto result = reusex::geometry::optimize_sensor_poses(db, options, dry_run);

    if (result.landmarks == 0 && result.loop_edges == 0)
      return reusex::pipeline::StageResult::success(
          fmt::format("no plane landmarks reached min_observations={} and no "
                      "loop edges present; poses left unchanged",
                      options.min_landmark_observations));

    if (!result.converged)
      return reusex::pipeline::StageResult::failure(
          "factor-graph optimizer failed to produce a solution; poses left "
          "unchanged");

    if (dry_run)
      return reusex::pipeline::StageResult::success(
          fmt::format("dry run: {} frames, {} landmarks, {:.4f} -> {:.4f} "
                      "error, max shift {:.4f} m (poses not written)",
                      result.frames, result.landmarks, result.initial_error,
                      result.final_error, result.max_pose_shift));

    return reusex::pipeline::StageResult::success(
        fmt::format("{} frames, {} landmarks, {:.4f} -> {:.4f} error, max "
                    "shift {:.4f} m",
                    result.frames, result.landmarks, result.initial_error,
                    result.final_error, result.max_pose_shift),
        // `sensor_frames` is the artifact optimized poses are written into.
        {{"table", "sensor_frames", static_cast<int64_t>(result.frames)}});
  } catch (const std::exception &e) {
    return reusex::pipeline::StageResult::failure(
        fmt::format("pose optimization failed: {}", e.what()));
  }
}

/// StageExecutor that handles `optimize` directly and delegates everything else
/// to the default executor (clouds / planes / rooms / instances / mesh).
reusex::pipeline::StageExecutor make_gui_stage_executor() {
  auto default_exec = reusex::pipeline::default_stage_executor();
  return [default_exec = std::move(default_exec)](
             const reusex::pipeline::StageContext &ctx)
             -> reusex::pipeline::StageResult {
    if (ctx.stage == reusex::pipeline::JobStage::optimize) {
      try {
        reusex::ProjectDB db(ctx.project, /*readOnly=*/false);
        return run_optimize_stage(db, ctx);
      } catch (const std::exception &e) {
        return reusex::pipeline::StageResult::failure(fmt::format(
            "could not open project '{}': {}", ctx.project.string(), e.what()));
      }
    }
    return default_exec(ctx);
  };
}

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

  sub->add_flag(
      "--segment-cuda,!--no-segment-cuda", opt->server.segment_cuda,
      "Use CUDA/TensorRT for the segment endpoints (default: on). Pass "
      "--no-segment-cuda on hosts without a GPU to route inference through "
      "the ONNX CPU backend instead");

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
    // Wire the optimize stage (#464): reusex_pipeline does not link
    // reusex_slam, so the executor that calls optimize_sensor_poses() lives
    // here in rux_lib and is injected rather than compiled into rux_gui_lib.
    server_options.stage_executor = make_gui_stage_executor();

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
