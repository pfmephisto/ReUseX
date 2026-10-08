// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The heavy pieces the web API needs but ruxd_api_lib must not link (#268):
// SAM3 segmenters (reusex_vision), the evidence renderer (reusex_visualize /
// VTK), the managed-SAM3 provider, and the `optimize` stage (reusex_slam /
// GTSAM). ruxd_lib links the `reusex` umbrella, so they are built here and
// injected into api::Server by ruxd's main (src/local.cpp). Moved from
// `rux gui` (apps/rux/src/gui.cpp) with the rest of the GUI backend.

#include "injected.hpp"

#include "api/BackgroundModelProvider.hpp"
#include "api/FrameSegmenter.hpp"
#include "api/ModelProvider.hpp"
#include "api/ViewRenderer.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/pipeline/optimize_stage.hpp>
#include <reusex/pipeline/stages.hpp>
#include <reusex/vision/model_factory.hpp>
#include <reusex/vision/sam3/sam3_assets.hpp>
#include <reusex/vision/sam3_prompt.hpp>
#include <reusex/vision/segment_image.hpp>
#include <reusex/vision/segment_panorama.hpp>
#include <reusex/visualize/render_view.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <spdlog/spdlog.h>

#include <atomic>
#include <filesystem>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

/// Concrete SAM3 segmenter registered with the API server by ruxd's main
/// (#409, #467). Lives in ruxd_lib (which links the full reusex umbrella
/// including reusex_vision) so ruxd_api_lib itself stays free of
/// libtorch/TensorRT.
///
/// Thread safety: model is created on first call and cached by (path,
/// use_cuda). A mutex ensures only one inference runs at a time (SAM3 engines
/// are not re-entrant).
class DefaultFrameSegmenter : public ruxd::api::IFrameSegmenter {
    public:
  ruxd::api::SegmentFrameResult
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

    reusex::vision::SegmentImageInfo info;
    cv::Mat label_map = reusex::vision::segment_image(
        *model_, image_bgr, prompts, confidence, &info);

    std::vector<std::string> class_names;
    class_names.reserve(prompts.size());
    for (const auto &p : prompts)
      class_names.push_back(p.text);

    return {std::move(label_map), std::move(class_names),
            info.geometry_prompts_used};
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
class DefaultPanoramaSegmenter : public ruxd::api::IPanoramaSegmenter {
    public:
  ruxd::api::SegmentPanoramaResult
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

/// Concrete evidence renderer (Kortlægning). render_view() drives VTK, which
/// is not safe to run concurrently in one process — hence the mutex.
class DefaultViewRenderer final : public ruxd::api::IViewRenderer {
    public:
  std::vector<std::uint8_t>
  render_png(const reusex::ProjectDB &db,
             const ruxd::api::RenderRequest &req) override {
    namespace viz = reusex::visualize;
    viz::RenderOptions o;
    o.layers.clear();
    for (const auto &name : req.layers) {
      const auto layer = viz::layer_from_string(name);
      if (!layer)
        throw std::invalid_argument("unknown layer '" + name + "'");
      o.layers.push_back(*layer);
    }
    const auto view = viz::view_preset_from_string(req.view);
    if (!view)
      throw std::invalid_argument("unknown view '" + req.view + "'");
    o.view = *view;
    o.orbit_index = req.orbit_index;
    o.width = req.width;
    o.height = req.height;
    if (req.highlight_instance)
      o.highlight = viz::InstanceHighlight{
          req.highlight_cloud.value_or(viz::InstanceHighlight{}.cloud_name),
          *req.highlight_instance};
    std::lock_guard lock(mutex_);
    cv::Mat image;
    try {
      image = viz::render_view(db, o);
    } catch (const viz::OffscreenGlUnavailable &e) {
      throw ruxd::api::RenderUnavailable(e.what());
    }
    if (image.empty())
      throw std::runtime_error("render produced an empty image");
    std::vector<std::uint8_t> png;
    if (!cv::imencode(".png", image, png) || png.empty())
      throw std::runtime_error("PNG encode failed for a " +
                               std::to_string(image.cols) + "x" +
                               std::to_string(image.rows) + " render");
    return png;
  }

    private:
  std::mutex mutex_;
};

/// Managed-SAM3-model provider (self-contained packaging). Resolves an omitted
/// request model_path to a managed model: downloads the portable ONNX on first
/// use and builds the device-specific TensorRT engines, both off the request
/// thread. The retry/cancellation state machine is ruxd::api's
/// BackgroundModelProvider; only the SAM3 prepare/probe glue lives here in
/// ruxd_lib (links reusex_vision), so the API server stays free of the
/// vision/TensorRT closure.
std::unique_ptr<ruxd::api::IModelProvider>
sam3_model_provider(std::filesystem::path models_dir,
                    std::string explicit_model, std::string manifest_url) {
  namespace sam3 = reusex::vision::sam3;
  sam3::Sam3AssetOptions base;
  base.models_dir = std::move(models_dir);
  base.manifest_url = std::move(manifest_url);

  auto prepare =
      [base](bool use_cuda,
             const ruxd::api::BackgroundModelProvider::ProgressFn &progress,
             const std::atomic<bool> &stop) {
        auto opts = base;
        opts.use_cuda = use_cuda;
        opts.cancel = &stop;
        const auto dir = sam3::prepare_sam3_model(
            opts, [&progress](const sam3::PrepProgress &p) {
              progress(
                  {sam3::to_string(p.state), p.fraction, p.message, "", {}});
            });
        return dir.string();
      };
  auto probe = [base](bool use_cuda) -> ruxd::api::ModelPrepStatus {
    auto opts = base;
    opts.use_cuda = use_cuda;
    const auto p = sam3::sam3_status(opts);
    return {sam3::to_string(p.state), p.fraction, p.message, "",
            p.update_engines};
  };
  return std::make_unique<ruxd::api::BackgroundModelProvider>(
      std::move(prepare), std::move(probe), std::move(explicit_model));
}

} // namespace

namespace ruxd {

std::unique_ptr<api::IFrameSegmenter> make_frame_segmenter() {
  return std::make_unique<DefaultFrameSegmenter>();
}

std::unique_ptr<api::IPanoramaSegmenter> make_panorama_segmenter() {
  return std::make_unique<DefaultPanoramaSegmenter>();
}

std::unique_ptr<api::IViewRenderer> make_view_renderer() {
  return std::make_unique<DefaultViewRenderer>();
}

std::unique_ptr<api::IModelProvider>
make_sam3_model_provider(std::filesystem::path models_dir,
                         std::string explicit_model, std::string manifest_url) {
  return sam3_model_provider(std::move(models_dir), std::move(explicit_model),
                             std::move(manifest_url));
}

reusex::pipeline::StageExecutor make_stage_executor() {
  // Shared with the Qt client (rux): reusex/pipeline/optimize_stage.hpp.
  return reusex::pipeline::stage_executor_with_optimize();
}

} // namespace ruxd
