// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/logging.hpp"
#include "core/processing_observer.hpp"
#include "vision/BackendFactory.hpp"
#include "vision/Dataloader.hpp"
// #include "vision/Dataset.hpp"
#include "vision/annotate.hpp"
// #include "vision/infer/sam3infer.hpp"
#include "vision/utils.hpp"

#include <array>
#include <opencv2/core.hpp>
#include <string>
#include <thread>

#ifdef NDEBUG
// For visualization/debugging purposes
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#endif

#include <pcl/common/colors.h>

#include <range/v3/all.hpp>
namespace reusex::vision {

// Ordered, single-threaded annotation for stateful video models (SAM 3.1).
//
// Unlike the default batch path, an IVideoModel carries a temporal memory bank
// and MUST see frames in order (see IVideoModel contract). We therefore bypass
// the shuffled Dataloader entirely and iterate the dataset indices in ascending
// order, calling step() one frame at a time and saving each result.
static int annotate_video(IMLBackend &backend, IDataset &dataset,
                          Model modelType,
                          const std::filesystem::path &modelPath,
                          const AnnotationConfig &config) {
  reusex::info("Running annotation in video-tracker mode (single-threaded, "
               "ordered)");

  auto tracker =
      backend.create_video_model(modelType, modelPath, config.use_cuda);
  if (!tracker) {
    reusex::error("Failed to create video model from path: {}", modelPath);
    return 1;
  }

  if (config.skip_annotated) {
    // The default video path (memory-conditioning off, see TensorRTSam3p1) is
    // stateless per frame and order-preserving, so already-annotated frames can
    // simply be dropped and the remainder processed in ascending node-id order.
    // (If the experimental memory-conditioning is enabled, dropping
    // mid-sequence frames leaves gaps in the memory bank; that path is opt-in
    // only.)
    auto skipped = dataset.filter_annotated();
    reusex::info("Skipping {} already-annotated frames", skipped);
    if (dataset.size() == 0) {
      reusex::info("All frames already annotated, nothing to do");
      return 0;
    }
  }

  // A ReUseX project DB holds a single ordered scan (sensor frames keyed by
  // ascending node id), so one reset() before the first frame is the correct
  // and only sequence boundary.
  const size_t total = dataset.size();
  size_t frame_count = 0;
  {
    auto observer = reusex::core::ProgressObserver(
        reusex::core::Stage::annotating_batches, total);
    for (size_t i = 0; i < total; ++i) {
      if (i == 0)
        tracker->reset(); // start of the (single) scan

      auto in = dataset.get(i);
      auto out = tracker->step(in);

      std::array<IDataset::Pair, 1> one{std::move(out)};
      dataset.save(one);

      reusex::trace("Video frame {}/{} annotated", ++frame_count, total);
      ++observer;
    }
  }

  reusex::info("Video annotation completed ({} frames)", frame_count);
  return 0;
}

auto annotate(const std::filesystem::path &dbPath,
              const std::filesystem::path &modelPath,
              const AnnotationConfig &config) -> int {
  reusex::trace("calling annotate");

  // Create backend, model, and dataset
  auto backendType = BackendFactory::detect_backend(modelPath);
  auto backend = BackendFactory::create(backendType);

  auto modelType = BackendFactory::detect_model(modelPath);
  auto dataset = backend->create_dataset(dbPath);
  dataset->set_confidence(config.confidence);
  if (!config.prompts.empty()) {
    reusex::info("Using {} custom concept prompts", config.prompts.size());
    dataset->set_prompts(config.prompts);
  }

  // Video-tracker path: stateful, ordered, single-threaded. Triggered by the
  // explicit --video flag or automatically when a SAM 3.1 model is detected.
  if (config.video || modelType == Model::sam3p1) {
    return annotate_video(*backend, *dataset, modelType, modelPath, config);
  }

  auto model = backend->create_model(modelType, modelPath, config.use_cuda);

  // Filter out already-annotated frames if requested
  if (config.skip_annotated) {
    auto skipped = dataset->filter_annotated();
    if (dataset->size() == 0) {
      reusex::info("All {} frames already annotated, nothing to do", skipped);
      return 0;
    }
  }

  // Log dataloader configuration
  reusex::info("Dataloader config: batch_size={}, shuffle={}, seed={}, "
               "num_workers={}, prefetch={}",
               config.batch_size, config.shuffle,
               config.seed ? std::to_string(*config.seed)
                           : std::string("random_device"),
               config.num_workers, config.prefetch_batches);

  // Warn about potentially problematic configurations
  if (config.batch_size > 64) {
    reusex::warn(
        "Batch size {} exceeds recommended maximum (64). May cause GPU OOM.",
        config.batch_size);
  }

  const auto hw_threads = std::thread::hardware_concurrency();
  if (config.num_workers > hw_threads) {
    reusex::warn("Workers ({}) exceed hardware threads ({}). May cause "
                 "over-subscription.",
                 config.num_workers, hw_threads);
  }

  if (config.prefetch_batches < config.num_workers) {
    reusex::warn("Prefetch ({}) < workers ({}). May cause GPU starvation. "
                 "Recommended: 2-3x workers.",
                 config.prefetch_batches, config.num_workers);
  }

  // Create dataloader with multi-threaded prefetching
  Dataloader loader(*dataset, config.batch_size, config.shuffle,
                    config.num_workers, config.prefetch_batches, config.seed);

#ifndef NDEBUG
  // INFO: Create and OpenCV window for visualizing the results during
  // development
  cv::namedWindow("Annotation", cv::WINDOW_AUTOSIZE);
#endif

  reusex::info("Starting annotation with {} batches using {} worker threads",
               loader.size(), loader.get_num_workers());

  size_t batch_count = 0;
  {
    auto observer = reusex::core::ProgressObserver(
        reusex::core::Stage::annotating_batches, loader.size());
    for (auto batch : loader) {
      // for (auto [logger, batch] : spdmon::LogProgress(loader)) {
      reusex::trace("Processing batch {}/{} with {} items", ++batch_count,
                    loader.size(), batch.size());
      auto results = model->forward(batch);
      dataset->save(results);
      ++observer;
    }
  }

#ifndef NDEBUG
  // INFO: Wait for user input before closing the visualization window
  cv::waitKey(5 * 1000); // Wait for 5 seconds
  cv::destroyWindow("Annotation");
  // cv::destroyAllWindows();
#endif

  reusex::info("Annotation completed");
  return 0;
}

} // namespace reusex::vision
