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

  // skip_annotated in video mode: only a contiguous already-done PREFIX may be
  // skipped, because the memory bank must be warmed from the first processed
  // frame. Mid-sequence skipping would leave gaps in the temporal state.
  size_t start_index = 0;
  if (config.skip_annotated) {
    reusex::warn("--skip-annotated in video mode only skips a contiguous "
                 "already-annotated prefix; mid-sequence skipping is "
                 "unsupported (the tracker memory must run in order).");
    // filter_annotated() removes non-contiguous ids too, which would corrupt
    // ordering, so we do NOT call it here. Prefix handling is a future
    // refinement (see TODO).
    // TODO: Support contiguous-prefix resume in video annotation
    // category=Vision estimate=2h
    // Query segmentation_image_ids(), find the longest already-done prefix of
    // the ordered id list, set start_index past it, and reset() the tracker at
    // that boundary. For v1 we always start at 0 and re-annotate everything.
  }

  // TODO: Detect sequence boundaries for multi-sequence datasets
  // category=Vision estimate=3h
  // The current dataset represents a single ordered scan, so one reset() at the
  // first frame is correct. If a dataset ever concatenates multiple sequences,
  // add an IDataset::node_id(index) accessor and reset() whenever the node-id
  // delta indicates a new sequence.

  const size_t total = dataset.size();
  size_t frame_count = 0;
  {
    auto observer = reusex::core::ProgressObserver(
        reusex::core::Stage::annotating_batches, total - start_index);
    for (size_t i = start_index; i < total; ++i) {
      if (i == start_index)
        tracker->reset(); // sequence boundary

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
