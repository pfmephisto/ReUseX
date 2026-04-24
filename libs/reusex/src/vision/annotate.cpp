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

#include <opencv2/core.hpp>
#include <thread>

#ifdef NDEBUG
// For visualization/debugging purposes
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#endif

#include <pcl/common/colors.h>

#include <range/v3/all.hpp>
namespace reusex::vision {

auto annotate(const std::filesystem::path &dbPath,
              const std::filesystem::path &modelPath,
              const AnnotationConfig &config) -> int {
  reusex::trace("calling annotate");

  // Create backend, model, and dataset
  auto backendType = BackendFactory::detect_backend(modelPath);
  auto backend = BackendFactory::create(backendType);

  auto modelType = BackendFactory::detect_model(modelPath);
  auto model = backend->create_model(modelType, modelPath, config.use_cuda);
  auto dataset = backend->create_dataset(dbPath);

  // Log dataloader configuration
  reusex::info("Dataloader config: batch_size={}, shuffle={}, num_workers={}, prefetch={}",
               config.batch_size, config.shuffle, config.num_workers, config.prefetch_batches);

  // Warn about potentially problematic configurations
  if (config.batch_size > 64) {
    reusex::warn("Batch size {} exceeds recommended maximum (64). May cause GPU OOM.", config.batch_size);
  }

  const auto hw_threads = std::thread::hardware_concurrency();
  if (config.num_workers > hw_threads) {
    reusex::warn("Workers ({}) exceed hardware threads ({}). May cause over-subscription.",
                 config.num_workers, hw_threads);
  }

  if (config.prefetch_batches < config.num_workers) {
    reusex::warn("Prefetch ({}) < workers ({}). May cause GPU starvation. Recommended: 2-3x workers.",
                 config.prefetch_batches, config.num_workers);
  }

  // Create dataloader with multi-threaded prefetching
  Dataloader loader(*dataset, config.batch_size, config.shuffle, config.num_workers,
                    config.prefetch_batches);

#ifndef NDEBUG
  // INFO: Create and OpenCV window for visualizing the results during
  // development
  cv::namedWindow("Annotation", cv::WINDOW_AUTOSIZE);
#endif

  reusex::info(
      "Starting annotation with {} batches using {} worker threads",
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
