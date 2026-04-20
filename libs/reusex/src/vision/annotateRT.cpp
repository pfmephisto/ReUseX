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

#ifdef NDEBUG
// For visualization/debugging purposes
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#endif

#include <pcl/common/colors.h>

#include <range/v3/all.hpp>
namespace ReUseX::vision {

auto annotate(const std::filesystem::path &dbPath,
              const std::filesystem::path &modelPath,
              bool use_cuda) -> int {
  ReUseX::core::trace("calling annotateRT-RTABMap");

  // Create backend, model, and dataset
  auto backendType = BackendFactory::detect_backend(modelPath);
  auto backend = BackendFactory::create(backendType);

  auto modelType = BackendFactory::detect_model(modelPath);
  auto model = backend->create_model(modelType, modelPath, use_cuda);
  auto dataset = backend->create_dataset(dbPath);

  // Create dataloader with multi-threaded prefetching
  // Parameters: dataset, batch_size, shuffle, num_workers, prefetch_batches
  constexpr size_t batch_size = 1; // 4;
  constexpr bool shuffle = false;
  constexpr size_t num_workers = 1;      // 4;
  constexpr size_t prefetch_batches = 1; // 2;

  Dataloader loader(*dataset, batch_size, shuffle, num_workers,
                    prefetch_batches);

#ifndef NDEBUG
  // INFO: Create and OpenCV window for visualizing the results during
  // development
  cv::namedWindow("Annotation", cv::WINDOW_AUTOSIZE);
#endif

  ReUseX::core::info(
      "Starting annotation with {} batches using {} worker threads",
      loader.size(), loader.get_num_workers());

  size_t batch_count = 0;
  {
    auto observer = ReUseX::core::ProgressObserver(
        ReUseX::core::Stage::annotating_batches, loader.size());
    for (auto batch : loader) {
      // for (auto [logger, batch] : spdmon::LogProgress(loader)) {
      ReUseX::core::trace("Processing batch {}/{} with {} items", ++batch_count,
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

  ReUseX::core::info("Annotation completed");
  return 0;
}

} // namespace ReUseX::vision
