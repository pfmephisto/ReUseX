// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ReUseX/vision/BackendFactory.hpp>
#include <ReUseX/vision/Dataloader.hpp>
// #include <ReUseX/vision/Dataset.hpp>
#include <ReUseX/vision/annotate.hpp>
// #include <ReUseX/vision/infer/sam3infer.hpp>
#include <ReUseX/vision/utils.hpp>

// #include <fmt/core.h>
// #include <fmt/ranges.h>
// #include <spdmon/spdmon.hpp>

#include <opencv2/core.hpp>
// For visualization/debugging purposes
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/common/colors.h>

#include <spdlog/spdlog.h>
#include <spdmon/spdmon.hpp>

#include <range/v3/all.hpp>
namespace ReUseX::vision {

auto annotate_NEW(const std::filesystem::path &dbPath,
                  const std::filesystem::path &modelPath) -> int {
  spdlog::trace("calling annotateRT-RTABMap");

  // Create backend, model, and dataset
  auto backendType = BackendFactory::detect_backend(modelPath);
  auto backend = BackendFactory::create(backendType);

  auto model = backend->createModel(ReUseX::vision::Model::Sam3, modelPath);
  auto dataset = backend->createDataset(dbPath);

  // Create dataloader with multi-threaded prefetching
  // Parameters: dataset, batch_size, shuffle, num_workers, prefetch_batches
  constexpr size_t batch_size = 4;
  constexpr bool shuffle = false;
  constexpr size_t num_workers = 4;
  constexpr size_t prefetch_batches = 2;

  Dataloader loader(*dataset, batch_size, shuffle, num_workers,
                    prefetch_batches);

  spdlog::info("Starting annotation with {} batches using {} worker threads",
               loader.size(), loader.get_num_workers());

  size_t batch_count = 0;
  // for (auto batch : loader) {
  for (auto [logger, batch] : spdmon::LogProgress(loader)) {
    spdlog::trace("Processing batch {}/{} with {} items", ++batch_count,
                  loader.size(), batch.size());
    //  auto results = model->forward(batch);
    //  dataset->save(results, batch);
  }

  spdlog::info("Annotation completed");
  return 0;
}

} // namespace ReUseX::vision
