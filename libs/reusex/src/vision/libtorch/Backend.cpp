// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/logging.hpp"
#include "vision/IMLBackend.hpp"
#include "vision/libtorch/Backend.hpp"
#include "vision/libtorch/Dataset.hpp"
#include "vision/libtorch/Yolo.hpp"

#include <memory>

namespace ReUseX::vision::libtorch {

std::unique_ptr<IModel>
LibTorchBackend::create_model(const Model type,
                              const std::filesystem::path &modelPath,
                              bool use_cuda) {
  ReUseX::info("Creating LibTorch model type {} from path: {}",
                     static_cast<int>(type), modelPath);
  switch (type) {
  case Model::yolo:
    return LibTorchYolo::create(modelPath, use_cuda);
  default:
    ReUseX::error("Unsupported model type for LibTorch backend: {}",
                        static_cast<int>(type));
    throw std::runtime_error("Unsupported model type for LibTorch backend");
  }
}

std::unique_ptr<IDataset>
LibTorchBackend::create_dataset(const std::filesystem::path &datasetPath) {
  ReUseX::info("Creating LibTorch dataset from path: {}", datasetPath);
  return std::make_unique<LibTorchDataset>(datasetPath);
}

} // namespace ReUseX::vision::libtorch
