// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/logging.hpp"
#include "vision/IMLBackend.hpp"
#include "vision/onnx/Backend.hpp"
#include "vision/onnx/Sam3.hpp"
#include "vision/onnx/Sam3Dataset.hpp"

#include <memory>

namespace ReUseX::vision::onnx {

std::unique_ptr<IModel>
ONNXBackend::create_model(const Model type,
                          const std::filesystem::path &modelPath,
                          bool use_cuda) {
  ReUseX::core::info("Creating ONNX Runtime model type {} from path: {}",
                     static_cast<int>(type), modelPath);
  switch (type) {
  case Model::sam3:
    return ONNXSam3::create(modelPath, use_cuda);
  default:
    ReUseX::core::error("Unsupported model type for ONNX backend: {}",
                        static_cast<int>(type));
    throw std::runtime_error("Unsupported model type for ONNX backend");
  }
}

std::unique_ptr<IDataset>
ONNXBackend::create_dataset(const std::filesystem::path &datasetPath) {
  ReUseX::core::info("Creating ONNX dataset from path: {}", datasetPath);
  return std::make_unique<ONNXSam3Dataset>(datasetPath);
}

} // namespace ReUseX::vision::onnx
