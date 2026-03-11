#include <ReUseX/vision/IMLBackend.hpp>
#include <ReUseX/vision/tensor_rt/Backend.hpp>
#include <ReUseX/vision/tensor_rt/Dataset.hpp>
#include <ReUseX/vision/tensor_rt/Sam3.hpp>

#include <spdlog/fmt/std.h>
#include <spdlog/spdlog.h>

#include <memory>

namespace ReUseX::vision::tensor_rt {

std::unique_ptr<IModel>
TensorRTBackend::createModel(const Model type,
                             const std::filesystem::path &modelPath) {
  spdlog::info("Creating TensorRT model type {} from path: {}",
               static_cast<int>(type), modelPath);
  switch (type) {
  case Model::Sam3:
    return TensorRTSam3::create(modelPath);
  default:
    spdlog::error("Unsupported model type: {}", static_cast<int>(type));
    throw std::runtime_error("Unsupported model type");
  }
}

std::unique_ptr<IDataset>
TensorRTBackend::createDataset(const std::filesystem::path &datasetPath) {
  spdlog::info("Creating TensorRT dataset from path: {}", datasetPath);
  return std::make_unique<TensorRTDataset>(datasetPath);
}

} // namespace ReUseX::vision::tensor_rt
