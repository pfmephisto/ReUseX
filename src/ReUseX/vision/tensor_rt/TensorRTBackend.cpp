#include <ReUseX/vision/IMLBackend.hpp>
#include <ReUseX/vision/tensor_rt/TensorRTBackend.hpp>
#include <ReUseX/vision/tensor_rt/TensorRTDataset.hpp>
#include <ReUseX/vision/tensor_rt/TensorRTSam3.hpp>
#include <memory>
#include <spdlog/spdlog.h>

namespace ReUseX::vision::tensor_rt {

std::unique_ptr<IModel>
TensorRTBackend::createModel(const Model type,
                             const std::filesystem::path &modelPath) {
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
  return std::make_unique<TensorRTDataset>(datasetPath);
}

} // namespace ReUseX::vision::tensor_rt
