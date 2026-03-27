#include "core/logging.hpp"
#include "vision/IMLBackend.hpp"
#include "vision/tensor_rt/Backend.hpp"
#include "vision/tensor_rt/Dataset.hpp"
#include "vision/tensor_rt/Sam3.hpp"

#include <memory>

namespace ReUseX::vision::tensor_rt {

std::unique_ptr<IModel>
TensorRTBackend::create_model(const Model type,
                              const std::filesystem::path &modelPath) {
  ReUseX::core::info("Creating TensorRT model type {} from path: {}",
                     static_cast<int>(type), modelPath);
  switch (type) {
  case Model::sam3:
    return TensorRTSam3::create(modelPath);
  default:
    ReUseX::core::error("Unsupported model type: {}", static_cast<int>(type));
    throw std::runtime_error("Unsupported model type");
  }
}

std::unique_ptr<IDataset>
TensorRTBackend::create_dataset(const std::filesystem::path &datasetPath) {
  ReUseX::core::info("Creating TensorRT dataset from path: {}", datasetPath);
  return std::make_unique<TensorRTDataset>(datasetPath);
}

} // namespace ReUseX::vision::tensor_rt
