#include "vision/tensor_rt/Backend.hpp"
#include "core/logging.hpp"
#include "vision/IMLBackend.hpp"
#include "vision/tensor_rt/Dataset.hpp"
#include "vision/tensor_rt/Sam3.hpp"
#include "vision/tensor_rt/Sam3p1.hpp"

#include <memory>

namespace reusex::vision::tensor_rt {

std::unique_ptr<IModel> TensorRTBackend::create_model(
    const Model type, const std::filesystem::path &modelPath, bool use_cuda) {
  // TensorRT always uses GPU, so use_cuda parameter is ignored
  (void)use_cuda; // Suppress unused parameter warning
  reusex::info("Creating TensorRT model type {} from path: {}",
               static_cast<int>(type), modelPath);
  switch (type) {
  case Model::sam3:
    return TensorRTSam3::create(modelPath);
  case Model::sam3p1:
    // SAM 3.1 is a stateful tracker; it must be driven through the video path.
    reusex::error("SAM3.1 tracker requires the video path; use "
                  "create_video_model / --video");
    throw std::runtime_error("SAM3.1 tracker requires the video path; use "
                             "create_video_model / --video");
  default:
    reusex::error("Unsupported model type: {}", static_cast<int>(type));
    throw std::runtime_error("Unsupported model type");
  }
}

std::unique_ptr<IVideoModel> TensorRTBackend::create_video_model(
    const Model type, const std::filesystem::path &modelPath, bool use_cuda) {
  // TensorRT always uses GPU, so use_cuda parameter is ignored.
  (void)use_cuda;
  reusex::info("Creating TensorRT video model type {} from path: {}",
               static_cast<int>(type), modelPath);
  switch (type) {
  case Model::sam3p1:
    return TensorRTSam3p1::create(modelPath);
  default:
    reusex::error("Unsupported video model type: {}", static_cast<int>(type));
    throw std::runtime_error("Unsupported video model type");
  }
}

std::unique_ptr<IDataset>
TensorRTBackend::create_dataset(const std::filesystem::path &datasetPath) {
  reusex::info("Creating TensorRT dataset from path: {}", datasetPath);
  return std::make_unique<TensorRTDataset>(datasetPath);
}

} // namespace reusex::vision::tensor_rt
