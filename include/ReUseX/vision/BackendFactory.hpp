#pragma once
#include <ReUseX/vision/IMLBackend.hpp>
#include <filesystem>
#include <spdlog/spdlog.h>

#include <ReUseX/vision/tensor_rt/TensorRTBackend.hpp>

namespace ReUseX::vision {
enum class Backend {
  OpenCV,
  TensorRT,
  libTorch,
  DNN,
  ONNXRuntime,
  OpenVINO,
};

class BackendFactory {
    public:
  /* Detects the appropriate backend based on the model path.
   * @pram model_path: The file or directory path of the model.
   * @return The detected backend type. Currently hardcoded to TensorRT.
   */
  static Backend detect_backend(const std::filesystem::path &model_path) {
    // TODO:: Update this to select backend based on model path
    spdlog::warn("Backend detection is currently hardcoded to TensorRT. Please "
                 "implement proper detection logic.");
    return Backend::TensorRT;
  }

  /* Creates an instance of the specified backend type.
   * @param type: The backend type to create.
   * @return A unique pointer to the created backend instance.
   * @throws std::runtime_error if the backend type is not implemented or
   * unsupported.
   */
  static std::unique_ptr<IMLBackend> create(Backend type) {
    switch (type) {
    case Backend::OpenCV:
      spdlog::error("OpenCV backend is not implemented yet.");
      throw std::runtime_error("OpenCV backend not implemented");
    case Backend::TensorRT:
      return std::make_unique<ReUseX::vision::tensor_rt::TensorRTBackend>();
    case Backend::libTorch:
      spdlog::error("libTorch backend is not implemented yet.");
      throw std::runtime_error("libTorch backend not implemented");
    case Backend::DNN:
      spdlog::error("DNN backend is not implemented yet.");
      throw std::runtime_error("DNN backend not implemented");
    case Backend::ONNXRuntime:
      spdlog::error("ONNXRuntime backend is not implemented yet.");
      throw std::runtime_error("ONNXRuntime backend not implemented");
    case Backend::OpenVINO:
      spdlog::error("OpenVINO backend is not implemented yet.");
      throw std::runtime_error("OpenVINO backend not implemented");
    default:
      spdlog::error("Unsupported backend type: {}", static_cast<int>(type));
      throw std::runtime_error("Unsupported backend");
    }
  }
};
} // namespace ReUseX::vision
