#pragma once
#include "reusex/core/logging.hpp"
#include "reusex/vision/IMLBackend.hpp"
#include "reusex/vision/tensor_rt/Backend.hpp"

#include <fmt/std.h>

#include <filesystem>

namespace ReUseX::vision {
enum class Backend {
  OpenCV,
  TensorRT,
  libTorch,
  DNN,
  ONNXRuntime,
  OpenVINO,
  Unknown,
};

class BackendFactory {
    public:
  /* Detects the appropriate backend based on the model path.
   * @param model_path: The file or directory path of the model.
   * @return The detected backend type. Currently hardcoded to TensorRT.
   */
  static Backend detect_backend(const std::filesystem::path &model_path) {
    using namespace std::filesystem;

    if (is_regular_file(model_path))
      return detect_backend_from_file(model_path);

    if (is_directory(model_path))
      for (const auto &entry : directory_iterator(model_path))
        if (entry.is_regular_file())
          if (auto backend = detect_backend_from_file(entry.path());
              backend != Backend::Unknown)
            return backend;

    return Backend::Unknown;
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
      ReUseX::core::error("OpenCV backend is not implemented yet.");
      throw std::runtime_error("OpenCV backend not implemented");
    case Backend::TensorRT:
      return std::make_unique<ReUseX::vision::tensor_rt::TensorRTBackend>();
    case Backend::libTorch:
      ReUseX::core::error("libTorch backend is not implemented yet.");
      throw std::runtime_error("libTorch backend not implemented");
    case Backend::DNN:
      ReUseX::core::error("DNN backend is not implemented yet.");
      throw std::runtime_error("DNN backend not implemented");
    case Backend::ONNXRuntime:
      ReUseX::core::error("ONNXRuntime backend is not implemented yet.");
      throw std::runtime_error("ONNXRuntime backend not implemented");
    case Backend::OpenVINO:
      ReUseX::core::error("OpenVINO backend is not implemented yet.");
      throw std::runtime_error("OpenVINO backend not implemented");
    default:
      ReUseX::core::error("Unsupported backend type: {}",
                          static_cast<int>(type));
      throw std::runtime_error("Unsupported backend");
    }
  }

    private:
  /* Helper function to detect backend type from a single file based on its
   * extension.
   * @param file_path: The path of the file to analyze.
   * @return The detected backend type or Unknown if the extension is not
   * recognized.
   */
  static Backend
  detect_backend_from_file(const std::filesystem::path &file_path) {
    auto ext = file_path.extension();

    if (ext.empty()) {
      ReUseX::core::warn("File {} has no extension. Unable to detect backend.",
                         file_path);
      return Backend::Unknown;
    }

    else if (ext == ".engine") {
      ReUseX::core::info("Detected TensorRT engine file: {}", file_path);
      return Backend::TensorRT;
    } else if (ext == ".pt" || ext == ".pth" || ext == ".torchscript") {
      ReUseX::core::info("Detected PyTorch model file: {}", file_path);
      return Backend::libTorch;
    } else if (ext == ".onnx") {
      ReUseX::core::info("Detected ONNX model file: {}", file_path);
      return Backend::ONNXRuntime;
    } else if (ext == ".xml" || ext == ".bin") {
      ReUseX::core::info("Detected OpenVINO model files: {}", file_path);
      return Backend::OpenVINO;
    }

    ReUseX::core::warn(
        "Unknown model file extension: {}. Defaulting to TensorRT.", ext);
    return Backend::Unknown;
  }
};
} // namespace ReUseX::vision
