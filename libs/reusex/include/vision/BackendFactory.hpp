#pragma once
#include "reusex/core/logging.hpp"
#include "reusex/vision/IMLBackend.hpp"

#ifdef REUSEX_USE_LIBTORCH
#include "reusex/vision/libtorch/Backend.hpp"
#endif

#ifdef REUSEX_USE_TENSORRT
#include "reusex/vision/tensor_rt/Backend.hpp"
#endif

#ifdef REUSEX_USE_ONNX
#include "reusex/vision/onnx/Backend.hpp"
#endif

#ifdef REUSEX_USE_OPENVINO
#include "reusex/vision/openvino/Backend.hpp"
#endif

#include <fmt/std.h>

#include <filesystem>

namespace ReUseX::vision {
enum class Backend {
  opencv,
  tensor_rt,
  libtorch,
  dnn,
  onnx_runtime,
  openvino,
  unknown,
};

class BackendFactory {
    public:
  /* Detects the appropriate backend based on the model path.
   * @param model_path: The file or directory path of the model.
   * @return The detected backend type.
   */
  static Backend detect_backend(const std::filesystem::path &model_path) {
    using namespace std::filesystem;

    if (is_regular_file(model_path))
      return detect_backend_from_file(model_path);

    if (is_directory(model_path))
      for (const auto &entry : directory_iterator(model_path))
        if (entry.is_regular_file())
          if (auto backend = detect_backend_from_file(entry.path());
              backend != Backend::unknown)
            return backend;

    return Backend::unknown;
  }

  /* Creates an instance of the specified backend type.
   * @param type: The backend type to create.
   * @return A unique pointer to the created backend instance.
   * @throws std::runtime_error if the backend type is not implemented or
   * unsupported.
   */
  static std::unique_ptr<IMLBackend> create(Backend type) {
    switch (type) {
    case Backend::opencv:
      ReUseX::core::error("OpenCV backend is not implemented yet.");
      throw std::runtime_error("OpenCV backend not implemented");

    case Backend::tensor_rt:
#ifdef REUSEX_USE_TENSORRT
      return std::make_unique<ReUseX::vision::tensor_rt::TensorRTBackend>();
#else
      ReUseX::core::error("TensorRT backend not compiled in this build. "
                          "Rebuild with -DML_BACKENDS=TensorRT or AUTO.");
      throw std::runtime_error("TensorRT backend not available");
#endif

    case Backend::libtorch:
#ifdef REUSEX_USE_LIBTORCH
      return std::make_unique<ReUseX::vision::libtorch::LibTorchBackend>();
#else
      ReUseX::core::error("LibTorch backend not compiled in this build. "
                          "Rebuild with -DML_BACKENDS=LibTorch or AUTO.");
      throw std::runtime_error("LibTorch backend not available");
#endif

    case Backend::dnn:
      ReUseX::core::error("DNN backend is not implemented yet.");
      throw std::runtime_error("DNN backend not implemented");

    case Backend::onnx_runtime:
#ifdef REUSEX_USE_ONNX
      return std::make_unique<ReUseX::vision::onnx::ONNXBackend>();
#else
      ReUseX::core::error("ONNX Runtime backend is not implemented yet.");
      throw std::runtime_error("ONNX Runtime backend not implemented");
#endif

    case Backend::openvino:
#ifdef REUSEX_USE_OPENVINO
      return std::make_unique<ReUseX::vision::openvino::OpenVINOBackend>();
#else
      ReUseX::core::error("OpenVINO backend is not implemented yet.");
      throw std::runtime_error("OpenVINO backend not implemented");
#endif

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
   * @return The detected backend type or unknown if the extension is not
   * recognized.
   */
  static Backend
  detect_backend_from_file(const std::filesystem::path &file_path) {
    auto ext = file_path.extension();

    if (ext.empty()) {
      ReUseX::core::warn("File {} has no extension. Unable to detect backend.",
                         file_path);
      return Backend::unknown;
    }

    else if (ext == ".engine") {
      ReUseX::core::info("Detected TensorRT engine file: {}", file_path);
#ifndef REUSEX_USE_TENSORRT
      ReUseX::core::warn("TensorRT detected but not compiled in this build. Backend unavailable.");
#endif
      return Backend::tensor_rt;
    } else if (ext == ".pt" || ext == ".pth" || ext == ".torchscript") {
      ReUseX::core::info("Detected PyTorch model file: {}", file_path);
#ifndef REUSEX_USE_LIBTORCH
      ReUseX::core::warn("LibTorch detected but not compiled in this build. Backend unavailable.");
#endif
      return Backend::libtorch;
    } else if (ext == ".onnx") {
      ReUseX::core::info("Detected ONNX model file: {}", file_path);
#ifndef REUSEX_USE_ONNX
      ReUseX::core::warn("ONNX Runtime detected but not compiled in this build. Backend unavailable.");
#endif
      return Backend::onnx_runtime;
    } else if (ext == ".xml" || ext == ".bin") {
      ReUseX::core::info("Detected OpenVINO model files: {}", file_path);
#ifndef REUSEX_USE_OPENVINO
      ReUseX::core::warn("OpenVINO detected but not compiled in this build. Backend unavailable.");
#endif
      return Backend::openvino;
    }

    ReUseX::core::warn(
        "Unknown model file extension: {}. Unable to detect backend; returning Backend::unknown.",
        ext);
    return Backend::unknown;
  }
};
} // namespace ReUseX::vision
