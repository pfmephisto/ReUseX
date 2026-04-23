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

#include <algorithm>
#include <cctype>
#include <filesystem>

namespace reusex::vision {
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
  /* Detects the model type from the model path.
   *
   * Inspects the path stem and, for directories, contained file names.
   * Detection is case-insensitive:
   *   - "sam3" or "sam2" in the name  -> Model::sam3
   *   - directory containing "vision-encoder.*" -> Model::sam3
   *   - otherwise                     -> Model::yolo
   *
   * @param model_path: The file or directory path of the model.
   * @return The detected model type.
   */
  static Model detect_model(const std::filesystem::path &model_path) {
    auto to_lower = [](std::string s) {
      std::transform(s.begin(), s.end(), s.begin(),
                     [](unsigned char c) { return std::tolower(c); });
      return s;
    };

    // Check directory/file name itself
    auto name = to_lower(model_path.stem().string());
    if (name.find("sam3") != std::string::npos ||
        name.find("sam2") != std::string::npos) {
      reusex::info("Detected SAM3 model from path name: {}", model_path);
      return Model::sam3;
    }

    // For directories, check contained filenames for SAM3 sub-models
    if (std::filesystem::is_directory(model_path)) {
      for (const auto &entry :
           std::filesystem::directory_iterator(model_path)) {
        auto stem = to_lower(entry.path().stem().string());
        if (stem.find("vision-encoder") != std::string::npos) {
          reusex::info(
              "Detected SAM3 model from sub-model file: {}", entry.path());
          return Model::sam3;
        }
      }
    }

    reusex::info("Defaulting to YOLO model type for path: {}",
                       model_path);
    return Model::yolo;
  }

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
      reusex::error("OpenCV backend is not implemented yet.");
      throw std::runtime_error("OpenCV backend not implemented");

    case Backend::tensor_rt:
#ifdef REUSEX_USE_TENSORRT
      return std::make_unique<reusex::vision::tensor_rt::TensorRTBackend>();
#else
      reusex::error("TensorRT backend not compiled in this build. "
                          "Rebuild with -DML_BACKENDS=TensorRT or AUTO.");
      throw std::runtime_error("TensorRT backend not available");
#endif

    case Backend::libtorch:
#ifdef REUSEX_USE_LIBTORCH
      return std::make_unique<reusex::vision::libtorch::LibTorchBackend>();
#else
      reusex::error("LibTorch backend not compiled in this build. "
                          "Rebuild with -DML_BACKENDS=LibTorch or AUTO.");
      throw std::runtime_error("LibTorch backend not available");
#endif

    case Backend::dnn:
      reusex::error("DNN backend is not implemented yet.");
      throw std::runtime_error("DNN backend not implemented");

    case Backend::onnx_runtime:
#ifdef REUSEX_USE_ONNX
      return std::make_unique<reusex::vision::onnx::ONNXBackend>();
#else
      reusex::error("ONNX Runtime backend is not implemented yet.");
      throw std::runtime_error("ONNX Runtime backend not implemented");
#endif

    case Backend::openvino:
#ifdef REUSEX_USE_OPENVINO
      return std::make_unique<reusex::vision::openvino::OpenVINOBackend>();
#else
      reusex::error("OpenVINO backend is not implemented yet.");
      throw std::runtime_error("OpenVINO backend not implemented");
#endif

    default:
      reusex::error("Unsupported backend type: {}",
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
      reusex::warn("File {} has no extension. Unable to detect backend.",
                         file_path);
      return Backend::unknown;
    }

    else if (ext == ".engine") {
      reusex::info("Detected TensorRT engine file: {}", file_path);
#ifndef REUSEX_USE_TENSORRT
      reusex::warn("TensorRT detected but not compiled in this build. Backend unavailable.");
#endif
      return Backend::tensor_rt;
    } else if (ext == ".pt" || ext == ".pth" || ext == ".torchscript") {
      reusex::info("Detected PyTorch model file: {}", file_path);
#ifndef REUSEX_USE_LIBTORCH
      reusex::warn("LibTorch detected but not compiled in this build. Backend unavailable.");
#endif
      return Backend::libtorch;
    } else if (ext == ".onnx") {
      reusex::info("Detected ONNX model file: {}", file_path);
#ifndef REUSEX_USE_ONNX
      reusex::warn("ONNX Runtime detected but not compiled in this build. Backend unavailable.");
#endif
      return Backend::onnx_runtime;
    } else if (ext == ".xml" || ext == ".bin") {
      reusex::info("Detected OpenVINO model files: {}", file_path);
#ifndef REUSEX_USE_OPENVINO
      reusex::warn("OpenVINO detected but not compiled in this build. Backend unavailable.");
#endif
      return Backend::openvino;
    }

    reusex::warn(
        "Unknown model file extension: {}. Unable to detect backend; returning Backend::unknown.",
        ext);
    return Backend::unknown;
  }
};
} // namespace reusex::vision
