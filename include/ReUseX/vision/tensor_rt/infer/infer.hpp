#pragma once
#include <ReUseX/vision/common/object.hpp>
#include <ReUseX/vision/tensor_rt/infer/sam3type.hpp>
#include <array>
#include <iostream>
#include <memory>
#include <vector>

namespace ReUseX::vision::tensor_rt {

/// @brief Convenience alias for a single image's detection results.
using InferResult = ::ReUseX::vision::common::object::DetectionBoxArray;

/// @brief Convenience alias for a batch of images' detection results.
using InferResultArray =
    std::vector<::ReUseX::vision::common::object::DetectionBoxArray>;

/// @brief Abstract base class for SAM3-compatible inference implementations.
///
/// Concrete subclasses (e.g., Sam3Infer) implement the TensorRT-specific
/// forward pass while sharing this unified interface.
class InferBase {
    public:
  virtual ~InferBase() = default;

  // Batch inference (Core API)

  /// @brief Run inference on a batch of inputs.
  /// @param inputs     Batch of image + prompt units.
  /// @param return_mask When true, each DetectionBox will include a
  ///                   segmentation mask.
  /// @param stream     Optional CUDA stream (nullptr = default stream).
  /// @return Per-image detection results.
  virtual InferResultArray forwards(const std::vector<Sam3Input> &inputs,
                                    bool return_mask = false,
                                    void *stream = nullptr) = 0;

  /// @brief Run inference using a pre-cached geometry label.
  virtual InferResultArray forwards(const std::vector<Sam3Input> &inputs,
                                    const std::string &geom_label,
                                    bool return_mask = false,
                                    void *stream = nullptr) = 0;

  // Single inference (Wrapper)

  /// @brief Convenience single-image forward pass.
  virtual InferResult forward(const Sam3Input &input, bool return_mask = false,
                              void *stream = nullptr) {
    return forwards({input}, return_mask, stream)[0];
  }

  /// @brief Cache tokenized text inputs for repeated use.
  virtual void
  setup_text_inputs(const std::string &input_text,
                    const std::array<int64_t, 32> &input_ids,
                    const std::array<int64_t, 32> &attention_mask) {}

  /// @brief Pre-compute geometry features from a reference image and its
  /// labelled boxes.
  virtual bool setup_geometry_input(
      const cv::Mat &image, const std::string &label,
      const std::vector<std::pair<std::string, std::array<float, 4>>> &boxes) {
    return false;
  }
};

/// @brief Factory: load a SAM3 inference engine from the given TRT engine
/// files.
std::shared_ptr<InferBase> load(const std::string &vision_encoder_path,
                                const std::string &text_encoder_path,
                                const std::string &geometry_encoder_path,
                                const std::string &decoder_path,
                                int gpu_id = 0);
} // namespace ReUseX::vision::tensor_rt
