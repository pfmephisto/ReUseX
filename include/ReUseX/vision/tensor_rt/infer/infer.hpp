#pragma once
#include <ReUseX/vision/common/object.hpp>
#include <ReUseX/vision/tensor_rt/infer/sam3type.hpp>
#include <array>
#include <iostream>
#include <memory>
#include <vector>

namespace ReUseX::vision::tensor_rt {

/// @brief Convenience namespace alias so existing tensor_rt code can continue
/// to reference types via `object::DetectionBox` etc.
namespace object = ReUseX::vision::common::object;

using InferResult = object::DetectionBoxArray;
using InferResultArray = std::vector<object::DetectionBoxArray>;

class InferBase {
    public:
  virtual ~InferBase() = default;

  // -------------------------------------------------------
  // Unified entry: whether single image, multiple images, with boxes, with
  // text, all go here
  // -------------------------------------------------------

  // Batch inference (Core API)
  virtual InferResultArray forwards(const std::vector<Sam3Input> &inputs,
                                    bool return_mask = false,
                                    void *stream = nullptr) = 0;
  virtual InferResultArray forwards(const std::vector<Sam3Input> &inputs,
                                    const std::string &geom_label,
                                    bool return_mask = false,
                                    void *stream = nullptr) = 0;

  // Single inference (Wrapper)
  virtual InferResult forward(const Sam3Input &input, bool return_mask = false,
                              void *stream = nullptr) {
    return forwards({input}, return_mask, stream)[0];
  }

  // Preset text Token (for Tokenizer cache)
  virtual void
  setup_text_inputs(const std::string &input_text,
                    const std::array<int64_t, 32> &input_ids,
                    const std::array<int64_t, 32> &attention_mask) {}

  // Pre-input geometry model boxes, try to use boxes from image A to recognize
  // categories in image B
  virtual bool setup_geometry_input(
      const cv::Mat &image, const std::string &label,
      const std::vector<std::pair<std::string, std::array<float, 4>>> &boxes) {
    return false;
  }
};

// Factory function declaration
std::shared_ptr<InferBase> load(const std::string &vision_encoder_path,
                                const std::string &text_encoder_path,
                                const std::string &geometry_encoder_path,
                                const std::string &decoder_path,
                                int gpu_id = 0);
} // namespace ReUseX::vision::tensor_rt
