#pragma once
#include <opencv2/opencv.hpp>

#include <array>
#include <string>
#include <utility>
#include <vector>

namespace reusex::vision::tensor_rt {

// Define BoxPrompt: <Label("pos"/"neg"), {x1, y1, x2, y2}>
using BoxPrompt = std::pair<std::string, std::array<float, 4>>;

// Single prompt unit: contains a piece of text and an optional set of boxes
struct Sam3PromptUnit {
  std::string text;
  std::vector<BoxPrompt> boxes;
  // Per-prompt detection confidence threshold. Negative => use the frame/global
  // threshold (TensorRTData::confidence_threshold). Lets each concept be tuned
  // independently (e.g. a low threshold for a hard-to-see "electrical outlet",
  // a high one for a noisy "person").
  float confidence = -1.0f;
  Sam3PromptUnit() = default;
  explicit Sam3PromptUnit(const std::string &t,
                          const std::vector<BoxPrompt> &b = {},
                          float conf = -1.0f)
      : text(t), boxes(b), confidence(conf) {}
};

// Unified input struct
struct Sam3Input {
  float confidence_threshold = 0.0f;
  cv::Mat image; // Required: input image
  std::vector<Sam3PromptUnit>
      prompts; // Required: all prompt words list corresponding to this image
  Sam3Input() = default;
  explicit Sam3Input(const cv::Mat &img) : image(img) {}
  Sam3Input(const cv::Mat &img, const std::vector<Sam3PromptUnit> &p,
            float conf)
      : image(img), prompts(p), confidence_threshold(conf) {}
};
} // namespace reusex::vision::tensor_rt
