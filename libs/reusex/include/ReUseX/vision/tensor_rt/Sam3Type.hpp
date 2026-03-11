#pragma once
#include <array>
#include <opencv2/opencv.hpp>
#include <string>
#include <utility>
#include <vector>

namespace ReUseX::vision::tensor_rt {

// Define BoxPrompt: <Label("pos"/"neg"), {x1, y1, x2, y2}>
using BoxPrompt = std::pair<std::string, std::array<float, 4>>;

// Single prompt unit: contains a piece of text and an optional set of boxes
struct Sam3PromptUnit {
  std::string text;
  std::vector<BoxPrompt> boxes;
  Sam3PromptUnit() = default;
  Sam3PromptUnit(const std::string &t, const std::vector<BoxPrompt> &b = {})
      : text(t), boxes(b) {}
};

// Unified input struct
struct Sam3Input {
  float confidence_threshold;
  cv::Mat image; // Required: input image
  std::vector<Sam3PromptUnit>
      prompts; // Required: all prompt words list corresponding to this image
  Sam3Input() = default;
  Sam3Input(const cv::Mat &img) : image(img) {}
  Sam3Input(const cv::Mat &img, const std::vector<Sam3PromptUnit> &p,
            float conf)
      : image(img), prompts(p), confidence_threshold(conf) {}
};
} // namespace ReUseX::vision::tensor_rt
