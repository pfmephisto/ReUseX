// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/vision/IData.hpp"
#include "reusex/vision/onnx/Sam3Type.hpp"

#include <opencv2/core/mat.hpp>

#include <vector>

namespace ReUseX::vision::onnx {

/// @brief Data carrier for ONNX Runtime SAM3 inference pipeline.
///
/// Holds the input image, text prompts, confidence threshold, and the output
/// label image for a single sample processed through the ONNX SAM3 model.
struct ONNXSam3Data : IData {
  cv::Mat image; ///< Input image (BGR, HWC) / Output label image (CV_32S)

  /// @brief Text prompts for semantic segmentation classes.
  std::vector<Sam3PromptUnit> prompts = {
      Sam3PromptUnit("ceiling"),
      Sam3PromptUnit("floor"),
      Sam3PromptUnit("wall"),
      Sam3PromptUnit("door frame"),
      Sam3PromptUnit("window"),
      Sam3PromptUnit("radiator"),
      Sam3PromptUnit("table"),
      Sam3PromptUnit("chair"),
      Sam3PromptUnit("shelf"),
      Sam3PromptUnit("bench"),
      Sam3PromptUnit("ceiling lamp"),
      Sam3PromptUnit("desk lamp"),
      Sam3PromptUnit("electrical outlet"),
  };

  float confidence_threshold = 0.5f; ///< Minimum confidence for detections
};

} // namespace ReUseX::vision::onnx
