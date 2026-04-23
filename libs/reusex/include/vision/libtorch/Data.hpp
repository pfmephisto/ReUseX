// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/vision/IData.hpp"

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace reusex::vision::libtorch {

/** @brief Data carrier for LibTorch YOLO inference pipeline.
 *
 * Holds the input image, preprocessing parameters, and output label image
 * for a single sample processed through the LibTorch backend.
 */
struct LibTorchData : IData {
  cv::Mat image;                      ///< Input: raw image (BGR, HWC)
  int target_size = 640;              ///< Letterbox target dimension
  float letterbox_scale = 1.0f;       ///< Scale applied during letterbox
  cv::Size original_size;             ///< Pre-letterbox dimensions
  cv::Mat label_image;                ///< Output: label image (CV_16U, class+1)
  float confidence_threshold = 0.25f; ///< Detection confidence threshold
  float iou_threshold = 0.45f;        ///< NMS IoU threshold
  int max_detections = 300;           ///< Maximum detections per image
};

} // namespace reusex::vision::libtorch
