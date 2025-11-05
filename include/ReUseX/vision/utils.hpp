// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <torch/torch.h>

#include <opencv4/opencv2/core/mat.hpp>
#include <opencv4/opencv2/core/types.hpp>

namespace ReUseX::vision {
float generate_scale(cv::Mat &image, const cv::Size &target_size,
                     bool scale_up = false);
float letterbox(cv::Mat &input_image, cv::Mat &output_image,
                const cv::Size &target_size);
float cropbox(cv::Mat &input_image, cv::Mat &output_image,
              const cv::Size &target_size);

torch::Tensor xyxy2xywh(const torch::Tensor &x);
torch::Tensor xywh2xyxy(const torch::Tensor &x);
torch::Tensor nms(const torch::Tensor &bboces, const torch::Tensor &scores,
                  float iou_threshold = 0.45);

torch::Tensor non_max_suppression(torch::Tensor &predictions,
                                  float confThreshold = 0.25,
                                  float iouThreshold = 0.45,
                                  int maxDetections = 300);
} // namespace ReUseX::vision
