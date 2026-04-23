// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace reusex::vision {
float generate_scale(cv::Mat &image, const cv::Size &target_size,
                     bool scale_up = false);
float letterbox(cv::Mat &input_image, cv::Mat &output_image,
                const cv::Size &target_size);
float cropbox(cv::Mat &input_image, cv::Mat &output_image,
              const cv::Size &target_size);
} // namespace reusex::vision
