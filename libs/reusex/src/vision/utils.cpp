// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "vision/utils.hpp"

#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace ReUseX::vision {

float generate_scale(cv::Mat &image, const cv::Size &target_size,
                     bool scale_up) {
  int origin_w = image.cols;
  int origin_h = image.rows;

  int target_h = target_size.height;
  int target_w = target_size.width;

  float ratio_h = static_cast<float>(target_h) / static_cast<float>(origin_h);
  float ratio_w = static_cast<float>(target_w) / static_cast<float>(origin_w);

  float resize_scale =
      scale_up ? std::max(ratio_h, ratio_w) : std::min(ratio_h, ratio_w);

  return resize_scale;
}

float letterbox(cv::Mat &input_image, cv::Mat &output_image,
                const cv::Size &target_size) {
  if (input_image.cols == target_size.width &&
      input_image.rows == target_size.height) {
    if (input_image.data == output_image.data) {
      return 1.;
    } else {
      output_image = input_image.clone();
      return 1.;
    }
  }

  float resize_scale = generate_scale(input_image, target_size, false);
  int new_shape_w = std::round(input_image.cols * resize_scale);
  int new_shape_h = std::round(input_image.rows * resize_scale);
  float padw = (target_size.width - new_shape_w) / 2.;
  float padh = (target_size.height - new_shape_h) / 2.;

  int top = std::round(padh - 0.1);
  int bottom = std::round(padh + 0.1);
  int left = std::round(padw - 0.1);
  int right = std::round(padw + 0.1);

  cv::resize(input_image, output_image, cv::Size(new_shape_w, new_shape_h), 0,
             0, cv::INTER_AREA);

  cv::copyMakeBorder(output_image, output_image, top, bottom, left, right,
                     cv::BORDER_CONSTANT, cv::Scalar(114., 114., 114));
  return resize_scale;
};

float cropbox(cv::Mat &input_image, cv::Mat &output_image,
              const cv::Size &target_size) {

  if (input_image.cols == target_size.width &&
      input_image.rows == target_size.height) {
    if (input_image.data == output_image.data) {
      return 1.;
    } else {
      output_image = input_image.clone();
      return 1.;
    }
  }

  float resize_scale = generate_scale(input_image, target_size, true);
  int new_shape_w = std::round(input_image.cols * resize_scale);
  int new_shape_h = std::round(input_image.rows * resize_scale);

  cv::resize(input_image, output_image, cv::Size(new_shape_w, new_shape_h), 0,
             0, cv::INTER_AREA);

  int x = (output_image.cols - target_size.width) / 2;
  int y = (output_image.rows - target_size.height) / 2;

  cv::Rect roi(x, y, target_size.width, target_size.height);
  output_image = output_image(roi).clone();

  return resize_scale;
};

} // namespace ReUseX::vision
