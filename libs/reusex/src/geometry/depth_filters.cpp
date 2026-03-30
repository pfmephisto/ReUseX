// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "geometry/depth_filters.hpp"
#include "core/logging.hpp"

#include <opencv2/imgproc.hpp>

namespace ReUseX::geometry {

void apply_depth_discontinuity_filter(cv::Mat &depth, cv::Mat &confidence,
                                      float gradient_threshold) {
  if (depth.empty() || depth.channels() != 1)
    return;

  ReUseX::core::trace("Applying depth discontinuity filter (threshold={})",
                       gradient_threshold);

  cv::Mat depth_float;
  if (depth.type() != CV_32F)
    depth.convertTo(depth_float, CV_32F);
  else
    depth_float = depth;

  cv::Mat grad_x, grad_y, grad_magnitude;
  cv::Sobel(depth_float, grad_x, CV_32F, 1, 0, 3);
  cv::Sobel(depth_float, grad_y, CV_32F, 0, 1, 3);
  cv::magnitude(grad_x, grad_y, grad_magnitude);

  cv::Mat valid_mask = grad_magnitude < gradient_threshold;

  depth.setTo(0, ~valid_mask);
  if (!confidence.empty())
    confidence.setTo(0, ~valid_mask);

  int removed = cv::countNonZero(~valid_mask);
  ReUseX::core::debug(
      "Depth discontinuity filter removed {} / {} pixels ({:.1f}%)", removed,
      depth.total(), 100.0 * removed / depth.total());
}

void apply_ray_consistency_filter(cv::Mat &depth, cv::Mat &confidence,
                                  float consistency_threshold) {
  if (depth.empty() || depth.channels() != 1)
    return;

  ReUseX::core::trace("Applying ray consistency filter (threshold={})",
                       consistency_threshold);

  cv::Mat depth_float;
  if (depth.type() != CV_32F)
    depth.convertTo(depth_float, CV_32F);
  else
    depth_float = depth;

  cv::Mat depth_median;
  cv::medianBlur(depth_float, depth_median, 5);

  cv::Mat deviation;
  cv::absdiff(depth_float, depth_median, deviation);

  cv::Mat valid_mask = deviation < consistency_threshold;

  cv::Mat kernel =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
  cv::morphologyEx(valid_mask, valid_mask, cv::MORPH_OPEN, kernel);

  depth.setTo(0, ~valid_mask);
  if (!confidence.empty())
    confidence.setTo(0, ~valid_mask);

  int removed = cv::countNonZero(~valid_mask);
  ReUseX::core::debug(
      "Ray consistency filter removed {} / {} pixels ({:.1f}%)", removed,
      depth.total(), 100.0 * removed / depth.total());
}

} // namespace ReUseX::geometry
