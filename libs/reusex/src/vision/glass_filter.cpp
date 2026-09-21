// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "vision/glass_filter.hpp"

#include <algorithm>

namespace reusex::vision {

const std::vector<std::string> &glass_prompt_list() {
  static const std::vector<std::string> prompts{
      "glass", "mirror", "window pane", "transparent surface"};
  return prompts;
}

bool is_glass_class(std::string_view class_name) {
  const auto &pl = glass_prompt_list();
  return std::any_of(pl.begin(), pl.end(), [class_name](const std::string &g) {
    return class_name == g;
  });
}

cv::Mat build_glass_confidence_map(const cv::Mat &label_image,
                                   const std::vector<int> &glass_ids) {
  cv::Mat confidence(label_image.size(), CV_8U, cv::Scalar(255));
  if (glass_ids.empty() || label_image.empty())
    return confidence;

  for (int gid : glass_ids) {
    cv::Mat glass_mask;
    cv::compare(label_image, gid, glass_mask, cv::CMP_EQ);
    confidence.setTo(0, glass_mask);
  }
  return confidence;
}

} // namespace reusex::vision
