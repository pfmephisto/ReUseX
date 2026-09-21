// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "vision/glass_filter.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace reusex::vision {

namespace {

// Strip the optional :<float> threshold suffix from a prompt spec and return
// the concept text.  Mirrors parse_prompt_spec in tensor_rt/Dataset.cpp.
std::string concept_text(const std::string &spec) {
  if (auto pos = spec.rfind(':');
      pos != std::string::npos && pos + 1 < spec.size()) {
    try {
      size_t used = 0;
      float v = std::stof(spec.substr(pos + 1), &used);
      if (used == spec.size() - pos - 1 && v >= 0.0f && v <= 1.0f)
        return spec.substr(0, pos);
    } catch (const std::exception &) {
    }
  }
  return spec;
}

} // namespace

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

std::string
find_duplicate_prompt_concept(const std::vector<std::string> &prompts) {
  std::vector<std::string> seen;
  for (const auto &p : prompts) {
    auto text = concept_text(p);
    if (std::find(seen.begin(), seen.end(), text) != seen.end())
      return text;
    seen.push_back(std::move(text));
  }
  return {};
}

} // namespace reusex::vision
