// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <array>
#include <string>
#include <utility>
#include <vector>

namespace reusex::vision::onnx {

/// @brief A bounding-box prompt: label ("pos"/"neg") and [x1, y1, x2, y2].
using BoxPrompt = std::pair<std::string, std::array<float, 4>>;

/// @brief A single SAM3 prompt unit with text and optional box prompts.
struct Sam3PromptUnit {
  std::string text;
  std::vector<BoxPrompt> boxes;
  Sam3PromptUnit() = default;
  Sam3PromptUnit(const std::string &t, const std::vector<BoxPrompt> &b = {})
      : text(t), boxes(b) {}
};

} // namespace reusex::vision::onnx
