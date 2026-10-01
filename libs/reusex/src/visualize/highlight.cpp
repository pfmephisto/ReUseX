// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/visualize/highlight.hpp"

#include <stdexcept>
#include <string>

namespace reusex::visualize {

std::size_t apply_instance_highlight(std::vector<unsigned char> &rgb,
                                     const std::vector<std::uint32_t> &labels,
                                     std::uint32_t instance_id) {
  if (rgb.size() != labels.size() * 3)
    throw std::invalid_argument(
        "apply_instance_highlight: " + std::to_string(rgb.size() / 3) +
        " coloured points but " + std::to_string(labels.size()) + " labels");
  std::size_t hit = 0;
  for (std::size_t i = 0; i < labels.size(); ++i) {
    unsigned char *c = &rgb[3 * i];
    if (labels[i] == instance_id) {
      c[0] = kHighlightRgb[0];
      c[1] = kHighlightRgb[1];
      c[2] = kHighlightRgb[2];
      ++hit;
    } else {
      for (int k = 0; k < 3; ++k)
        c[k] = static_cast<unsigned char>(c[k] * kHighlightDimFactor);
    }
  }
  return hit;
}

} // namespace reusex::visualize
