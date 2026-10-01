// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Instance highlight colouring for evidence renders (Kortlægning, #265 Phase
// 2 Task 8): paint one instance's points in a fixed accent colour and dim
// every other point, so a plan/orbit/front render can call out a single
// element the way the frontend's instance picker does.

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace reusex::visualize {

/// Accent colour painted onto the highlighted instance's points.
inline constexpr std::array<unsigned char, 3> kHighlightRgb{255, 176, 64};

/// Brightness multiplier applied to every non-highlighted point's colour.
inline constexpr double kHighlightDimFactor = 0.3;

/// Paint points whose label == instance_id in kHighlightRgb and dim every
/// other point to kHighlightDimFactor of its colour. rgb is 3 bytes per
/// point.
/// @return number of highlighted points.
/// @throws std::invalid_argument on size mismatch.
std::size_t apply_instance_highlight(std::vector<unsigned char> &rgb,
                                     const std::vector<std::uint32_t> &labels,
                                     std::uint32_t instance_id);

} // namespace reusex::visualize
