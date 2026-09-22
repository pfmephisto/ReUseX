// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <cstdint>

namespace reusex::geometry {

/// Spread one 10-bit value into a 30-bit word (bit k → position 3k).
///
/// Combine three axes with shifts of 0, 1, 2 before OR-ing:
///   morton30 = expand3(xi) | (expand3(yi) << 1) | (expand3(zi) << 2)
inline uint32_t morton_expand3(uint32_t v) {
  v &= 0x000003ffu;
  v = (v | (v << 16u)) & 0x030000ffu;
  v = (v | (v << 8u)) & 0x0300f00fu;
  v = (v | (v << 4u)) & 0x030c30c3u;
  v = (v | (v << 2u)) & 0x09249249u;
  return v;
}

/// Reverse the 30 significant bits of a Morton code.
///
/// Moves the coarsest octant bits (top bits of plain Morton) to the bottom of
/// the result. Any prefix of a cloud sorted by this key visits all octants
/// before refining any single one (the spatial-stratification property).
inline uint32_t morton_reverse_bits30(uint32_t v) {
  v = ((v >> 1u) & 0x55555555u) | ((v & 0x55555555u) << 1u);
  v = ((v >> 2u) & 0x33333333u) | ((v & 0x33333333u) << 2u);
  v = ((v >> 4u) & 0x0f0f0f0fu) | ((v & 0x0f0f0f0fu) << 4u);
  v = ((v >> 8u) & 0x00ff00ffu) | ((v & 0x00ff00ffu) << 8u);
  v = (v >> 16u) | (v << 16u);
  return v >> 2u;
}

} // namespace reusex::geometry
