// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstddef>
#include <tuple>

namespace reusex::visualize {

/// Normalized bounds of a single viewport within a render window.
///
/// All values are in [0, 1] normalized display coordinates, where (left, top)
/// is one corner and (right, bottom) the opposite corner of the rectangle.
struct ViewportBounds {
  float left;
  float top;
  float right;
  float bottom;
};

/// Compute the normalized bounds for one viewport in an N-way split layout.
///
/// Uses recursive binary space partitioning to tile the unit square into
/// @p total non-overlapping viewports. Even counts are split evenly in half
/// (alternating horizontal / vertical), while odd counts give the first
/// viewport a larger (60%) region and recursively split the remainder.
///
/// This is pure geometry (no rendering dependencies) so it can be unit tested
/// independently of any visualization backend.
///
/// @param index Index of the viewport (0-based, must be < total)
/// @param total Total number of viewports (must be >= 1)
/// @param left Left bound of the current area (default 0.0)
/// @param top Top bound of the current area (default 0.0)
/// @param right Right bound of the current area (default 1.0)
/// @param bottom Bottom bound of the current area (default 1.0)
/// @param split_horizontal Whether to split horizontally first (default true)
/// @return ViewportBounds for the requested viewport
ViewportBounds viewport_bounds(std::size_t index, std::size_t total,
                               float left = 0.0f, float top = 0.0f,
                               float right = 1.0f, float bottom = 1.0f,
                               bool split_horizontal = true);

} // namespace reusex::visualize
