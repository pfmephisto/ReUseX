// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// PLY header inspection for INRIA-format 3D Gaussian Splatting files (#322).
//
// A trained splat is opaque bytes to everything downstream — `ProjectDB`
// stores it, the GUI streams it, the browser renders it — but *something* has
// to establish that the bytes really are a splat before any of that happens.
// A point cloud written by `rux export ply` is a perfectly valid PLY, and
// handing one to a splat renderer produces an empty viewport and no clue why.
//
// This header is that check, and it lives in `core` (not in the app or in
// `gsplat`) because `ProjectDB::save_gaussian_splat()` is the choke point: the
// file is validated on the way *into* the project, so a stored splat is a
// splat by construction and no reader has to re-litigate it.
//
// Deliberately dependency-free: no PCL, no OpenCV, no sqlite. Just the ASCII
// header at the front of the file.

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace reusex::core {

/// What the PLY header of an INRIA 3D Gaussian Splatting file says.
struct GaussianSplatHeader {
  std::uint64_t gaussian_count = 0;
  /// Spherical-harmonic degree, derived from the number of `f_rest_*`
  /// properties: 3 * ((degree + 1)^2 - 1) of them.
  int sh_degree = 0;
};

/// How many bytes from the head of a file are enough to find `end_header`.
///
/// An INRIA header is ~1.5 KiB (62 properties at degree 3). 64 KiB is room for
/// an implementation with comments and then some, while bounding the read of a
/// file that turns out not to be a PLY at all.
inline constexpr std::size_t kGaussianSplatHeaderProbeBytes = 64 * 1024;

/// Parse the PLY header in @p head and verify it is an INRIA splat file.
///
/// @param head The first bytes of the file, at least up to `end_header`
///             (see @ref kGaussianSplatHeaderProbeBytes).
/// @throws std::runtime_error when @p head is not a PLY, is truncated before
///         `end_header`, is `binary_big_endian`, or is a PLY without the
///         Gaussian properties — the last being the interesting case, since
///         `rux export ply` writes a perfectly valid PLY that no splat
///         renderer can draw. The message is user-facing: it names what is
///         missing and which command produces the right file.
GaussianSplatHeader parse_gaussian_splat_ply_header(std::string_view head);

} // namespace reusex::core
