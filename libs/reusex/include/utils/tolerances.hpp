// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

namespace reusex {

/// \file tolerances.hpp
/// \brief Unified geometric epsilon / tolerance policy (see docs/STANDARDS.md
/// §4).
///
/// This header is the single source of truth for degenerate-geometry epsilons.
/// It replaces the scattered ad-hoc `1e-6`/`1e-9`/`1e-15` literals that used to
/// live in `geometry/utils.cpp`, `CoplanarPolygon.cpp`,
/// `Solidifier_to_mesh.cpp` and friends.
///
/// ### Which constant to use when
///
/// - **`kEpsilonLengthM`** — absolute length comparisons expressed in *meters*.
///   Use it to decide whether a length/distance/norm is effectively zero
///   (degenerate edge, zero-area face, near-zero direction vector). Because it
///   is absolute and metric, it only makes sense on quantities that live in the
///   scene's metric coordinate frame. Prefer recentering coordinates near the
///   origin (see #216) before applying it so that floating-point resolution is
///   uniform across the scene.
///
/// - **`kEpsilonAngleRad`** — absolute angular comparisons in *radians*. Use it
///   for "is this angle effectively zero / are these two directions parallel"
///   tests when the quantity you have is an angle (e.g. `std::acos(dot)`), not
///   a raw dot product.
///
/// - **`kEpsilonRelative`** — dimensionless / already-normalized comparisons.
///   Use it for dot products of unit vectors, squared norms of normals that are
///   supposed to be unit length, and any ratio/relative test where the operands
///   carry no physical unit. This is the correct guard for divisions by a
///   squared norm (a normalized normal has squared norm ~1, so a value below
///   `kEpsilonRelative` means the "normal" is degenerate).
///
/// - **`kToleranceCoplanarM`** — coplanarity / adjacency tolerance in *meters*.
///   Two planes whose signed offsets differ by less than this (and whose
///   normals are parallel) are considered the same physical surface. Also the
///   default gap under which two disjoint inlier patches count as spatially
///   adjacent.
///
/// ### Rules (STANDARDS §4)
///
/// - No new magic numbers in algorithm code — reach for one of these constants,
///   or add a tunable to the relevant options struct.
/// - Absolute tolerances state their unit (meters) in the name (`...M`) or a
///   comment. Angular tolerances state radians.
/// - When comparing two vectors for parallelism, prefer converting to an angle
///   and using `kEpsilonAngleRad`, or compare `1 - |dot|` against
///   `kEpsilonRelative`; do not invent a per-call-site literal.

/// Absolute length zero-check, in meters. A distance / edge length / vector
/// norm below this is treated as zero (degenerate).
inline constexpr double kEpsilonLengthM = 1e-9;

/// Absolute angular tolerance, in radians. Angles below this are treated as
/// zero; two directions whose subtended angle is below this are parallel.
inline constexpr double kEpsilonAngleRad = 1e-6;

/// Dimensionless / normalized tolerance. Use for dot products of unit vectors,
/// squared norms of (supposedly) unit normals, and relative ratios. This is the
/// guard for divisions by a squared norm.
inline constexpr double kEpsilonRelative = 1e-12;

/// Coplanarity / spatial-adjacency tolerance, in meters. Planes whose parallel
/// offsets differ by less than this are the same surface; inlier patches with a
/// gap below this are adjacent.
inline constexpr double kToleranceCoplanarM = 0.01;

} // namespace reusex
