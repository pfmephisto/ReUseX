// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <utility>
#include <vector>

namespace ReUseX::geometry {

/// A closed 3D polygon whose vertices are coplanar.
///
/// Vertices are ordered; the polygon is implicitly closed (last→first).
/// The plane is stored as Hessian normal form [a,b,c,d]: ax+by+cz+d=0.
struct CoplanarPolygon {
  std::vector<Eigen::Vector3d> vertices;
  Eigen::Vector4d plane{0, 0, 0, 0};

  /// Signed area via Newell's method projected onto the plane normal.
  double area() const;

  /// Centroid (arithmetic mean of vertices).
  Eigen::Vector3d centroid() const;

  /// Unit normal extracted from plane coefficients.
  Eigen::Vector3d normal() const;

  /// True when >= 3 vertices and nonzero plane normal.
  bool is_valid() const;

  /// Axis-aligned bounding box (min, max).
  std::pair<Eigen::Vector3d, Eigen::Vector3d> bounding_box() const;

  /// Serialize vertices to compact binary (N * 3 * sizeof(double) bytes).
  std::vector<uint8_t> serialize_vertices() const;

  /// Deserialize vertices from compact binary.
  static std::vector<Eigen::Vector3d> deserialize_vertices(const void *data,
                                                           size_t size);
};

} // namespace ReUseX::geometry
