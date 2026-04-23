// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "geometry/CoplanarPolygon.hpp"

#include <cstring>
#include <limits>
#include <stdexcept>

namespace reusex::geometry {

double CoplanarPolygon::area() const {
  if (vertices.size() < 3)
    return 0.0;

  // Newell's method: compute cross product sum, project onto plane normal
  Eigen::Vector3d cross_sum = Eigen::Vector3d::Zero();
  const size_t n = vertices.size();
  for (size_t i = 0; i < n; ++i) {
    const auto &curr = vertices[i];
    const auto &next = vertices[(i + 1) % n];
    cross_sum += curr.cross(next);
  }

  Eigen::Vector3d n_vec = plane.head<3>();
  double n_len = n_vec.norm();
  if (n_len < 1e-15)
    return 0.0;

  return 0.5 * std::abs(cross_sum.dot(n_vec) / n_len);
}

Eigen::Vector3d CoplanarPolygon::centroid() const {
  if (vertices.empty())
    return Eigen::Vector3d::Zero();

  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (const auto &v : vertices)
    sum += v;
  return sum / static_cast<double>(vertices.size());
}

Eigen::Vector3d CoplanarPolygon::normal() const {
  Eigen::Vector3d n = plane.head<3>();
  double len = n.norm();
  if (len < 1e-15)
    return Eigen::Vector3d::Zero();
  return n / len;
}

bool CoplanarPolygon::is_valid() const {
  return vertices.size() >= 3 && plane.head<3>().squaredNorm() > 1e-30;
}

std::pair<Eigen::Vector3d, Eigen::Vector3d>
CoplanarPolygon::bounding_box() const {
  if (vertices.empty())
    return {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

  Eigen::Vector3d lo = vertices.front();
  Eigen::Vector3d hi = vertices.front();
  for (size_t i = 1; i < vertices.size(); ++i) {
    lo = lo.cwiseMin(vertices[i]);
    hi = hi.cwiseMax(vertices[i]);
  }
  return {lo, hi};
}

std::vector<uint8_t> CoplanarPolygon::serialize_vertices() const {
  static constexpr size_t BYTES_PER_VERTEX = 3 * sizeof(double); // 24
  std::vector<uint8_t> buf(vertices.size() * BYTES_PER_VERTEX);
  auto *dst = buf.data();
  for (const auto &v : vertices) {
    std::memcpy(dst, v.data(), BYTES_PER_VERTEX);
    dst += BYTES_PER_VERTEX;
  }
  return buf;
}

std::vector<Eigen::Vector3d>
CoplanarPolygon::deserialize_vertices(const void *data, size_t size) {
  static constexpr size_t BYTES_PER_VERTEX = 3 * sizeof(double);
  if (size % BYTES_PER_VERTEX != 0)
    throw std::runtime_error(
        "Invalid vertex data size: not a multiple of 24 bytes");

  size_t count = size / BYTES_PER_VERTEX;
  std::vector<Eigen::Vector3d> verts(count);
  auto *src = static_cast<const uint8_t *>(data);
  for (size_t i = 0; i < count; ++i) {
    std::memcpy(verts[i].data(), src, BYTES_PER_VERTEX);
    src += BYTES_PER_VERTEX;
  }
  return verts;
}

} // namespace reusex::geometry
