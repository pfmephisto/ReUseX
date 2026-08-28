// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Deterministic synthetic scenes shared by integration tests and benchmarks.
// All generators take an explicit seed so results are reproducible
// (docs/STANDARDS.md §6).

#include <reusex/types.hpp>

#include <Eigen/Core>

#include <random>

namespace reusex::test_support {

struct SyntheticScene {
  CloudPtr cloud;
  CloudNPtr normals;
};

/// Sample a rectangular patch into an existing scene.
/// @param origin  corner of the patch
/// @param u,v     edge vectors spanning the patch (length = patch extent)
/// @param normal  outward-facing (into the room) unit normal
/// @param spacing sample spacing in meters
/// @param sigma   Gaussian position noise along the normal, in meters
inline void add_patch(SyntheticScene &scene, const Eigen::Vector3f &origin,
                      const Eigen::Vector3f &u, const Eigen::Vector3f &v,
                      const Eigen::Vector3f &normal, float spacing, float sigma,
                      std::mt19937 &gen) {
  std::normal_distribution<float> noise(0.0F, sigma);
  const int nu = static_cast<int>(u.norm() / spacing);
  const int nv = static_cast<int>(v.norm() / spacing);
  const Eigen::Vector3f du = u / static_cast<float>(nu);
  const Eigen::Vector3f dv = v / static_cast<float>(nv);

  for (int i = 0; i <= nu; ++i) {
    for (int j = 0; j <= nv; ++j) {
      const Eigen::Vector3f p = origin + static_cast<float>(i) * du +
                                static_cast<float>(j) * dv +
                                (sigma > 0.0F ? noise(gen) : 0.0F) * normal;
      PointT pt;
      pt.x = p.x();
      pt.y = p.y();
      pt.z = p.z();
      pt.r = pt.g = pt.b = 200;
      scene.cloud->push_back(pt);

      NormalT n;
      n.normal_x = normal.x();
      n.normal_y = normal.y();
      n.normal_z = normal.z();
      scene.normals->push_back(n);
    }
  }
}

/// A single rectangular room (axis-aligned box interior): floor, ceiling and
/// four walls, sampled at @p spacing with @p sigma position noise.
/// Default dimensions 4m x 3m x 2.5m. Normals face into the room.
inline SyntheticScene make_room(float sx = 4.0F, float sy = 3.0F,
                                float sz = 2.5F, float spacing = 0.05F,
                                float sigma = 0.005F, unsigned seed = 42) {
  std::mt19937 gen(seed);
  SyntheticScene scene{std::make_shared<Cloud>(), std::make_shared<CloudN>()};

  const Eigen::Vector3f ex(sx, 0, 0), ey(0, sy, 0), ez(0, 0, sz);
  const Eigen::Vector3f o(0, 0, 0);

  // floor (z=0, normal up) / ceiling (z=sz, normal down)
  add_patch(scene, o, ex, ey, {0, 0, 1}, spacing, sigma, gen);
  add_patch(scene, o + ez, ex, ey, {0, 0, -1}, spacing, sigma, gen);
  // walls x=0 / x=sx
  add_patch(scene, o, ey, ez, {1, 0, 0}, spacing, sigma, gen);
  add_patch(scene, o + ex, ey, ez, {-1, 0, 0}, spacing, sigma, gen);
  // walls y=0 / y=sy
  add_patch(scene, o, ex, ez, {0, 1, 0}, spacing, sigma, gen);
  add_patch(scene, o + ey, ex, ez, {0, -1, 0}, spacing, sigma, gen);

  scene.cloud->width = static_cast<uint32_t>(scene.cloud->size());
  scene.cloud->height = 1;
  scene.normals->width = static_cast<uint32_t>(scene.normals->size());
  scene.normals->height = 1;
  return scene;
}

} // namespace reusex::test_support
