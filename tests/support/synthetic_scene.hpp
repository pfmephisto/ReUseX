// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Deterministic synthetic scenes shared by integration tests and benchmarks.
// All generators take an explicit seed so results are reproducible
// (docs/STANDARDS.md §6).

#include <reusex/types.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <random>

namespace reusex::test_support {

struct SyntheticScene {
  CloudPtr cloud;
  CloudNPtr normals;
  /// Ground-truth surface labels (1-based, one per generated patch;
  /// 0 = unlabeled). Index-aligned with cloud/normals.
  CloudLPtr labels;
};

/// Sample a rectangular patch into an existing scene.
/// @param origin  corner of the patch
/// @param u,v     edge vectors spanning the patch (length = patch extent)
/// @param normal  outward-facing (into the room) unit normal
/// @param spacing sample spacing in meters
/// @param sigma   Gaussian position noise along the normal, in meters
/// @param label   ground-truth surface label (> 0)
inline void add_patch(SyntheticScene &scene, const Eigen::Vector3f &origin,
                      const Eigen::Vector3f &u, const Eigen::Vector3f &v,
                      const Eigen::Vector3f &normal, float spacing, float sigma,
                      uint32_t label, std::mt19937 &gen) {
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

      LabelT l;
      l.label = label;
      scene.labels->push_back(l);
    }
  }
}

/// A single rectangular room (axis-aligned box interior): floor, ceiling and
/// four walls, sampled at @p spacing with @p sigma position noise.
/// Default dimensions 4m x 3m x 2.5m. Normals face into the room.
/// Ground-truth labels: 1=floor, 2=ceiling, 3..6=walls.
inline SyntheticScene make_room(float sx = 4.0F, float sy = 3.0F,
                                float sz = 2.5F, float spacing = 0.05F,
                                float sigma = 0.005F, unsigned seed = 42) {
  std::mt19937 gen(seed);
  SyntheticScene scene{std::make_shared<Cloud>(), std::make_shared<CloudN>(),
                       std::make_shared<CloudL>()};

  const Eigen::Vector3f ex(sx, 0, 0), ey(0, sy, 0), ez(0, 0, sz);
  const Eigen::Vector3f o(0, 0, 0);

  // floor (z=0, normal up) / ceiling (z=sz, normal down)
  add_patch(scene, o, ex, ey, {0, 0, 1}, spacing, sigma, 1, gen);
  add_patch(scene, o + ez, ex, ey, {0, 0, -1}, spacing, sigma, 2, gen);
  // walls x=0 / x=sx
  add_patch(scene, o, ey, ez, {1, 0, 0}, spacing, sigma, 3, gen);
  add_patch(scene, o + ex, ey, ez, {-1, 0, 0}, spacing, sigma, 4, gen);
  // walls y=0 / y=sy
  add_patch(scene, o, ex, ez, {0, 1, 0}, spacing, sigma, 5, gen);
  add_patch(scene, o + ey, ex, ez, {0, -1, 0}, spacing, sigma, 6, gen);

  scene.cloud->width = static_cast<uint32_t>(scene.cloud->size());
  scene.cloud->height = 1;
  scene.normals->width = static_cast<uint32_t>(scene.normals->size());
  scene.normals->height = 1;
  scene.labels->width = static_cast<uint32_t>(scene.labels->size());
  scene.labels->height = 1;
  return scene;
}

/// Simulate SLAM pose error: apply an independent small rigid transform to
/// each contiguous chunk of @p chunk_size points (points are appended in
/// scan order, so a chunk approximates the points of one sensor frame).
///
/// Rotation is applied about the chunk centroid so the displacement stays
/// local (like a drifted camera pose), not amplified by lever arms to the
/// origin.
///
/// @param sigma_trans   per-axis Gaussian translation noise [m]
/// @param sigma_rot_rad Gaussian rotation angle noise about a random axis
inline SyntheticScene perturb_chunks(const SyntheticScene &in,
                                     std::size_t chunk_size, float sigma_trans,
                                     float sigma_rot_rad, unsigned seed = 42) {
  std::mt19937 gen(seed);
  std::normal_distribution<float> trans_noise(0.0F, sigma_trans);
  std::normal_distribution<float> rot_noise(0.0F, sigma_rot_rad);
  std::uniform_real_distribution<float> axis_dist(-1.0F, 1.0F);

  SyntheticScene out{std::make_shared<Cloud>(*in.cloud),
                     std::make_shared<CloudN>(*in.normals),
                     std::make_shared<CloudL>(*in.labels)};

  const std::size_t n = out.cloud->size();
  for (std::size_t start = 0; start < n; start += chunk_size) {
    const std::size_t end = std::min(start + chunk_size, n);

    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (std::size_t i = start; i < end; ++i)
      centroid += out.cloud->points[i].getVector3fMap();
    centroid /= static_cast<float>(end - start);

    Eigen::Vector3f axis(axis_dist(gen), axis_dist(gen), axis_dist(gen));
    if (axis.norm() < 1e-6F)
      axis = Eigen::Vector3f::UnitZ();
    axis.normalize();

    const Eigen::AngleAxisf rot(rot_noise(gen), axis);
    const Eigen::Vector3f t(trans_noise(gen), trans_noise(gen),
                            trans_noise(gen));

    for (std::size_t i = start; i < end; ++i) {
      auto p = out.cloud->points[i].getVector3fMap();
      p = rot * (p - centroid) + centroid + t;
      out.cloud->points[i].getVector3fMap() = p;

      auto nrm = out.normals->points[i].getNormalVector3fMap();
      nrm = rot * nrm;
      out.normals->points[i].getNormalVector3fMap() = nrm;
    }
  }
  return out;
}

} // namespace reusex::test_support
