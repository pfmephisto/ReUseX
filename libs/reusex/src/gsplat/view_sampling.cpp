// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "view_sampling.hpp"

#include <reusex/core/logging.hpp>

#include <Eigen/Core>

#include <algorithm>

namespace reusex::gsplat::detail {

ViewSplit split_views(std::size_t n_views, int holdout_every) {
  ViewSplit s;
  for (std::size_t i = 0; i < n_views; ++i) {
    const bool held =
        holdout_every > 0 && i % static_cast<std::size_t>(holdout_every) == 0;
    (held ? s.holdout : s.train).push_back(i);
  }

  // A split that leaves nothing to train on is worse than no split. This only
  // triggers for tiny view sets (holdout_every == 1, or a single view).
  if (s.train.empty()) {
    core::warn("gsplat: a held-out split of every {}th view would leave {} "
               "training views out of {} — training on all views instead, and "
               "reporting no held-out metric",
               holdout_every, s.train.size(), n_views);
    s.train.clear();
    s.holdout.clear();
    for (std::size_t i = 0; i < n_views; ++i)
      s.train.push_back(i);
  }
  return s;
}

std::vector<std::size_t> stride_sample(const std::vector<std::size_t> &src,
                                       int max_n) {
  if (max_n <= 0 || src.size() <= static_cast<std::size_t>(max_n))
    return src;
  const std::size_t step = src.size() / static_cast<std::size_t>(max_n);
  std::vector<std::size_t> out;
  out.reserve(static_cast<std::size_t>(max_n));
  for (std::size_t i = 0; out.size() < static_cast<std::size_t>(max_n);
       i += step)
    out.push_back(src[i]);
  return out;
}

double scene_extent(const std::vector<TrainingView> &views) {
  if (views.empty())
    return 1e-3;

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> centers;
  centers.reserve(views.size());
  for (const auto &v : views) {
    // camera centre in world = -R^T t  for T_cw = [R|t]
    const Eigen::Matrix3d R = v.T_cw.block<3, 3>(0, 0);
    const Eigen::Vector3d t = v.T_cw.block<3, 1>(0, 3);
    centers.push_back(-R.transpose() * t);
    centroid += centers.back();
  }
  centroid /= static_cast<double>(centers.size());
  double radius = 0.0;
  for (const auto &c : centers)
    radius = std::max(radius, (c - centroid).norm());
  // A single-viewpoint capture would otherwise scale the LR to zero.
  return std::max(radius, 1e-3);
}

} // namespace reusex::gsplat::detail
