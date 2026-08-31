// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/geometry/accuracy_metrics.hpp"
#include "reusex/core/logging.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace reusex::geometry {

namespace {

using LocCloud = pcl::PointCloud<pcl::PointXYZ>;

/// Median of a vector (destructive: reorders it). Empty -> 0.
double median(std::vector<float> &values) {
  if (values.empty())
    return 0.0;
  const std::size_t mid = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + mid, values.end());
  const double hi = values[mid];
  if (values.size() % 2 == 1)
    return hi;
  // Even count: average the two central elements.
  const double lo = *std::max_element(values.begin(), values.begin() + mid);
  return 0.5 * (lo + hi);
}

/// Copy the finite XYZ positions of an XYZRGB cloud into a plain XYZ cloud.
LocCloud::Ptr finite_positions(const Cloud &cloud) {
  auto out = std::make_shared<LocCloud>();
  out->reserve(cloud.size());
  for (const auto &p : cloud.points) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
      out->emplace_back(p.x, p.y, p.z);
  }
  out->width = static_cast<uint32_t>(out->size());
  out->height = 1;
  return out;
}

/// Keep only the finite points of an XYZ cloud.
LocCloud::Ptr finite_positions(const CloudLoc &cloud) {
  auto out = std::make_shared<LocCloud>();
  out->reserve(cloud.size());
  for (const auto &p : cloud.points) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
      out->emplace_back(p.x, p.y, p.z);
  }
  out->width = static_cast<uint32_t>(out->size());
  out->height = 1;
  return out;
}

/// For every point in @p query, the Euclidean distance to its nearest
/// neighbour in the tree built over @p reference.
std::vector<float> nearest_distances(const LocCloud &query,
                                     const LocCloud::Ptr &reference) {
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(reference);

  std::vector<float> dists;
  dists.reserve(query.size());
  std::vector<int> idx(1);
  std::vector<float> sq(1);
  for (const auto &p : query.points) {
    if (tree.nearestKSearch(p, 1, idx, sq) > 0)
      dists.push_back(std::sqrt(sq[0]));
  }
  return dists;
}

double mean(const std::vector<float> &values) {
  if (values.empty())
    return 0.0;
  double sum = 0.0;
  for (const float v : values)
    sum += v;
  return sum / static_cast<double>(values.size());
}

/// Fraction of distances strictly below @p threshold.
double inlier_fraction(const std::vector<float> &dists, float threshold) {
  if (dists.empty())
    return 0.0;
  std::size_t inliers = 0;
  for (const float d : dists)
    if (d < threshold)
      ++inliers;
  return static_cast<double>(inliers) / static_cast<double>(dists.size());
}

} // namespace

AccuracyReport compute_accuracy(const Cloud &cloud, const CloudLoc &gt,
                                const AccuracyMetricsOptions &opt) {
  if (cloud.empty())
    throw std::invalid_argument("compute_accuracy: reconstruction cloud is "
                                "empty");
  if (gt.empty())
    throw std::invalid_argument("compute_accuracy: ground-truth cloud is "
                                "empty");

  // Strip non-finite points up front so KdTree math and counts are exact.
  LocCloud::Ptr recon = finite_positions(cloud);
  LocCloud::Ptr gt_pts = finite_positions(gt);

  if (recon->empty())
    throw std::invalid_argument("compute_accuracy: reconstruction cloud has no "
                                "finite points");
  if (gt_pts->empty())
    throw std::invalid_argument("compute_accuracy: ground-truth cloud has no "
                                "finite points");

  // Optional GT downsample. Survey clouds are far denser than reconstructions;
  // downsampling keeps completeness fair and the KdTree affordable.
  if (opt.gt_voxel > 0.0f) {
    auto reduced = std::make_shared<LocCloud>();
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(gt_pts);
    vg.setLeafSize(opt.gt_voxel, opt.gt_voxel, opt.gt_voxel);
    vg.filter(*reduced);
    if (reduced->empty()) {
      reusex::warn("compute_accuracy: GT voxel downsample ({} m) produced an "
                   "empty cloud; using raw ground truth instead",
                   opt.gt_voxel);
    } else {
      gt_pts = reduced;
    }
  }

  AccuracyReport report;
  report.threshold = opt.threshold;
  report.cloud_points = recon->size();
  report.gt_points = gt_pts->size();

  // accuracy: reconstruction -> nearest ground truth.
  std::vector<float> acc = nearest_distances(*recon, gt_pts);
  // completeness: ground truth -> nearest reconstruction.
  std::vector<float> comp = nearest_distances(*gt_pts, recon);

  report.accuracy_mean = mean(acc);
  report.completeness_mean = mean(comp);
  report.chamfer = 0.5 * (report.accuracy_mean + report.completeness_mean);

  report.precision = inlier_fraction(acc, opt.threshold);
  report.recall = inlier_fraction(comp, opt.threshold);
  const double pr = report.precision + report.recall;
  report.fscore =
      pr > 0.0 ? (2.0 * report.precision * report.recall) / pr : 0.0;

  // Medians last: they reorder the distance vectors.
  report.accuracy_median = median(acc);
  report.completeness_median = median(comp);

  return report;
}

std::string AccuracyReport::to_json(int indent) const {
  nlohmann::json j;
  j["cloud_points"] = cloud_points;
  j["gt_points"] = gt_points;
  j["accuracy_mean"] = accuracy_mean;
  j["accuracy_median"] = accuracy_median;
  j["completeness_mean"] = completeness_mean;
  j["completeness_median"] = completeness_median;
  j["chamfer"] = chamfer;
  j["precision"] = precision;
  j["recall"] = recall;
  j["fscore"] = fscore;
  j["threshold"] = threshold;
  return j.dump(indent);
}

} // namespace reusex::geometry
