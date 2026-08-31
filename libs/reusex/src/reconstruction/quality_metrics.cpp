// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/reconstruction/quality_metrics.hpp"
#include "reusex/core/logging.hpp"

#include <Eigen/Eigenvalues>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>

namespace reusex::geometry {

namespace {

/// 90th percentile of |values| (destructive: reorders the vector).
double percentile90_abs(std::vector<double> &values) {
  if (values.empty())
    return 0.0;
  for (auto &v : values)
    v = std::abs(v);
  const auto idx =
      static_cast<std::ptrdiff_t>(static_cast<double>(values.size() - 1) * 0.9);
  std::nth_element(values.begin(), values.begin() + idx, values.end());
  return values[static_cast<std::size_t>(idx)];
}

} // namespace

QualityReport compute_plane_quality(const Cloud &cloud, const CloudL &labels,
                                    const QualityMetricsOptions &options) {
  if (cloud.empty())
    throw std::invalid_argument("compute_plane_quality: cloud is empty");
  if (cloud.size() != labels.size())
    throw std::invalid_argument(fmt::format(
        "compute_plane_quality: cloud size ({}) != labels size ({})",
        cloud.size(), labels.size()));

  QualityReport report;
  report.total_points = cloud.size();

  // Group point indices by plane label (label 0 = unlabeled, skipped).
  std::map<uint32_t, std::vector<std::size_t>> by_label;
  for (std::size_t i = 0; i < labels.size(); ++i) {
    const uint32_t label = labels.points[i].label;
    if (label > 0) {
      by_label[label].push_back(i);
      ++report.labeled_points;
    }
  }

  double rms_weighted_sum = 0.0;
  double p90_weighted_sum = 0.0;
  std::size_t weight_total = 0;

  for (const auto &[label, indices] : by_label) {
    if (indices.size() < options.min_points)
      continue;

    // Least-squares plane refit via PCA: centroid + covariance, the normal
    // is the eigenvector of the smallest eigenvalue.
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto idx : indices) {
      const auto &p = cloud.points[idx];
      centroid += Eigen::Vector3d(p.x, p.y, p.z);
    }
    centroid /= static_cast<double>(indices.size());

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto idx : indices) {
      const auto &p = cloud.points[idx];
      const Eigen::Vector3d d = Eigen::Vector3d(p.x, p.y, p.z) - centroid;
      covariance += d * d.transpose();
    }

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    const Eigen::Vector3d normal = solver.eigenvectors().col(0);

    std::vector<double> residuals;
    residuals.reserve(indices.size());
    double sq_sum = 0.0;
    for (const auto idx : indices) {
      const auto &p = cloud.points[idx];
      const double r = normal.dot(Eigen::Vector3d(p.x, p.y, p.z) - centroid);
      residuals.push_back(r);
      sq_sum += r * r;
    }

    PlaneQuality pq;
    pq.label = label;
    pq.point_count = indices.size();
    pq.rms = std::sqrt(sq_sum / static_cast<double>(indices.size()));
    pq.p90 = percentile90_abs(residuals);
    pq.normal = normal;
    pq.centroid = centroid;

    rms_weighted_sum += pq.rms * static_cast<double>(pq.point_count);
    p90_weighted_sum += pq.p90 * static_cast<double>(pq.point_count);
    weight_total += pq.point_count;
    report.flatness_rms_max = std::max(report.flatness_rms_max, pq.rms);

    report.planes.push_back(std::move(pq));
  }

  if (weight_total > 0) {
    report.flatness_rms = rms_weighted_sum / static_cast<double>(weight_total);
    report.thickness_p90 = p90_weighted_sum / static_cast<double>(weight_total);
  } else {
    reusex::warn("compute_plane_quality: no plane met min_points={}; "
                 "report contains no per-plane entries",
                 options.min_points);
  }

  return report;
}

std::string QualityReport::to_json(int indent) const {
  nlohmann::json j;
  j["total_points"] = total_points;
  j["labeled_points"] = labeled_points;
  j["plane_count"] = planes.size();
  j["flatness_rms"] = flatness_rms;
  j["flatness_rms_max"] = flatness_rms_max;
  j["thickness_p90"] = thickness_p90;

  auto &planes_json = j["planes"] = nlohmann::json::array();
  for (const auto &p : planes) {
    planes_json.push_back({
        {"label", p.label},
        {"point_count", p.point_count},
        {"rms", p.rms},
        {"p90", p.p90},
        {"normal", {p.normal.x(), p.normal.y(), p.normal.z()}},
        {"centroid", {p.centroid.x(), p.centroid.y(), p.centroid.z()}},
    });
  }
  return j.dump(indent);
}

} // namespace reusex::geometry
