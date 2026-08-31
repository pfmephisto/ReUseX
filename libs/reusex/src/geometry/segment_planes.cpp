// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "geometry/segment_planes.hpp"
#include "core/logging.hpp"
#include "geometry/noise_estimate.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace reusex::geometry {

namespace {

/// Clamp helper (std::clamp requires equal types; keep it explicit and typed).
template <typename T> T clamp(T v, T lo, T hi) {
  return std::max(lo, std::min(v, hi));
}

/// Resolve the effective distance/inlier thresholds from measured cloud noise.
///
/// Adaptive rules (issue #214):
///   plane_dist_threshold = clamp(dist_sigma_factor * sigma, dist_min,
///   dist_max) min_inliers          = clamp(min_surface_area / spacing^2,
///                                 min_inliers_floor, min_inliers_ceiling)
/// where `spacing` is the median nearest-neighbour distance (density proxy).
///
/// Explicit overrides always win over adaptivity for their own parameter.
/// When `adaptive` is false, the caller's fixed values are returned unchanged.
SegmentPlanesOptions resolve_options(CloudConstPtr cloud,
                                     CloudNConstPtr normals,
                                     const SegmentPlanesOptions &in) {
  SegmentPlanesOptions out = in;

  if (!in.adaptive) {
    // Honour overrides even in fixed mode for API symmetry.
    if (in.plane_dist_threshold_override)
      out.plane_dist_threshold = *in.plane_dist_threshold_override;
    if (in.min_inliers_override)
      out.min_inliers = *in.min_inliers_override;
    reusex::info("segment_planes: adaptive thresholds disabled; using "
                 "plane_dist_threshold={:.4f} m, min_inliers={}",
                 out.plane_dist_threshold, out.min_inliers);
    return out;
  }

  NoiseEstimateOptions nopt;
  nopt.seed = in.noise_seed;
  const NoiseEstimate est = estimate_cloud_noise(cloud, normals, nopt);

  reusex::info("segment_planes: estimated sigma={:.4f} m, density(spacing)="
               "{:.4f} m from {} patches",
               est.sigma, est.density, est.samples);

  // Distance threshold: ~3 sigma, clamped. Fall back to the incoming value if
  // the estimate is degenerate (sigma == 0, e.g. tiny/failed cloud).
  if (in.plane_dist_threshold_override) {
    out.plane_dist_threshold = *in.plane_dist_threshold_override;
    reusex::info("segment_planes: plane_dist_threshold pinned to {:.4f} m "
                 "(explicit override; adaptivity bypassed)",
                 out.plane_dist_threshold);
  } else if (est.sigma > 0.0F) {
    out.plane_dist_threshold =
        clamp(in.dist_sigma_factor * est.sigma, in.dist_min, in.dist_max);
    reusex::info("segment_planes: adaptive plane_dist_threshold={:.4f} m "
                 "({:.1f} * sigma, clamped [{:.3f}, {:.3f}])",
                 out.plane_dist_threshold, in.dist_sigma_factor, in.dist_min,
                 in.dist_max);
  } else {
    reusex::warn("segment_planes: sigma estimate is 0; keeping default "
                 "plane_dist_threshold={:.4f} m",
                 out.plane_dist_threshold);
  }

  // Min inliers: scale by density so ~min_surface_area m^2 of surface is the
  // acceptance floor regardless of point spacing.
  if (in.min_inliers_override) {
    out.min_inliers = *in.min_inliers_override;
    reusex::info("segment_planes: min_inliers pinned to {} "
                 "(explicit override; adaptivity bypassed)",
                 out.min_inliers);
  } else if (est.density > 0.0F) {
    const double pts =
        static_cast<double>(in.min_surface_area) /
        (static_cast<double>(est.density) * static_cast<double>(est.density));
    out.min_inliers = clamp(static_cast<int>(std::lround(pts)),
                            in.min_inliers_floor, in.min_inliers_ceiling);
    reusex::info("segment_planes: adaptive min_inliers={} "
                 "(~{:.2f} m^2 at spacing {:.4f} m, clamped [{}, {}])",
                 out.min_inliers, in.min_surface_area, est.density,
                 in.min_inliers_floor, in.min_inliers_ceiling);
  } else {
    reusex::warn("segment_planes: density estimate is 0; keeping default "
                 "min_inliers={}",
                 out.min_inliers);
  }

  return out;
}

} // namespace

/**
 * @brief Implementation of plane segmentation using multi-scale region growing.
 *
 * Performs planar region growing segmentation on a point cloud using adaptive
 * interval scaling and returns labeled planes with their centroids and normals.
 *
 * @param cloud Input point cloud.
 * @param normals Point cloud normals.
 * @param options Segmentation options (thresholds, filter, etc.).
 * @return Tuple of (labeled point cloud, plane centroids, plane normals).
 */
auto segment_planes_impl(CloudConstPtr cloud, CloudNConstPtr normals,
                         const SegmentPlanesOptions &options)
    -> std::tuple<CloudLPtr, CloudLocPtr, CloudNPtr> {

  if (options.cancel_token != nullptr && options.cancel_token->load()) {
    reusex::warn("segment_planes: cancellation requested before execution.");
    throw std::runtime_error("Plane segmentation cancelled.");
  }

  reusex::trace("Initialize the segmentation algorithm");
  pcl::PlanarRegionGrowing<PointT, NormalT, LabelT> seg;
  seg.setInputCloud(cloud);
  seg.setInputNormals(normals);

  // Apply filter if provided
  if (options.filter) {
    seg.setIndices(options.filter);
    reusex::debug("Plane segmentation using {} filtered points",
                  options.filter->size());
  }

  seg.setAngularThreshold(options.angle_threshold);
  seg.setDistanceThreshold(options.plane_dist_threshold);
  seg.setMinInliers(options.min_inliers);

  seg.setRadiusSearch(options.radius);

  seg.setInitialInterval(options.interval_0);
  seg.setIntervalFactor(options.interval_factor);

  reusex::trace("Initialize labels and copy xyzrgb data to labels");
  CloudLPtr labels(new CloudL);
  pcl::copyPointCloud(*cloud, *labels);

  reusex::trace("Call the segmentation algorithm");
  seg.segment(labels);

  reusex::info("Found {} clusters", seg.getCentroids().size());

  std::vector<pcl::ModelCoefficients> model_coefficients =
      seg.getModelCoefficients();
  auto centroids = seg.getCentroids();
  auto inlier_indices = seg.getInlierIndices();

  CloudLocPtr centroids_cloud(new CloudLoc);
  centroids_cloud->points.resize(centroids.size());
  centroids_cloud->width = static_cast<uint32_t>(centroids.size());
  centroids_cloud->height = 1;
  for (size_t i = 0; i < centroids.size(); ++i) {
    centroids_cloud->points[i].x = centroids[i][0];
    centroids_cloud->points[i].y = centroids[i][1];
    centroids_cloud->points[i].z = centroids[i][2];
  }
  CloudNPtr plane_normals(new CloudN);
  plane_normals->points.resize(model_coefficients.size());
  plane_normals->height = 1;
  plane_normals->width = static_cast<uint32_t>(model_coefficients.size());
  for (size_t i = 0; i < model_coefficients.size(); ++i) {
    plane_normals->points[i].normal_x = model_coefficients[i].values[0];
    plane_normals->points[i].normal_y = model_coefficients[i].values[1];
    plane_normals->points[i].normal_z = model_coefficients[i].values[2];
  }

  return std::make_tuple(labels, centroids_cloud, plane_normals);
}

auto segment_planes(CloudConstPtr cloud, CloudNConstPtr normals,
                    const SegmentPlanesOptions &options)
    -> std::tuple<CloudLPtr, CloudLocPtr, CloudNPtr> {

  // Derive noise-adaptive thresholds (or honour explicit overrides) before
  // running the region-growing segmentation (issue #214).
  const SegmentPlanesOptions effective =
      resolve_options(cloud, normals, options);

  auto result = segment_planes_impl(cloud, normals, effective);
  const auto &centroids = std::get<1>(result);
  const auto &labels = std::get<0>(result);

  // --- Plane-count / coverage sanity warnings (issue #214) ------------------
  const std::size_t plane_count = centroids ? centroids->size() : 0;
  if (plane_count > static_cast<std::size_t>(effective.warn_plane_count))
    reusex::warn(
        "segment_planes: {} planes detected (> {}) — over-fragmented; noise or "
        "thresholds too tight (plane_dist_threshold={:.4f} m, min_inliers={}).",
        plane_count, effective.warn_plane_count, effective.plane_dist_threshold,
        effective.min_inliers);

  // Labeled fraction: points assigned to a plane (label > 0) over the
  // considered point count (the filtered subset if a filter was supplied,
  // otherwise the whole cloud).
  if (labels && !labels->empty()) {
    std::size_t labeled = 0;
    for (const auto &pt : labels->points)
      if (pt.label > 0)
        ++labeled;
    const std::size_t considered =
        effective.filter ? effective.filter->size() : labels->size();
    const float frac = considered > 0 ? static_cast<float>(labeled) /
                                            static_cast<float>(considered)
                                      : 0.0F;
    if (frac < effective.warn_labeled_fraction)
      reusex::warn(
          "segment_planes: only {:.1f}% of points labeled (< {:.0f}%) — many "
          "surfaces unassigned; thresholds may be too tight or the cloud too "
          "noisy ({} / {} points).",
          100.0F * frac, 100.0F * effective.warn_labeled_fraction, labeled,
          considered);
  }

  return result;
}
} // namespace reusex::geometry
