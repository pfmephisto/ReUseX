// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "geometry/segment_planes.hpp"
#include "core/logging.hpp"

#include <stdexcept>

namespace reusex::geometry {

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
    reusex::warn(
        "segment_planes: cancellation requested before execution.");
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
  return segment_planes_impl(cloud, normals, options);
}
} // namespace reusex::geometry
