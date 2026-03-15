// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ReUseX/core/logging.hpp>
#include <ReUseX/geometry/segment_planes.hpp>

namespace ReUseX::geometry {

/**
 * @brief Implementation of plane segmentation using multi-scale region growing.
 * 
 * Performs planar region growing segmentation on a point cloud using adaptive
 * interval scaling and returns labeled planes with their centroids and normals.
 * 
 * @param cloud Input point cloud.
 * @param normals Point cloud normals.
 * @param angle_threshold Angular threshold for plane fitting (degrees).
 * @param plane_dist_threshold Distance threshold for plane fitting.
 * @param min_inliers Minimum number of inliers for a valid plane.
 * @param radius Search radius for region growing.
 * @param interval_0 Initial interval for multi-scale processing.
 * @param interval_factor Factor for interval scaling between iterations.
 * @param visualize Enable visualization of segmentation process.
 * @return Tuple of (labeled point cloud, plane centroids, plane normals).
 */
auto segment_planes_impl(CloudConstPtr cloud, CloudNConstPtr normals,
                         const float angle_threshold,
                         const float plane_dist_threshold,
                         const int min_inliers, const float radius,
                         const float interval_0, const float interval_factor,
                         const bool visualize)
    -> std::tuple<CloudLPtr, CloudLocPtr, CloudNPtr> {

  ReUseX::core::trace("Initialize the segmentation algorithm");
  pcl::PlanarRegionGrowing<PointT, NormalT, LabelT> seg;
  seg.setInputCloud(cloud);
  seg.setInputNormals(normals);

  seg.setAngularThreshold(angle_threshold);
  seg.setDistanceThreshold(plane_dist_threshold);
  seg.setMinInliers(min_inliers);

  seg.setRadiusSearch(radius);

  seg.setInitialInterval(interval_0);
  seg.setIntervalFactor(interval_factor);

  pcl::visualization::PCLVisualizer::Ptr viewer;
  if (visualize) {
    static int plane_id = 0;
    ReUseX::core::warn("Visualization is an experimental feature.");
    viewer = pcl::visualization::PCLVisualizer::Ptr(
        new pcl::visualization::PCLVisualizer("MCL Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    seg.registerVisualizationCallback(
        [&viewer](const pcl::ModelCoefficients &plane,
                  const Eigen::Vector4f &origin) {
          ReUseX::core::trace("Visualization callback called");
          const auto name = "plane" + std::to_string(plane_id);
          const auto color = pcl::GlasbeyLUT::at(plane_id);
          viewer->addPlane(plane, origin.x(), origin.y(), origin.z(), name);
          viewer->setShapeRenderingProperties(
              pcl::visualization::PCL_VISUALIZER_COLOR, color.r / 254,
              color.g / 254, color.b / 254, name);
          viewer->setShapeRenderingProperties(
              pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, name);
          plane_id++;
        });

    viewer->addPointCloud<PointT>(cloud, "cloud");
  }

  ReUseX::core::trace("Initialize labels and copy xyzrgb data to labels");
  CloudLPtr labels(new CloudL);
  pcl::copyPointCloud(*cloud, *labels);

  ReUseX::core::trace("Call the segmentation algorithm");
  seg.segment(labels);

  ReUseX::core::info("Found {} clusters", seg.getCentroids().size());
  if (viewer)
    while (!viewer->wasStopped()) {
      viewer->spinOnce(100);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

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
} // namespace ReUseX::geometry
