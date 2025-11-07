// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ReUseX/geometry/segment_planes.hpp"

namespace ReUseX::geometry {

auto segment_planes_impl(CloudConstPtr cloud, CloudNConstPtr normals,
                         const float angle_threshold,
                         const float plane_dist_threshold,
                         const int min_inliers, const float radius,
                         const float interval_0, const float interval_factor,
                         const bool visualize) -> CloudLPtr {

  spdlog::trace("Initialize the segmentation algorithm");
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
    spdlog::warn("Visualization is an experimental feature.");
    viewer = pcl::visualization::PCLVisualizer::Ptr(
        new pcl::visualization::PCLVisualizer("MCL Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    seg.registerVisualizationCallback(
        [&viewer](const pcl::ModelCoefficients &plane,
                  const Eigen::Vector4f &origin) {
          spdlog::trace("Visualization callback called");
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

  spdlog::trace("Initialize labels and copy xyzrgb data to labels");
  CloudLPtr labels(new CloudL);
  pcl::copyPointCloud(*cloud, *labels);

  spdlog::trace("Call the segmentation algorithm");
  seg.segment(labels);

  spdlog::info("Found {} clusters", seg.getCentroids().size());
  if (viewer)
    while (!viewer->wasStopped()) {
      viewer->spinOnce(100);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

  std::vector<pcl::ModelCoefficients> model_coefficients =
      seg.getModelCoefficients();
  auto centroids = seg.getCentroids();
  auto inlier_indices = seg.getInlierIndices();

  return labels;
}
} // namespace ReUseX::geometry
