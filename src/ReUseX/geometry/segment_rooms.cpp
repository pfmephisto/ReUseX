// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ReUseX/geometry/segment_rooms.hpp"

namespace ReUseX::geometry {
auto segment_rooms_impl(CloudConstPtr cloud, CloudNPtr normals,
                        const float grid_size, const float inflation,
                        const float expansion, const float pruning_threshold,
                        const float convergence_threshold, const int max_iter,
                        const bool visualize) -> CloudLPtr {

  std::vector<pcl::ModelCoefficients> model_coefficients;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>
      centroids;
  std::vector<IndicesPtr> inlier_indices;

  assert(model_coefficients.size() == inlier_indices.size());
  assert(model_coefficients.size() == centroids.size());

  IndicesPtr indices(new Indices);
  pcl::UniformSampling<PointT> us;
  us.setInputCloud(cloud);
  us.setRadiusSearch(grid_size);
  for (const auto &idx : inlier_indices) {
    us.setIndices(idx);
    IndicesPtr local_indices(new Indices);
    us.filter(*local_indices);
    indices->insert(indices->end(), local_indices->begin(),
                    local_indices->end());
  }

  // Set the normals according to the plane coefficients
  for (size_t i = 0; i < model_coefficients.size(); ++i) {
    const auto &coeff = model_coefficients[i];
    for (const auto &index : *inlier_indices[i]) {
      normals->points[index].normal_x = coeff.values[0];
      normals->points[index].normal_y = coeff.values[1];
      normals->points[index].normal_z = coeff.values[2];
    }
  }

  spdlog::trace("Initialize the Markov Clustering algorithm");
  pcl::MarkovClustering<PointT, NormalT, LabelT> mcl;
  mcl.setInflationFactor(inflation);
  mcl.setExpansionFactor(expansion);
  mcl.setPruningThreshold(pruning_threshold);
  mcl.setConvergenceThreshold(convergence_threshold);
  mcl.setMaxIterations(max_iter);

  mcl.setGridSize(grid_size);
  mcl.setInputCloud(cloud);
  mcl.setInputNormals(normals);
  mcl.setIndices(indices);

  // Set up PCL Visualizer
  pcl::visualization::PCLVisualizer::Ptr viewer;
  if (visualize) {
    spdlog::warn("Visualization is an experimental feature.");
    viewer = pcl::visualization::PCLVisualizer::Ptr(
        new pcl::visualization::PCLVisualizer("MCL Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    mcl.registerVisualizationCallback(
        [&viewer](typename pcl::PointCloud<pcl::PointXYZ>::Ptr points,
                  std::shared_ptr<std::vector<pcl::Vertices>> vertices,
                  pcl::CorrespondencesPtr correspondences) {
          spdlog::trace("Updating visualization context with {} points and "
                        "{} polygons",
                        points->size(), vertices->size());

          constexpr std::string_view polygon_id = "disk_polygon";
          viewer->addPolygonMesh<pcl::PointXYZ>(points, *vertices,
                                                polygon_id.data());
          viewer->setPointCloudRenderingProperties(
              pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0,
              polygon_id.data());
          viewer->setPointCloudRenderingProperties(
              pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5,
              polygon_id.data());

          constexpr std::string_view corr_id = "correspondences";

          viewer->addCorrespondences<pcl::PointXYZ>(
              points, points, *correspondences, corr_id.data());

          viewer->setShapeRenderingProperties(
              pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0.5,
              corr_id.data());
          viewer->setShapeRenderingProperties(
              pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, corr_id.data());
          viewer->setShapeRenderingProperties(
              pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, corr_id.data());
        });

    viewer->addPointCloud<PointT>(cloud, "cloud");
  }

  spdlog::trace("Initialize labels and copy xyzrgb data to labels");
  CloudLPtr labels(new CloudL);
  pcl::copyPointCloud(*cloud, *labels);
  for (size_t i = 0; i < labels->points.size(); ++i)
    labels->points[i].label = -1;

  mcl.cluster(*labels);
  spdlog::trace("Done clustering");

  if (viewer)
    while (!viewer->wasStopped()) {
      viewer->spinOnce(100);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

  // Assign the label to all points
  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(cloud, indices);
  IndicesPtr missing_indices(new Indices);
  spdlog::trace("Resizing missing indices to size {}, cloud size: {}, "
                "indices size: {}",
                cloud->points.size() - indices->size(), cloud->points.size(),
                indices->size());
  missing_indices->reserve(cloud->points.size() - indices->size());

  std::sort(indices->begin(), indices->end());

  int j = 0;
  for (int i = 0; i < static_cast<int>(cloud->size()); ++i) {
    if (j < static_cast<int>(indices->size()) && indices->at(j) == i) {
      ++j; // skip
    } else {
      missing_indices->push_back(i);
    }
  }

  for (size_t i = 0; i < missing_indices->size(); ++i) {
    const size_t idx = missing_indices->at(i);
    std::vector<int> nn_indices(1);
    std::vector<float> nn_sqr_dists(1);
    if (kdtree.nearestKSearch(cloud->points[idx], 1, nn_indices, nn_sqr_dists) >
        0) {
      labels->points[idx].label = labels->points[nn_indices[0]].label;
    }
  }

  spdlog::info("Number of clusters found: {}", mcl.getNumClusters());

  return labels;
}
} // namespace ReUseX::geometry
