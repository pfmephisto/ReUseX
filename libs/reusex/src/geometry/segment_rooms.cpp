// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "geometry/segment_rooms.hpp"
#include "core/logging.hpp"

#include <stdexcept>
#include <unordered_set>

namespace ReUseX::geometry {
/**
 * @brief Implementation of room segmentation using community detection.
 *
 * Segments a point cloud into rooms using the Leiden community detection
 * algorithm based on visual relations between points. Uses uniform sampling
 * and Embree ray tracing to build a visibility graph.
 *
 * @param cloud Input point cloud.
 * @param normals Point cloud normals.
 * @param planes Labeled point cloud with plane IDs.
 * @param options Segmentation options (Leiden parameters, filter, etc.).
 * @return Labeled point cloud with room assignments.
 */
auto segment_rooms_impl(CloudConstPtr cloud, CloudNConstPtr normals,
                        CloudLConstPtr planes,
                        const SegmentRoomsOptions &options) -> CloudLPtr {

  if (options.cancel_token != nullptr && options.cancel_token->load()) {
    ReUseX::core::warn(
        "segment_rooms: cancellation requested before execution.");
    throw std::runtime_error("Room segmentation cancelled.");
  }

  // Build set of filtered indices for fast lookup if filter is provided
  std::unordered_set<int> filtered_indices_set;
  if (options.filter) {
    filtered_indices_set.insert(options.filter->begin(), options.filter->end());
    ReUseX::core::debug("Room segmentation using {} filtered points",
                        options.filter->size());
  }

  std::unordered_map<int, IndicesPtr> plane_inlier_map;
  for (size_t i = 0; i < planes->points.size(); ++i) {
    // Skip if not in filter (when filter is provided)
    if (options.filter && filtered_indices_set.find(static_cast<int>(i)) ==
                              filtered_indices_set.end()) {
      continue;
    }

    const int label = planes->points[i].label;

    // Skip unlabeled points
    if (label < 1)
      continue;

    if (plane_inlier_map.find(label) == plane_inlier_map.end()) {
      plane_inlier_map[label] = IndicesPtr(new Indices);
    }
    plane_inlier_map[label]->push_back(i);
  }

  IndicesPtr indices(new Indices);
  pcl::UniformSampling<PointT> us;
  us.setInputCloud(cloud);
  us.setRadiusSearch(options.grid_size);
  for (const auto &[key, idx] : plane_inlier_map) {
    us.setIndices(idx);
    IndicesPtr local_indices(new Indices);
    us.filter(*local_indices);
    indices->insert(indices->end(), local_indices->begin(),
                    local_indices->end());
  }

  pcl::CommunityClustering<PointT, NormalT, LabelT> cc;
  cc.setResolution(options.resolution);
  cc.setBeta(options.beta);
  cc.setMaxIterations(options.max_iter);

  cc.setGridSize(options.grid_size);
  cc.setInputCloud(cloud);
  cc.setInputNormals(normals);
  cc.setIndices(indices);

  ReUseX::core::trace("Initialize labels and copy xyzrgb data to labels");
  CloudLPtr labels(new CloudL);
  pcl::copyPointCloud(*cloud, *labels);
  for (size_t i = 0; i < labels->points.size(); ++i)
    labels->points[i].label = -1;

  cc.cluster(*labels);
  ReUseX::core::trace("Done clustering");

  // Assign the label to all points
  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(cloud, indices);
  IndicesPtr missing_indices(new Indices);
  ReUseX::core::trace("Resizing missing indices to size {}, cloud size: {}, "
                      "indices size: {}",
                      cloud->points.size() - indices->size(),
                      cloud->points.size(), indices->size());
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
    if (kdtree.nearestKSearch(cloud->points[idx], 1, nn_indices,
                              nn_sqr_dists) > 0) {
      labels->points[idx].label = labels->points[nn_indices[0]].label;
    }
  }

  ReUseX::core::info("Number of clusters found: {}", cc.getNumClusters());

  return labels;
}

auto segment_rooms(CloudConstPtr cloud, CloudNConstPtr normals,
                   CloudLConstPtr planes, const SegmentRoomsOptions &options)
    -> CloudLPtr {
  return segment_rooms_impl(cloud, normals, planes, options);
}
} // namespace ReUseX::geometry
