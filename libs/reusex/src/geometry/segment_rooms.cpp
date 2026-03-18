// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "geometry/segment_rooms.hpp"
#include "core/logging.hpp"

#include <stdexcept>

namespace ReUseX::geometry {
/**
 * @brief Implementation of room segmentation using Markov clustering.
 *
 * Segments a point cloud into rooms using Markov clustering based on
 * visual relations between points. Uses uniform sampling and MCL algorithm.
 *
 * @param cloud Input point cloud.
 * @param normals Point cloud normals.
 * @param planes Labeled point cloud with plane IDs.
 * @param grid_size Uniform sampling grid size.
 * @param inflation MCL inflation factor (higher = more clusters).
 * @param expansion MCL expansion factor.
 * @param pruning_threshold MCL pruning threshold.
 * @param convergence_threshold MCL convergence threshold.
 * @param max_iter Maximum MCL iterations.
 * @return Labeled point cloud with room assignments.
 */
auto segment_rooms_impl(const SegmentRoomsRequest &request) -> CloudLPtr {

  if (request.cancel_token != nullptr && request.cancel_token->load()) {
    ReUseX::core::warn(
        "segment_rooms: cancellation requested before execution.");
    throw std::runtime_error("Room segmentation cancelled.");
  }

  std::unordered_map<int, IndicesPtr> plane_inlier_map;
  for (size_t i = 0; i < request.planes->points.size(); ++i) {
    const int label = request.planes->points[i].label;

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
  us.setInputCloud(request.cloud);
  us.setRadiusSearch(request.grid_size);
  for (const auto &[key, idx] : plane_inlier_map) {
    us.setIndices(idx);
    IndicesPtr local_indices(new Indices);
    us.filter(*local_indices);
    indices->insert(indices->end(), local_indices->begin(),
                    local_indices->end());
  }

  pcl::MarkovClustering<PointT, NormalT, LabelT> mcl;
  mcl.setInflationFactor(request.inflation);
  mcl.setExpansionFactor(request.expansion);
  mcl.setPruningThreshold(request.pruning_threshold);
  mcl.setConvergenceThreshold(request.convergence_threshold);
  mcl.setMaxIterations(request.max_iter);

  mcl.setGridSize(request.grid_size);
  mcl.setInputCloud(request.cloud);
  mcl.setInputNormals(request.normals);
  mcl.setIndices(indices);

  ReUseX::core::trace("Initialize labels and copy xyzrgb data to labels");
  CloudLPtr labels(new CloudL);
  pcl::copyPointCloud(*request.cloud, *labels);
  for (size_t i = 0; i < labels->points.size(); ++i)
    labels->points[i].label = -1;

  mcl.cluster(*labels);
  ReUseX::core::trace("Done clustering");

  // Assign the label to all points
  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(request.cloud, indices);
  IndicesPtr missing_indices(new Indices);
  ReUseX::core::trace("Resizing missing indices to size {}, cloud size: {}, "
                      "indices size: {}",
                      request.cloud->points.size() - indices->size(),
                      request.cloud->points.size(), indices->size());
  missing_indices->reserve(request.cloud->points.size() - indices->size());

  std::sort(indices->begin(), indices->end());

  int j = 0;
  for (int i = 0; i < static_cast<int>(request.cloud->size()); ++i) {
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
    if (kdtree.nearestKSearch(request.cloud->points[idx], 1, nn_indices,
                              nn_sqr_dists) > 0) {
      labels->points[idx].label = labels->points[nn_indices[0]].label;
    }
  }

  ReUseX::core::info("Number of clusters found: {}", mcl.getNumClusters());

  return labels;
}

auto segment_rooms(const SegmentRoomsRequest &request) -> CloudLPtr {
  return segment_rooms_impl(request);
}
} // namespace ReUseX::geometry
