// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "reusex/types.hpp"
#include <atomic>
#include <map>
#include <set>

namespace reusex::geometry {

/**
 * @brief Request structure for instance segmentation via Euclidean clustering
 *
 * Applies Euclidean clustering within each semantic class to generate
 * instance-level labels. For example, multiple walls with the same semantic
 * label will be separated into wall_1, wall_2, etc.
 */
struct SegmentInstancesRequest {
  /// Input point cloud (spatial positions)
  CloudConstPtr cloud;

  /// Semantic class labels (same size as cloud)
  CloudLConstPtr semantic_labels;

  /// Euclidean distance threshold for clustering (meters)
  float cluster_tolerance = 0.5F;

  /// Minimum points per instance cluster
  int min_cluster_size = 50;

  /// Maximum points per instance cluster
  int max_cluster_size = 1000000;

  /**
   * @brief Optional filter for specific semantic labels
   *
   * If empty, processes all labels > 0 (label 0 = unlabeled/background).
   * If specified, only these semantic labels will be clustered.
   */
  std::set<uint32_t> labels_to_process;

  /// Optional cancellation token for long-running operations
  const std::atomic_bool *cancel_token = nullptr;
};

/**
 * @brief Result structure for instance segmentation
 */
struct SegmentInstancesResult {
  /// Instance-labeled cloud (same size as input cloud)
  /// Labels are sequential: 1, 2, 3, ... (0 = unlabeled)
  CloudLPtr instance_labels;

  /// Mapping from instance ID to semantic class ID
  std::map<uint32_t, uint32_t> instance_to_semantic;

  /// Mapping from instance ID to point count
  std::map<uint32_t, size_t> instance_sizes;
};

/**
 * @brief Segment point cloud into instances using Euclidean clustering
 *
 * Applies PCL's Euclidean clustering algorithm within each semantic class
 * to separate spatially-disconnected regions into distinct instances.
 *
 * Algorithm:
 * 1. Extract unique semantic labels (skip label 0)
 * 2. For each semantic class (parallelized with OpenMP):
 *    - Build KdTree with points from that class
 *    - Apply pcl::extractEuclideanClusters
 *    - Assign sequential instance IDs
 * 3. Store metadata (instance → semantic mapping, sizes)
 *
 * @param request Configuration and input data
 * @return Result containing instance labels and metadata
 * @throws std::invalid_argument if inputs are invalid
 * @throws std::runtime_error if cancellation requested
 */
auto segment_instances(const SegmentInstancesRequest &request)
    -> SegmentInstancesResult;

/**
 * @brief Implementation function for segment_instances
 *
 * Separated for future enhancements (e.g., different clustering algorithms)
 */
auto segment_instances_impl(const SegmentInstancesRequest &request)
    -> SegmentInstancesResult;

} // namespace reusex::geometry
