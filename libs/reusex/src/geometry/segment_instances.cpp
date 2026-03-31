// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/geometry/segment_instances.hpp"
#include "reusex/core/processing_observer.hpp"
#include "reusex/core/stages.hpp"

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace ReUseX::geometry {

auto segment_instances(const SegmentInstancesRequest &request)
    -> SegmentInstancesResult {
  return segment_instances_impl(request);
}

auto segment_instances_impl(const SegmentInstancesRequest &request)
    -> SegmentInstancesResult {
  // Validate inputs
  if (!request.cloud) {
    throw std::invalid_argument("Input cloud is null");
  }
  if (!request.semantic_labels) {
    throw std::invalid_argument("Semantic labels cloud is null");
  }
  if (request.cloud->empty()) {
    throw std::invalid_argument("Input cloud is empty");
  }
  if (request.cloud->size() != request.semantic_labels->size()) {
    throw std::invalid_argument(
        fmt::format("Size mismatch: cloud has {} points, semantic_labels has "
                    "{} points",
                    request.cloud->size(), request.semantic_labels->size()));
  }
  if (request.cluster_tolerance <= 0.0F) {
    throw std::invalid_argument(
        fmt::format("cluster_tolerance must be positive, got {}",
                    request.cluster_tolerance));
  }
  if (request.min_cluster_size <= 0) {
    throw std::invalid_argument(
        fmt::format("min_cluster_size must be positive, got {}",
                    request.min_cluster_size));
  }
  if (request.max_cluster_size < request.min_cluster_size) {
    throw std::invalid_argument(fmt::format(
        "max_cluster_size ({}) must be >= min_cluster_size ({})",
        request.max_cluster_size, request.min_cluster_size));
  }

  spdlog::info("Starting instance segmentation on {} points",
               request.cloud->size());
  spdlog::debug("Parameters: tolerance={:.3f}m, min_size={}, max_size={}",
                request.cluster_tolerance, request.min_cluster_size,
                request.max_cluster_size);

  // Initialize result
  SegmentInstancesResult result;
  result.instance_labels = CloudLPtr(new CloudL);
  result.instance_labels->resize(request.cloud->size());
  std::fill(result.instance_labels->begin(), result.instance_labels->end(),
            pcl::Label());

  // Extract unique semantic labels (skip label 0 = unlabeled/background)
  std::set<uint32_t> unique_labels;
  for (const auto &label : *request.semantic_labels) {
    if (label.label > 0) {
      unique_labels.insert(label.label);
    }
  }

  // Apply optional label filtering
  if (!request.labels_to_process.empty()) {
    std::set<uint32_t> filtered_labels;
    std::set_intersection(unique_labels.begin(), unique_labels.end(),
                          request.labels_to_process.begin(),
                          request.labels_to_process.end(),
                          std::inserter(filtered_labels, filtered_labels.end()));
    unique_labels = filtered_labels;
    spdlog::debug("Filtered to {} semantic labels", unique_labels.size());
  }

  if (unique_labels.empty()) {
    spdlog::warn("No semantic labels found (all points are label 0)");
    return result;
  }

  spdlog::info("Processing {} semantic classes", unique_labels.size());

  // Convert set to vector for indexed access in parallel loop
  std::vector<uint32_t> labels_vec(unique_labels.begin(), unique_labels.end());

  // Build index maps: semantic_label -> point indices
  std::unordered_map<uint32_t, pcl::Indices> label_to_indices;
  for (size_t i = 0; i < request.semantic_labels->size(); ++i) {
    uint32_t label = (*request.semantic_labels)[i].label;
    if (label > 0 && unique_labels.count(label) > 0) {
      label_to_indices[label].push_back(static_cast<int>(i));
    }
  }

  // Progress tracking
  auto observer = ReUseX::core::ProgressObserver(
      ReUseX::core::Stage::instance_clustering, labels_vec.size());

  // Shared state for instance ID assignment (protected by critical section)
  uint32_t next_instance_id = 1;
  bool cancelled = false;

// Parallel processing of each semantic class
#pragma omp parallel for schedule(dynamic)
  for (size_t class_idx = 0; class_idx < labels_vec.size(); ++class_idx) {
    // Check cancellation
    if (request.cancel_token && request.cancel_token->load()) {
#pragma omp critical
      {
        if (!cancelled) {
          spdlog::warn("Instance segmentation cancelled by user");
          cancelled = true;
        }
      }
      continue; // Skip this iteration
    }

    uint32_t semantic_label = labels_vec[class_idx];
    const auto &indices = label_to_indices[semantic_label];

    spdlog::debug("Processing semantic label {} with {} points", semantic_label,
                  indices.size());

    // Build KdTree for this semantic class
    auto tree = pcl::search::KdTree<PointT>::Ptr(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(request.cloud, pcl::IndicesPtr(new pcl::Indices(indices)));

    // Extract Euclidean clusters
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::extractEuclideanClusters<PointT>(
        *request.cloud, indices, tree, request.cluster_tolerance,
        cluster_indices, request.min_cluster_size, request.max_cluster_size);

    spdlog::debug("Found {} clusters for semantic label {}", cluster_indices.size(),
                  semantic_label);

    // Assign instance IDs to clusters
#pragma omp critical
    {
      for (const auto &cluster : cluster_indices) {
        uint32_t instance_id = next_instance_id++;

        // Assign instance label to all points in cluster
        for (int point_idx : cluster.indices) {
          (*result.instance_labels)[point_idx].label = instance_id;
        }

        // Store metadata
        result.instance_to_semantic[instance_id] = semantic_label;
        result.instance_sizes[instance_id] = cluster.indices.size();
      }

      // Update progress
      ++observer;
    }
  }

  // Log final statistics
  size_t labeled_points = 0;
  for (const auto &label : *result.instance_labels) {
    if (label.label > 0) {
      ++labeled_points;
    }
  }

  // Check if operation was cancelled
  if (cancelled) {
    throw std::runtime_error("Instance segmentation cancelled by user");
  }

  spdlog::info("Instance segmentation complete: {} instances, {}/{} points "
               "labeled ({:.1f}%)",
               result.instance_to_semantic.size(), labeled_points,
               request.cloud->size(),
               100.0 * labeled_points / request.cloud->size());

  return result;
}

} // namespace ReUseX::geometry
