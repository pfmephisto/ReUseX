// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/geometry/segment_instances.hpp>
#include <reusex/types.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <random>

using namespace ReUseX;
using namespace ReUseX::geometry;

// Helper: Create a cloud with points clustered around specified centers
static CloudPtr create_clustered_cloud(
    const std::vector<Eigen::Vector3f> &centers, int points_per_cluster,
    float spread = 0.1F) {
  CloudPtr cloud(new Cloud);
  std::mt19937 gen(42); // Fixed seed for reproducibility
  std::normal_distribution<float> dist(0.0F, spread);

  for (const auto &center : centers) {
    for (int i = 0; i < points_per_cluster; ++i) {
      PointT point;
      point.x = center.x() + dist(gen);
      point.y = center.y() + dist(gen);
      point.z = center.z() + dist(gen);
      point.r = 255;
      point.g = 255;
      point.b = 255;
      cloud->push_back(point);
    }
  }

  return cloud;
}

// Helper: Create semantic labels for clustered cloud
static CloudLPtr create_semantic_labels(
    const std::vector<uint32_t> &labels_per_cluster, int points_per_cluster) {
  CloudLPtr labels(new CloudL);

  for (uint32_t label : labels_per_cluster) {
    for (int i = 0; i < points_per_cluster; ++i) {
      pcl::Label l;
      l.label = label;
      labels->push_back(l);
    }
  }

  return labels;
}

TEST_CASE("segment_instances: Basic clustering",
          "[geometry][segment_instances]") {
  // Create 3 spatially-separated clusters with same semantic label
  std::vector<Eigen::Vector3f> centers = {
      {0.0F, 0.0F, 0.0F}, {5.0F, 0.0F, 0.0F}, {0.0F, 5.0F, 0.0F}};
  int points_per_cluster = 100;

  auto cloud = create_clustered_cloud(centers, points_per_cluster, 0.1F);
  auto semantic_labels =
      create_semantic_labels({1, 1, 1}, points_per_cluster); // All same class

  SegmentInstancesRequest request;
  request.cloud = cloud;
  request.semantic_labels = semantic_labels;
  request.cluster_tolerance = 0.5F; // Clusters are 5m apart, so this separates
  request.min_cluster_size = 50;

  auto result = segment_instances(request);

  SECTION("Creates correct number of instances") {
    REQUIRE(result.instance_to_semantic.size() == 3);
  }

  SECTION("All instances map to same semantic class") {
    for (const auto &[instance_id, semantic_class] :
         result.instance_to_semantic) {
      REQUIRE(semantic_class == 1);
    }
  }

  SECTION("Instance sizes are approximately equal") {
    for (const auto &[instance_id, size] : result.instance_sizes) {
      REQUIRE(size == static_cast<size_t>(points_per_cluster));
    }
  }

  SECTION("All points are labeled") {
    size_t labeled = 0;
    for (const auto &label : *result.instance_labels) {
      if (label.label > 0)
        ++labeled;
    }
    REQUIRE(labeled == cloud->size());
  }
}

TEST_CASE("segment_instances: Multi-class clustering",
          "[geometry][segment_instances]") {
  // Create 4 clusters: 2 of class 1, 2 of class 2
  std::vector<Eigen::Vector3f> centers = {{0.0F, 0.0F, 0.0F},
                                            {5.0F, 0.0F, 0.0F},
                                            {0.0F, 5.0F, 0.0F},
                                            {5.0F, 5.0F, 0.0F}};
  int points_per_cluster = 100;

  auto cloud = create_clustered_cloud(centers, points_per_cluster, 0.1F);
  auto semantic_labels = create_semantic_labels({1, 1, 2, 2}, points_per_cluster);

  SegmentInstancesRequest request;
  request.cloud = cloud;
  request.semantic_labels = semantic_labels;
  request.cluster_tolerance = 0.5F;
  request.min_cluster_size = 50;

  auto result = segment_instances(request);

  SECTION("Creates 4 instances total") {
    REQUIRE(result.instance_to_semantic.size() == 4);
  }

  SECTION("2 instances per semantic class") {
    size_t class1_count = 0;
    size_t class2_count = 0;
    for (const auto &[instance_id, semantic_class] :
         result.instance_to_semantic) {
      if (semantic_class == 1)
        ++class1_count;
      if (semantic_class == 2)
        ++class2_count;
    }
    REQUIRE(class1_count == 2);
    REQUIRE(class2_count == 2);
  }
}

TEST_CASE("segment_instances: Cluster tolerance sensitivity",
          "[geometry][segment_instances]") {
  // Create 2 clusters 1m apart
  std::vector<Eigen::Vector3f> centers = {{0.0F, 0.0F, 0.0F},
                                            {1.0F, 0.0F, 0.0F}};
  int points_per_cluster = 100;

  auto cloud = create_clustered_cloud(centers, points_per_cluster, 0.1F);
  auto semantic_labels = create_semantic_labels({1, 1}, points_per_cluster);

  SECTION("Small tolerance: 2 separate instances") {
    SegmentInstancesRequest request;
    request.cloud = cloud;
    request.semantic_labels = semantic_labels;
    request.cluster_tolerance = 0.3F; // Too small to bridge 1m gap
    request.min_cluster_size = 50;

    auto result = segment_instances(request);
    REQUIRE(result.instance_to_semantic.size() == 2);
  }

  SECTION("Large tolerance: 1 merged instance") {
    SegmentInstancesRequest request;
    request.cloud = cloud;
    request.semantic_labels = semantic_labels;
    request.cluster_tolerance = 1.5F; // Large enough to merge
    request.min_cluster_size = 50;

    auto result = segment_instances(request);
    REQUIRE(result.instance_to_semantic.size() == 1);
    REQUIRE(result.instance_sizes.begin()->second == static_cast<size_t>(2 * points_per_cluster));
  }
}

TEST_CASE("segment_instances: Min cluster size filtering",
          "[geometry][segment_instances]") {
  // Create 3 clusters with different sizes
  auto cloud = CloudPtr(new Cloud);
  auto semantic_labels = CloudLPtr(new CloudL);

  // Large cluster: 200 points at origin
  for (int i = 0; i < 200; ++i) {
    cloud->push_back(PointT{0.0F, 0.0F, 0.0F, 255, 255, 255});
    semantic_labels->push_back(pcl::Label{1});
  }

  // Medium cluster: 100 points at (5, 0, 0)
  for (int i = 0; i < 100; ++i) {
    cloud->push_back(PointT{5.0F, 0.0F, 0.0F, 255, 255, 255});
    semantic_labels->push_back(pcl::Label{1});
  }

  // Small cluster: 20 points at (10, 0, 0)
  for (int i = 0; i < 20; ++i) {
    cloud->push_back(PointT{10.0F, 0.0F, 0.0F, 255, 255, 255});
    semantic_labels->push_back(pcl::Label{1});
  }

  SECTION("Min size = 50: Filters out small cluster") {
    SegmentInstancesRequest request;
    request.cloud = cloud;
    request.semantic_labels = semantic_labels;
    request.cluster_tolerance = 0.5F;
    request.min_cluster_size = 50;

    auto result = segment_instances(request);
    REQUIRE(result.instance_to_semantic.size() == 2);

    // Verify only large and medium clusters remain
    std::vector<size_t> sizes;
    for (const auto &[id, size] : result.instance_sizes) {
      sizes.push_back(size);
    }
    std::sort(sizes.begin(), sizes.end());
    REQUIRE(sizes[0] == 100);
    REQUIRE(sizes[1] == 200);
  }

  SECTION("Min size = 10: Includes all clusters") {
    SegmentInstancesRequest request;
    request.cloud = cloud;
    request.semantic_labels = semantic_labels;
    request.cluster_tolerance = 0.5F;
    request.min_cluster_size = 10;

    auto result = segment_instances(request);
    REQUIRE(result.instance_to_semantic.size() == 3);
  }
}

TEST_CASE("segment_instances: Label filtering",
          "[geometry][segment_instances]") {
  // Create 4 clusters: 2 of class 1, 2 of class 2
  std::vector<Eigen::Vector3f> centers = {{0.0F, 0.0F, 0.0F},
                                            {5.0F, 0.0F, 0.0F},
                                            {0.0F, 5.0F, 0.0F},
                                            {5.0F, 5.0F, 0.0F}};
  int points_per_cluster = 100;

  auto cloud = create_clustered_cloud(centers, points_per_cluster, 0.1F);
  auto semantic_labels = create_semantic_labels({1, 1, 2, 2}, points_per_cluster);

  SECTION("Process only class 1") {
    SegmentInstancesRequest request;
    request.cloud = cloud;
    request.semantic_labels = semantic_labels;
    request.cluster_tolerance = 0.5F;
    request.min_cluster_size = 50;
    request.labels_to_process = {1}; // Only class 1

    auto result = segment_instances(request);

    // Should create 2 instances (only class 1 clusters)
    REQUIRE(result.instance_to_semantic.size() == 2);

    // All instances should be class 1
    for (const auto &[id, semantic_class] : result.instance_to_semantic) {
      REQUIRE(semantic_class == 1);
    }

    // Only half the points should be labeled
    size_t labeled = 0;
    for (const auto &label : *result.instance_labels) {
      if (label.label > 0)
        ++labeled;
    }
    REQUIRE(labeled == static_cast<size_t>(2 * points_per_cluster));
  }
}

TEST_CASE("segment_instances: Edge cases", "[geometry][segment_instances]") {
  SECTION("Empty cloud") {
    CloudPtr cloud(new Cloud);
    CloudLPtr labels(new CloudL);

    SegmentInstancesRequest request;
    request.cloud = cloud;
    request.semantic_labels = labels;

    REQUIRE_THROWS_AS(segment_instances(request), std::invalid_argument);
  }

  SECTION("All unlabeled (label = 0)") {
    auto cloud =
        create_clustered_cloud({{0.0F, 0.0F, 0.0F}}, 100, 0.1F);
    auto labels = create_semantic_labels({0}, 100); // All background

    SegmentInstancesRequest request;
    request.cloud = cloud;
    request.semantic_labels = labels;
    request.cluster_tolerance = 0.5F;
    request.min_cluster_size = 50;

    auto result = segment_instances(request);

    // Should return empty result (no instances)
    REQUIRE(result.instance_to_semantic.empty());
    REQUIRE(result.instance_sizes.empty());
  }

  SECTION("Size mismatch") {
    auto cloud = create_clustered_cloud({{0.0F, 0.0F, 0.0F}}, 100, 0.1F);
    auto labels = create_semantic_labels({1}, 50); // Wrong size

    SegmentInstancesRequest request;
    request.cloud = cloud;
    request.semantic_labels = labels;

    REQUIRE_THROWS_AS(segment_instances(request), std::invalid_argument);
  }

  SECTION("Null inputs") {
    SegmentInstancesRequest request;
    request.cloud = nullptr;
    request.semantic_labels = CloudLPtr(new CloudL);

    REQUIRE_THROWS_AS(segment_instances(request), std::invalid_argument);
  }

  SECTION("Invalid tolerance") {
    auto cloud = create_clustered_cloud({{0.0F, 0.0F, 0.0F}}, 100, 0.1F);
    auto labels = create_semantic_labels({1}, 100);

    SegmentInstancesRequest request;
    request.cloud = cloud;
    request.semantic_labels = labels;
    request.cluster_tolerance = -0.5F; // Invalid

    REQUIRE_THROWS_AS(segment_instances(request), std::invalid_argument);
  }
}

TEST_CASE("segment_instances: Cancellation",
          "[geometry][segment_instances]") {
  auto cloud =
      create_clustered_cloud({{0.0F, 0.0F, 0.0F}}, 100, 0.1F);
  auto labels = create_semantic_labels({1}, 100);

  std::atomic_bool cancel_flag{true}; // Pre-cancelled

  SegmentInstancesRequest request;
  request.cloud = cloud;
  request.semantic_labels = labels;
  request.cluster_tolerance = 0.5F;
  request.min_cluster_size = 50;
  request.cancel_token = &cancel_flag;

  REQUIRE_THROWS_AS(segment_instances(request), std::runtime_error);
}

TEST_CASE("segment_instances: Sequential instance IDs",
          "[geometry][segment_instances]") {
  // Verify instance IDs are sequential (1, 2, 3, ...)
  std::vector<Eigen::Vector3f> centers = {{0.0F, 0.0F, 0.0F},
                                            {5.0F, 0.0F, 0.0F},
                                            {10.0F, 0.0F, 0.0F}};
  auto cloud = create_clustered_cloud(centers, 100, 0.1F);
  auto labels = create_semantic_labels({1, 1, 1}, 100);

  SegmentInstancesRequest request;
  request.cloud = cloud;
  request.semantic_labels = labels;
  request.cluster_tolerance = 0.5F;
  request.min_cluster_size = 50;

  auto result = segment_instances(request);

  std::vector<uint32_t> instance_ids;
  for (const auto &[id, semantic] : result.instance_to_semantic) {
    instance_ids.push_back(id);
  }
  std::sort(instance_ids.begin(), instance_ids.end());

  REQUIRE(instance_ids.size() == 3);
  REQUIRE(instance_ids[0] == 1);
  REQUIRE(instance_ids[1] == 2);
  REQUIRE(instance_ids[2] == 3);
}
