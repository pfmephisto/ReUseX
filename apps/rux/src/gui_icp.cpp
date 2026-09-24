// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// ICP-based relative-pose refinement for POST /api/v1/posegraph/icp (#465).
//
// LAYERING: PCL registration headers are heavy. This file lives in rux_lib
// (apps/rux/src/), which links the `reusex` umbrella and inherits PCL.
// rux_gui_lib must NOT link this file — the Server stores an IcpRefineFn
// callback (std::function) that gui.cpp fills in via
// ServerOptions::icp_refine_fn.

#include "gui_icp.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>

#include <Eigen/Core>

#include <opencv2/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>

#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

namespace rux {
namespace {

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

// Mirror the parameters used by segmentation/reconstruct.cpp so ICP sees the
// same density and depth range as the merged cloud pipeline.
constexpr int kStep = 4; // sample every 4th pixel
constexpr float kMinDepthM = 0.3f;
constexpr float kMaxDepthM = 4.0f;
constexpr int kMaxIter = 50;            // STANDARDS §6: deterministic bound
constexpr double kMaxCorrM = 0.5;       // 50 cm initial correspondence window
constexpr float kInlierThreshM = 0.05f; // 5 cm inlier gate

/// Back-project a stored depth frame into a PCL cloud in world coordinates.
/// Depth is CV_16UC1 in millimetres; result is in metres.
CloudT::Ptr depth_to_world_cloud(const reusex::ProjectDB &db, int node_id) {
  const cv::Mat depth16 = db.sensor_frame_depth(node_id);
  if (depth16.empty())
    throw std::runtime_error("frame " + std::to_string(node_id) +
                             " has no stored depth image");
  if (!db.has_sensor_frame_pose(node_id))
    throw std::runtime_error("frame " + std::to_string(node_id) +
                             " has no stored pose");

  const auto pose_arr = db.sensor_frame_pose(node_id);
  const auto intr = db.sensor_frame_intrinsics(node_id);

  cv::Mat depth_f;
  depth16.convertTo(depth_f, CV_32FC1, 1.0 / 1000.0);

  // Scale intrinsics to depth resolution (same logic as reconstruct.cpp).
  const double sx = static_cast<double>(depth_f.cols) / std::max(1, intr.width);
  const double sy =
      static_cast<double>(depth_f.rows) / std::max(1, intr.height);
  const float fx = static_cast<float>(intr.fx * sx);
  const float fy = static_cast<float>(intr.fy * sy);
  const float cx = static_cast<float>(intr.cx * sx);
  const float cy = static_cast<float>(intr.cy * sy);

  Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> world_m(
      pose_arr.data());
  Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> local_m(
      intr.local_transform.data());
  const Eigen::Matrix4f world_tf = world_m.cast<float>();
  const Eigen::Matrix4f local_tf = local_m.cast<float>();

  auto cloud = std::make_shared<CloudT>();
  cloud->reserve(static_cast<size_t>(depth_f.rows / kStep + 1) *
                 static_cast<size_t>(depth_f.cols / kStep + 1));

  for (int v = 0; v < depth_f.rows; v += kStep) {
    const float *row = depth_f.ptr<float>(v);
    for (int u = 0; u < depth_f.cols; u += kStep) {
      const float z = row[u];
      if (z < kMinDepthM || z > kMaxDepthM)
        continue;
      const float x = (static_cast<float>(u) - cx) * z / fx;
      const float y = (static_cast<float>(v) - cy) * z / fy;
      const Eigen::Vector4f cam_pt(x, y, z, 1.0f);
      const Eigen::Vector4f world_pt = world_tf * (local_tf * cam_pt);
      cloud->emplace_back(world_pt[0], world_pt[1], world_pt[2]);
    }
  }
  cloud->is_dense = true;
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  return cloud;
}

} // anonymous namespace

gui::IcpRefineFn make_icp_refine_fn() {
  return [](const reusex::ProjectDB &db, int from_id,
            int to_id) -> gui::IcpRefineResult {
    auto src = depth_to_world_cloud(db, from_id);
    auto dst = depth_to_world_cloud(db, to_id);

    if (src->empty())
      throw std::runtime_error(
          "frame " + std::to_string(from_id) +
          " produced an empty cloud (all depth out of valid range)");
    if (dst->empty())
      throw std::runtime_error(
          "frame " + std::to_string(to_id) +
          " produced an empty cloud (all depth out of valid range)");

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(src);
    icp.setInputTarget(dst);
    icp.setMaximumIterations(kMaxIter);
    icp.setMaxCorrespondenceDistance(kMaxCorrM);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-4);

    CloudT aligned;
    icp.align(aligned);

    const Eigen::Matrix4f T_delta = icp.getFinalTransformation();
    const bool converged = icp.hasConverged();
    // getFitnessScore() returns MSE; convert to RMS.
    const double fitness_rms = std::sqrt(std::max(0.0, icp.getFitnessScore()));

    // Relative pose: T_to^{-1} @ T_delta @ T_from (maps from-cam → to-cam).
    const auto from_arr = db.sensor_frame_pose(from_id);
    const auto to_arr = db.sensor_frame_pose(to_id);
    Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> T_from(
        from_arr.data());
    Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> T_to(
        to_arr.data());
    const Eigen::Matrix4d T_rel =
        T_to.inverse() * T_delta.cast<double>() * T_from;

    // Inlier fraction: aligned source points within kInlierThreshM of target.
    double inlier_fraction = 0.0;
    if (!aligned.empty() && !dst->empty()) {
      pcl::search::KdTree<PointT> tree;
      tree.setInputCloud(dst);
      const float sq_thresh = kInlierThreshM * kInlierThreshM;
      int inliers = 0;
      for (const auto &pt : aligned) {
        std::vector<int> idx(1);
        std::vector<float> sq_dist(1);
        if (tree.nearestKSearch(pt, 1, idx, sq_dist) > 0 &&
            sq_dist[0] <= sq_thresh)
          ++inliers;
      }
      inlier_fraction =
          static_cast<double>(inliers) / static_cast<double>(src->size());
    }

    gui::IcpRefineResult result;
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j)
        result.relative_pose[static_cast<size_t>(i * 4 + j)] = T_rel(i, j);
    result.fitness = fitness_rms;
    result.inlier_fraction = inlier_fraction;
    result.converged = converged;
    return result;
  };
}

} // namespace rux
