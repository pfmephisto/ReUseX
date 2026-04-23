// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "geometry/reconstruct.hpp"
#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"
#include "core/processing_observer.hpp"
#include "geometry/depth_filters.hpp"
#include "utils/fmt_formatter.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <fmt/format.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <nlohmann/json.hpp>

#include <cstring>
#include <unordered_map>

namespace reusex::geometry {

namespace {

/// Build a 4x4 Eigen affine from a row-major double[16] array.
Eigen::Affine3f to_affine(const std::array<double, 16> &m) {
  Eigen::Matrix4f mat;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      mat(r, c) = static_cast<float>(m[r * 4 + c]);
  Eigen::Affine3f aff;
  aff.matrix() = mat;
  return aff;
}

} // anonymous namespace

void reconstruct_point_clouds(ProjectDB &db,
                              const ReconstructionParams &params) {
  core::info("Reconstructing point clouds from sensor frames");
  core::stopwatch timer;

  auto frameIds = db.sensor_frame_ids();
  if (frameIds.empty()) {
    core::warn("No sensor frames in database, nothing to reconstruct");
    return;
  }

  core::info("Processing {} sensor frames", frameIds.size());

  // Accumulated clouds (serial accumulation; per-frame work dominates)
  auto merged_cloud = std::make_shared<Cloud>();
  auto merged_normals = std::make_shared<CloudN>();
  auto merged_labels = std::make_shared<CloudL>();

  auto observer =
      core::ProgressObserver(core::Stage::assembling_cloud, frameIds.size());

  for (int nodeId : frameIds) {
    // ── Read from ProjectDB ──────────────────────────────────────
    cv::Mat color = db.sensor_frame_image(nodeId);
    cv::Mat depth16 = db.sensor_frame_depth(nodeId);   // CV_16UC1 mm
    cv::Mat confidence = db.sensor_frame_confidence(nodeId); // CV_8UC1
    auto pose = db.sensor_frame_pose(nodeId);
    auto intr = db.sensor_frame_intrinsics(nodeId);

    if (color.empty() || depth16.empty()) {
      core::debug("Node {} missing color or depth, skipping", nodeId);
      ++observer;
      continue;
    }

    // Convert depth to float meters for filtering
    cv::Mat depth_f;
    depth16.convertTo(depth_f, CV_32FC1, 1.0 / 1000.0);

    // ── Depth filters ────────────────────────────────────────────
    apply_depth_discontinuity_filter(depth_f, confidence, 0.5f);
    apply_ray_consistency_filter(depth_f, confidence, 0.2f);

    // ── Build per-frame transforms ───────────────────────────────
    Eigen::Affine3f localTf = to_affine(intr.local_transform);
    Eigen::Affine3f worldTf = to_affine(pose);

    // Debug logging for first frame to verify transform correctness
    if (nodeId == frameIds[0]) {
      core::debug("=== First frame transform verification (nodeId={}) ===", nodeId);
      core::debug("localTf rotation:\n{}", localTf.rotation().format(OctaveFmt));
      core::debug("worldTf translation: [{:.3f}, {:.3f}, {:.3f}]",
                  worldTf.translation().x(), worldTf.translation().y(),
                  worldTf.translation().z());
    }

    const float fx = static_cast<float>(intr.fx);
    const float fy = static_cast<float>(intr.fy);
    const float cx_cam = static_cast<float>(intr.cx);
    const float cy_cam = static_cast<float>(intr.cy);
    const int step = params.sampling_factor;
    const float min_d = params.min_distance;
    const float max_d = params.max_distance;
    const int conf_thresh = params.confidence_threshold;

    // ── Read segmentation labels if available ────────────────────
    cv::Mat seg_labels; // CV_32S or empty
    if (db.has_segmentation_image(nodeId))
      seg_labels = db.segmentation_image(nodeId); // CV_32S, -1 = bg

    // ── Pinhole back-projection ──────────────────────────────────
    auto frame_cloud = std::make_shared<Cloud>();
    auto frame_labels = std::make_shared<CloudL>();

    size_t estimated_pts =
        static_cast<size_t>((depth_f.rows / step) * (depth_f.cols / step));
    frame_cloud->reserve(estimated_pts);
    frame_labels->reserve(estimated_pts);

    for (int v = 0; v < depth_f.rows; v += step) {
      const float *depth_row = depth_f.ptr<float>(v);
      const uchar *conf_row =
          confidence.empty() ? nullptr : confidence.ptr<uchar>(v);

      for (int u = 0; u < depth_f.cols; u += step) {
        float z = depth_row[u];
        if (z <= 0.0f)
          continue;
        if (z < min_d || z > max_d)
          continue;
        if (conf_row && conf_row[u] < conf_thresh)
          continue;

        // Camera-frame 3D point in optical frame
        float x = (static_cast<float>(u) - cx_cam) * z / fx;
        float y = (static_cast<float>(v) - cy_cam) * z / fy;

        Eigen::Vector3f cam_pt(x, y, z);
        // Apply local transform (optical → sensor) then world transform (sensor → world)
        Eigen::Vector3f world_pt = worldTf * (localTf * cam_pt);

        // Get color from the color image
        const auto &pixel = color.at<cv::Vec3b>(v, u);

        PointT pt;
        pt.x = world_pt.x();
        pt.y = world_pt.y();
        pt.z = world_pt.z();
        pt.b = pixel[0];
        pt.g = pixel[1];
        pt.r = pixel[2];
        frame_cloud->push_back(pt);

        // Label
        LabelT lbl;
        if (!seg_labels.empty() && v < seg_labels.rows && u < seg_labels.cols)
          lbl.label = static_cast<uint32_t>(seg_labels.at<int>(v, u));
        else
          lbl.label = static_cast<uint32_t>(-1);
        frame_labels->push_back(lbl);
      }
    }

    if (frame_cloud->empty()) {
      ++observer;
      continue;
    }

    // ── Normal estimation ────────────────────────────────────────
    pcl::NormalEstimationOMP<PointT, NormalT> ne;
    ne.setInputCloud(frame_cloud);
    ne.setRadiusSearch(0.1);

    // Use world pose translation as viewpoint
    float vx = static_cast<float>(pose[3]);
    float vy = static_cast<float>(pose[7]);
    float vz = static_cast<float>(pose[11]);
    ne.setViewPoint(vx, vy, vz);

    auto frame_normals = std::make_shared<CloudN>();
    ne.compute(*frame_normals);

    // ── Accumulate ───────────────────────────────────────────────
    *merged_cloud += *frame_cloud;
    *merged_normals += *frame_normals;
    *merged_labels += *frame_labels;

    core::trace("Node {}: {} points", nodeId, frame_cloud->size());
    ++observer;
  }

  if (merged_cloud->empty()) {
    core::error("No points generated from any sensor frame");
    return;
  }

  core::info("Accumulated {} points from {} frames", merged_cloud->size(),
             frameIds.size());

  // ── Voxelization ─────────────────────────────────────────────────
  core::info("Voxel grid filtering (resolution={}, {} points)",
             params.resolution, merged_cloud->size());
  timer.reset();

  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(merged_cloud);
  vg.setLeafSize(params.resolution, params.resolution, params.resolution);
  auto ds_cloud = std::make_shared<Cloud>();
  vg.filter(*ds_cloud);

  core::debug("Downsampled to {} points in {:.3f}s", ds_cloud->size(), timer);

  // ── Remove NaN normals ───────────────────────────────────────────
  // Build KdTree on original cloud for label majority voting
  if (merged_cloud->size() < 10) {
    core::error("Merged cloud too small for KdTree ({} points)",
                merged_cloud->size());
    return;
  }

  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(merged_cloud);

  // ── Normal estimation on downsampled cloud ───────────────────────
  // Re-estimate normals on the voxelized cloud for consistency
  pcl::NormalEstimationOMP<PointT, NormalT> ne_ds;
  ne_ds.setInputCloud(ds_cloud);
  ne_ds.setRadiusSearch(0.1);
  auto ds_normals = std::make_shared<CloudN>();
  ne_ds.compute(*ds_normals);

  // Remove NaN normals
  pcl::Indices nan_filter;
  pcl::removeNaNNormalsFromPointCloud(*ds_normals, *ds_normals, nan_filter);

  // Apply same filter to cloud
  auto ds_cloud_filtered = std::make_shared<Cloud>();
  pcl::ExtractIndices<PointT> extract_nan;
  extract_nan.setInputCloud(ds_cloud);
  auto nan_indices = std::make_shared<pcl::Indices>(nan_filter);
  extract_nan.setIndices(nan_indices);
  extract_nan.filter(*ds_cloud_filtered);
  ds_cloud = ds_cloud_filtered;

  // ── Label majority voting ────────────────────────────────────────
  core::info("Label majority voting");
  auto ds_labels = std::make_shared<CloudL>();
  ds_labels->resize(ds_cloud->size());

  const float half_res = params.resolution / 2.0f;
  for (size_t i = 0; i < ds_cloud->size(); ++i) {
    std::vector<int> indices;
    std::vector<float> distances;
    kdtree.radiusSearch((*ds_cloud)[i], half_res, indices, distances);

    std::unordered_map<int, int> counts;
    for (int idx : indices) {
      int label = static_cast<int>((*merged_labels)[idx].label);
      counts[label]++;
    }

    int best_label = -1;
    int best_count = 0;
    for (const auto &[label, count] : counts) {
      if (count > best_count) {
        best_count = count;
        best_label = label;
      }
    }
    (*ds_labels)[i].label = static_cast<uint32_t>(best_label);
  }

  // ── Outlier removal ──────────────────────────────────────────────
  core::info("Filtering outliers ({} points)", ds_cloud->size());
  timer.reset();

  // Statistical outlier removal
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(ds_cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  auto stat_inliers = std::make_shared<pcl::Indices>();
  sor.filter(*stat_inliers);
  core::debug("SOR kept {}/{}", stat_inliers->size(), ds_cloud->size());

  // Radius outlier removal
  pcl::RadiusOutlierRemoval<PointT> ror;
  ror.setInputCloud(ds_cloud);
  ror.setIndices(stat_inliers);
  ror.setRadiusSearch(params.resolution * 2.0);
  ror.setMinNeighborsInRadius(5);
  auto filtered_indices = std::make_shared<pcl::Indices>();
  ror.filter(*filtered_indices);
  core::debug("ROR kept {}/{}", filtered_indices->size(),
              stat_inliers->size());

  // Extract filtered results
  auto out_cloud = std::make_shared<Cloud>();
  pcl::ExtractIndices<PointT> ext_pts;
  ext_pts.setInputCloud(ds_cloud);
  ext_pts.setIndices(filtered_indices);
  ext_pts.filter(*out_cloud);

  auto out_normals = std::make_shared<CloudN>();
  pcl::ExtractIndices<NormalT> ext_nrm;
  ext_nrm.setInputCloud(ds_normals);
  ext_nrm.setIndices(filtered_indices);
  ext_nrm.filter(*out_normals);

  auto out_labels = std::make_shared<CloudL>();
  pcl::ExtractIndices<LabelT> ext_lbl;
  ext_lbl.setInputCloud(ds_labels);
  ext_lbl.setIndices(filtered_indices);
  ext_lbl.filter(*out_labels);

  core::info("Filtering complete in {:.3f}s: {}/{} points ({:.1f}%)", timer,
             out_cloud->size(), ds_cloud->size(),
             100.0 * out_cloud->size() / ds_cloud->size());

  // ── Save to ProjectDB ────────────────────────────────────────────
  core::info("Saving point clouds to database");
  std::string paramsJson = fmt::format(
      R"({{"resolution":{},"min_distance":{},"max_distance":{},"sampling_factor":{},"confidence_threshold":{}}})",
      params.resolution, params.min_distance, params.max_distance,
      params.sampling_factor, params.confidence_threshold);

  if (!out_cloud->empty())
    db.save_point_cloud("cloud", *out_cloud, "reconstruct", paramsJson);

  if (!out_normals->empty())
    db.save_point_cloud("normals", *out_normals, "reconstruct", paramsJson);

  if (!out_labels->empty()) {
    db.save_point_cloud("labels", *out_labels, "reconstruct", paramsJson);

    // Copy class names from annotation pipeline log to label_definitions
    for (const auto &entry : db.pipeline_log()) {
      if (entry.stage == "annotate_class_map" && !entry.parameters.empty()) {
        try {
          auto j = nlohmann::json::parse(entry.parameters);
          std::map<int, std::string> class_map;
          for (auto &[key, val] : j.items())
            class_map[std::stoi(key)] = val.get<std::string>();
          if (!class_map.empty())
            db.save_label_definitions("labels", class_map);
        } catch (const std::exception &e) {
          core::warn("Failed to parse annotation class map: {}", e.what());
        }
        break;
      }
    }
  }

  core::info("Reconstruction complete: {} points, {} normals, {} labels",
             out_cloud->size(), out_normals->size(), out_labels->size());
}

} // namespace reusex::geometry
