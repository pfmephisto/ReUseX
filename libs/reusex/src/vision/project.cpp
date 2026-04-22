// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "vision/project.hpp"
#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"
#include "core/processing_observer.hpp"
#include "types.hpp"

#include <range/v3/view/iota.hpp>

#include <fmt/std.h>

#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/core/util3d.h>

#include <pcl/filters/frustum_culling.h>

#include <cmath>
#include <limits>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

using namespace rtabmap;

namespace ReUseX::vision {

namespace {

/// @brief Create an rtabmap::CameraModel from ProjectDB SensorIntrinsics
inline rtabmap::CameraModel
createCameraModelFromIntrinsics(const ReUseX::core::SensorIntrinsics &intrinsics,
                                 const cv::Size &targetSize) {
  // Build 3x3 intrinsic matrix K
  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  K.at<double>(0, 0) = intrinsics.fx;
  K.at<double>(1, 1) = intrinsics.fy;
  K.at<double>(0, 2) = intrinsics.cx;
  K.at<double>(1, 2) = intrinsics.cy;

  // Convert local_transform to rtabmap::Transform (expects 3x4 matrix)
  // NOTE: Data is stored row-major, so we must use RowMajor mapping
  using Matrix4dRM = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
  Matrix4dRM local_eigen = Eigen::Map<const Matrix4dRM>(
      intrinsics.local_transform.data());
  Eigen::Matrix4f local_float = local_eigen.cast<float>();
  cv::Mat local_mat_4x4;
  cv::eigen2cv(local_float, local_mat_4x4);
  // Extract first 3 rows for rtabmap::Transform (3x4 format)
  cv::Mat local_mat = local_mat_4x4(cv::Rect(0, 0, 4, 3));
  rtabmap::Transform localTransform(local_mat);

  // Create CameraModel (no distortion, no rectification)
  cv::Mat D;  // Empty distortion
  cv::Mat R = cv::Mat::eye(3, 3, CV_64F);  // No rotation
  cv::Mat P;  // Empty projection matrix (will be computed from K)

  cv::Size imageSize(intrinsics.width, intrinsics.height);
  rtabmap::CameraModel cm("ProjectDB", imageSize, K, D, R, P, localTransform);

  // Scale to target size if needed
  if (targetSize != imageSize) {
    const double scaleX = static_cast<double>(targetSize.width) / imageSize.width;
    const double scaleY = static_cast<double>(targetSize.height) / imageSize.height;

    cv::Mat K_scaled = K.clone();
    K_scaled.at<double>(0, 0) *= scaleX;  // fx
    K_scaled.at<double>(1, 1) *= scaleY;  // fy
    K_scaled.at<double>(0, 2) *= scaleX;  // cx
    K_scaled.at<double>(1, 2) *= scaleY;  // cy

    return rtabmap::CameraModel("ProjectDB", targetSize, K_scaled, D, R, P, localTransform);
  }

  return cm;
}

} // namespace

auto project(ProjectDB &db, CloudConstPtr cloud) -> CloudLPtr {
  ReUseX::trace("calling project");

  // auto viewer =
  //     std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
  // viewer->setBackgroundColor(0.1, 0.1, 0.1);
  // viewer->addCoordinateSystem(0.5);
  // viewer->initCameraParameters();

  // Create output labeled point cloud
  CloudLPtr labels(new CloudL);
  labels->points.resize(cloud->points.size());
  labels->width = cloud->width;
  labels->height = cloud->height;
  labels->is_dense = cloud->is_dense;

  ReUseX::info("Loading sensor frames from ProjectDB ...");
  ReUseX::core::stopwatch timer;
  auto frameIds = db.sensor_frame_ids();
  ReUseX::debug("Loaded {} sensor frames in {:.3f}s", frameIds.size(), timer);
  timer.reset();

  // viewer->addPointCloud<Cloud::PointType>(cloud, "input_cloud");
  // viewer->spinOnce();

  const auto rtabmapCam_T_pclCam =
      rtabmap::Transform(1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0);

  /* Dicription of the different coordinate frames
   *
   * Frames:
   *  - World: RTAB-Map odometry frame
   *  - Base: RTAB-Map local frame (node pose)
   *  - Camera: RTAB-Map camera frame
   *  - Image: Image plane
   *
   * Transforms:
   *  - pose: Base to World
   *  - cm.localTransform(): Camera to Base
   *  - cm.K(): Image to Camera
   *
   *  - ?: RTABMap to PCL
   */

  {
    auto observer = std::make_shared<ReUseX::core::ProgressObserver>(
        ReUseX::core::Stage::projecting_labels, frameIds.size());
    for (size_t i = 0; i < frameIds.size(); ++i) {
      int id = frameIds[i];
      ReUseX::trace("Processing node {}/{}", i + 1, frameIds.size());

      // --- Fetch label image
      cv::Mat labeledImage = db.segmentation_image(id); // CV_32S
      if (labeledImage.empty()) {
        ReUseX::warn("No segmentation for node {}, skipping", id);
        ++(*observer);
        continue;
      }

      // --- Fetch sensor data from ProjectDB
      auto pose_array = db.sensor_frame_pose(id);  // std::array<double, 16>
      auto intrinsics = db.sensor_frame_intrinsics(id);  // SensorIntrinsics

      // Convert pose to rtabmap::Transform (expects 3x4 matrix)
      // NOTE: Data is stored row-major (see MEMORY.md), so we must use RowMajor mapping
      using Matrix4dRM = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
      Matrix4dRM pose_eigen = Eigen::Map<const Matrix4dRM>(pose_array.data());
      Eigen::Matrix4f pose_float = pose_eigen.cast<float>();
      cv::Mat pose_mat_4x4;
      cv::eigen2cv(pose_float, pose_mat_4x4);
      // Extract first 3 rows for rtabmap::Transform (3x4 format)
      cv::Mat pose_mat = pose_mat_4x4(cv::Rect(0, 0, 4, 3));
      rtabmap::Transform pose(pose_mat);

      // Convert local_transform to rtabmap::Transform (expects 3x4 matrix)
      // NOTE: Data is stored row-major, so we must use RowMajor mapping
      Matrix4dRM local_eigen = Eigen::Map<const Matrix4dRM>(
          intrinsics.local_transform.data());
      Eigen::Matrix4f local_float = local_eigen.cast<float>();
      cv::Mat local_mat_4x4;
      cv::eigen2cv(local_float, local_mat_4x4);
      // Extract first 3 rows for rtabmap::Transform (3x4 format)
      cv::Mat local_mat = local_mat_4x4(cv::Rect(0, 0, 4, 3));
      rtabmap::Transform t_cam(local_mat);

      std::filesystem::path tmpPath = "./debth_debug";
      if (!std::filesystem::exists(tmpPath))
        std::filesystem::create_directory(tmpPath);

      // Create scaled camera model for projection
      rtabmap::CameraModel cm = createCameraModelFromIntrinsics(
          intrinsics, cv::Size(intrinsics.width / 16, intrinsics.height / 16));

      auto t_local = pose.inverse();

      // Frustum culling setup
      // pcl::FrustumCulling<Cloud::PointType> fc;
      // fc.setInputCloud(cloud);
      // fc.setCameraPose(
      //    (pose * cm.localTransform().inverse() * rtabmapCam_T_pclCam)
      //        .toEigen4f());
      // fc.setNearPlaneDistance(0.05f);
      // fc.setFarPlaneDistance(50.0f); // wide enough to include valid points
      // auto [fovYdeg, fovXdeg] = fovsFromCameraModel(cm);
      // fc.setVerticalFOV(static_cast<float>(fovYdeg));
      // fc.setHorizontalFOV(static_cast<float>(fovXdeg));

      {
        auto visual_observer = ReUseX::core::get_visual_observer();
        auto camera_pose =
            (pose * cm.localTransform().inverse()).toEigen3f();
        visual_observer->viewer_add_camera_frustum(
            fmt::format("camera_{}", id), cm.fx(), cm.fy(),
            cm.imageSize().width, cm.imageSize().height,
            Eigen::Affine3f(camera_pose),
            ReUseX::core::Stage::projecting_labels, static_cast<int>(i));
      }

      pcl::Indices indices;
      // fc.filter(indices);
      indices.resize(cloud->points.size());
      std::iota(indices.begin(), indices.end(), 0);

      // INFO: Visualize frustum culled points
      //{
      //  // --- Visualize frustum culled points
      //  CloudPtr result(new Cloud);
      //  fc.filter(*result);
      //  auto red_hander =
      //      pcl::visualization::PointCloudColorHandlerCustom<Cloud::PointType>(
      //          result, 255, 0, 0);
      //  viewer->addPointCloud<Cloud::PointType>(result, red_hander,
      //                                          fmt::format("cloud_node_{}",
      //                                          id));
      //  viewer->setPointCloudRenderingProperties(
      //      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,
      //      fmt::format("cloud_node_{}", id));
      //}

      std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_lf(
          new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(*cloud, *cloud_lf);
      cloud_lf = rtabmap::util3d::transformPointCloud(
          cloud_lf, t_local); // Mover cloud to link frame

      // --- Build Z-buffer from frustum points and fill holes
      cv::Mat zbuffer = rtabmap::util3d::projectCloudToCamera(
          cm.imageSize() /*labeledImage.size()*/, cm.K_raw(), cloud_lf, t_cam);

      ReUseX::trace("Filling Z-buffer holes for node {}", id);
      // rtabmap::util2d::fillDepthHoles(zbuffer, 1, 0.50f);
      // rtabmap::util2d::fillRegisteredDepthHoles(zbuffer, true, false, false);
      rtabmap::util3d::fillProjectedCloudHoles(zbuffer, true, true);

      cv::resize(zbuffer, zbuffer, labeledImage.size());

      // cv::Mat zbufferDisplay = zbuffer.clone();
      // cv::normalize(zbufferDisplay, zbufferDisplay, 0.0, 1.0,
      // cv::NORM_MINMAX); cv::imshow("Z-Buffer", zbufferDisplay);

      // cv::Mat labeledDisplay = cv::Mat(labeledImage.size(), CV_8UC3);
      // for (size_t i = 0; i < labeledImage.total(); ++i) {
      //   auto color = pcl::GlasbeyLUT::at(labeledImage.at<int>(i) %
      //                                    pcl::GlasbeyLUT::size());
      //   labeledDisplay.at<cv::Vec3b>(i) = cv::Vec3b(color.b, color.g,
      //   color.r);
      // }
      // cv::imshow("labeled", labeledDisplay);

      // cv::imshow("input", data.imageRaw());
      // cv::waitKey(0);

      // while (!viewer->wasStopped()) {
      //   viewer->spinOnce(100);
      //   std::this_thread::sleep_for(std::chrono::milliseconds(100));
      // }

      // cv::imwrite(tmpPath / fmt::format("zbuffer_node_{}.png", id),
      //             normalizeDepthImage(zbuffer) /* zbufferDisplay * 255.0*/);

      cv::Mat temp(labeledImage.size(), CV_8UC3);
      temp.setTo(cv::Vec3b(0, 0, 0));

      // Scale camera model to match label image resolution
      cm = createCameraModelFromIntrinsics(intrinsics, labeledImage.size());
      {
        ReUseX::trace("Assigning labels for node {}", id);
        // Assign labels to the points in the point cloud
        auto fx = cm.fx();
        auto fy = cm.fy();
        auto cx = cm.cx();
        auto cy = cm.cy();
        auto h = cm.imageHeight();
        auto w = cm.imageWidth();
#pragma omp parallel for shared(labels, cloud, indices, labeledImage, zbuffer, \
                                    temp)                                      \
    firstprivate(t_cam, fx, fy, cx, cy, w, h, tmpPath)
        for (int j = 0; j < static_cast<int>(indices.size()); ++j) {
          auto i = indices[j];

          auto pt_cam = rtabmap::util3d::transformPoint(cloud_lf->points[i],
                                                        t_cam.inverse());
          if (pt_cam.z <= 0.0f ||
              pt_cam.z >
                  7.0f) // TODO: Make maximum depth threshold configurable
            // category=Vision estimate=30m
            // Hardcoded 7.0m cutoff may not suit all sensors (LiDAR has longer
            // range). Should add as function parameter or read from config
            // file. Typical values: 3-5m for depth cameras, 10-30m for LiDAR
            continue;

          auto inv_z = 1.0 / static_cast<double>(pt_cam.z);

          const double dx = (fx * static_cast<double>(pt_cam.x)) * inv_z + cx;
          const double dy = (fy * static_cast<double>(pt_cam.y)) * inv_z + cy;

          const int px = static_cast<int>(std::round(dx));
          const int py = static_cast<int>(std::round(dy));
          if (px < 0 || py < 0 || px >= w || py >= h)
            continue;

          const auto zb = zbuffer.at<float>(py, px);
          const auto z = static_cast<float>(pt_cam.z);
          if (!std::isfinite(zb) || std::abs(z - zb) > 0.15f)
            continue;

          labels->points[i].label = labeledImage.at<int>(py, px);

          temp.at<cv::Vec3b>(py, px) = cv::Vec3b(
              cloud->points[i].b, cloud->points[i].g, cloud->points[i].r);
        }
      } // end assign labels

      ++(*observer);
      // cv::imwrite(tmpPath / fmt::format("temp_{}.png", id), temp);
    }
  }

  ReUseX::info("Projection completed in {:.3f}s", timer);

  return labels;
}

} // namespace ReUseX::vision
