// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ReUseX/types.hpp>
#include <ReUseX/vision/project.hpp>
#include <ReUseX/visualize/Visualizer.hpp>
#include <ReUseX/visualize/pcl.hpp>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>
#include <spdmon/spdmon.hpp>

#include <range/v3/view/iota.hpp>

#include <fmt/std.h>

#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/core/util3d.h>

#include <pcl/filters/frustum_culling.h>

#include <cmath>
#include <limits>
#include <opencv2/opencv.hpp>
#include <sqlite3.h>
#include <stdexcept>
#include <vector>

#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Transform.h>

#include <Eigen/Core>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <pcl/visualization/pcl_visualizer.h>

using namespace rtabmap;

namespace ReUseX::vision {

namespace {

inline rtabmap::CameraModel scaledCameraModel(const rtabmap::CameraModel &cm,
                                              const cv::Size &targetSize) {
  const double scaleX =
      static_cast<double>(targetSize.width) / cm.imageSize().width;
  const double scaleY =
      static_cast<double>(targetSize.height) / cm.imageSize().height;

  if (!cm.isValidForProjection()) {
    spdlog::warn(
        "CameraModel is not valid for projection, returning unscaled model.");
    return cm;
  }
  // has only effect on K and P
  cv::Mat K = cm.K_raw().clone();
  if (!K.empty()) {
    // Scale K
    K.at<double>(0, 0) *= scaleX; // fx
    K.at<double>(1, 1) *= scaleY; // fy
    K.at<double>(0, 2) *= scaleX; // cx
    K.at<double>(1, 2) *= scaleY; // cy
  }

  cv::Mat P = cm.P().clone();
  if (!P.empty()) {
    P.at<double>(0, 0) *= scaleX; // fx
    P.at<double>(1, 1) *= scaleY; // fy
    P.at<double>(0, 2) *= scaleX; // cx
    P.at<double>(1, 2) *= scaleY; // cy
    P.at<double>(0, 3) *= scaleX; // Tx
    P.at<double>(1, 3) *= scaleY; // Ty
  }

  return CameraModel(cm.name(), targetSize, K, cm.D_raw(), cm.R(), P,
                     cm.localTransform());
}

inline std::pair<double, double>
fovsFromCameraModel(const rtabmap::CameraModel &cm) {

  cv::Mat K = cm.K_raw(); // 3x3 intrinsic matrix
  double fx = K.at<double>(0, 0);
  double fy = K.at<double>(1, 1);
  double width = static_cast<double>(cm.imageSize().width);
  double height = static_cast<double>(cm.imageSize().height);

  double fov_x = 2.0 * std::atan(width / (2.0 * fx)) * 180.0 / CV_PI;
  double fov_y = 2.0 * std::atan(height / (2.0 * fy)) * 180.0 / CV_PI;

  return {fov_y, fov_x}; // (vertical, horizontal)
}

auto getLabeledImage(sqlite3_stmt *stmt, int id) -> cv::Mat {
  spdlog::trace("Fetching label image for id {} from database", id);

  cv::Mat labledImage;
  sqlite3_bind_int(stmt, 1, id);
  if (sqlite3_step(stmt) != SQLITE_ROW) {
    spdlog::error("No label image found for id {} in database", id);
    sqlite3_clear_bindings(stmt);
    sqlite3_reset(stmt);
    return labledImage;
  }

  spdlog::trace("Reading blob data for label image id {} from database", id);

  int idx = 0;
  const void *data = sqlite3_column_blob(stmt, idx);
  spdlog::trace("Blob data pointer: {}", fmt::ptr((void *)data));
  int datasize = sqlite3_column_bytes(stmt, idx++);

  spdlog::trace("Decoding label image from database");
  // imgMat = cv::imdecode(imgMat, cv::IMREAD_UNCHANGED);

  // The blob is just a stream of bytes (compressed PNG)
  std::vector<uchar> buffer((uchar *)data, (uchar *)data + datasize);

  // Decode the PNG into a cv::Mat
  labledImage = cv::imdecode(buffer, cv::IMREAD_UNCHANGED);

  if (labledImage.empty()) {
    spdlog::error("Failed to decode image {} from database", id);
    sqlite3_clear_bindings(stmt);
    sqlite3_reset(stmt);
    return labledImage;
  }

  spdlog::debug("Decoded image: {}x{}, channels={}", labledImage.cols,
                labledImage.rows, labledImage.channels());

  spdlog::trace("Convert label image to CV_32S");
  labledImage.convertTo(labledImage, CV_32S);
  labledImage -= 1; // Move 0 to -1 so that it can be stored as signed
                    // 32-bit integer

  // sqlite3_finalize(stmt);
  sqlite3_clear_bindings(stmt);
  sqlite3_reset(stmt);
  return labledImage;
}

inline void
add_camera(std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
           const rtabmap::CameraModel &cm, const rtabmap::Transform &pose) {
  // read current camera
  const double focal_x = cm.fx();
  const double focal_y = cm.fy();
  const double height = cm.imageSize().height;
  const double width = cm.imageSize().width;
  auto pose_af = (cm.localTransform().inverse() * pose).toEigen3f();

  // create a 5-point visual for each camera
  pcl::PointXYZ p1, p2, p3, p4, p5;
  p1.x = 0;
  p1.y = 0;
  p1.z = 0;
  double dist = 0.75;
  double minX, minY, maxX, maxY;
  maxX = dist * tan(std::atan(width / (2.0 * focal_x)));
  minX = -maxX;
  maxY = dist * tan(std::atan(height / (2.0 * focal_y)));
  minY = -maxY;
  p2.x = minX;
  p2.y = minY;
  p2.z = dist;
  p3.x = maxX;
  p3.y = minY;
  p3.z = dist;
  p4.x = maxX;
  p4.y = maxY;
  p4.z = dist;
  p5.x = minX;
  p5.y = maxY;
  p5.z = dist;
  p1 = pcl::transformPoint(p1, pose_af);
  p2 = pcl::transformPoint(p2, pose_af);
  p3 = pcl::transformPoint(p3, pose_af);
  p4 = pcl::transformPoint(p4, pose_af);
  p5 = pcl::transformPoint(p5, pose_af);

  viewer->addText3D("camera", p1, 0.1, 1.0, 1.0, 1.0, "camera");
  viewer->addLine(p1, p2, fmt::format("{}_line1", "camera"));
  viewer->addLine(p1, p3, fmt::format("{}_line2", "camera"));
  viewer->addLine(p1, p4, fmt::format("{}_line3", "camera"));
  viewer->addLine(p1, p5, fmt::format("{}_line4", "camera"));
  viewer->addLine(p2, p5, fmt::format("{}_line5", "camera"));
  viewer->addLine(p5, p4, fmt::format("{}_line6", "camera"));
  viewer->addLine(p4, p3, fmt::format("{}_line7", "camera"));
  viewer->addLine(p3, p2, fmt::format("{}_line8", "camera"));
}

cv::Mat colorizeLabels(const cv::Mat &labels) {
  cv::Mat coloredLabels(labels.size(), CV_8UC3);

#pragma omp parallel for shared(coloredLabels, labels)
  for (int i = 0; i < labels.rows; ++i) {
    for (int j = 0; j < labels.cols; ++j) {
      int label = labels.at<int>(i, j);
      auto color = pcl::GlasbeyLUT::at(label % pcl::GlasbeyLUT::size());
      coloredLabels.at<cv::Vec3b>(i, j) = cv::Vec3b(color.b, color.g, color.r);
    }
  }

  return coloredLabels;
}

cv::Mat normalizeDepthImage(const cv::Mat &depthImage) {
  cv::Mat normalizedDepth = depthImage.clone();
  cv::normalize(depthImage, normalizedDepth, 0.0, 1.0, cv::NORM_MINMAX);
  normalizedDepth = normalizedDepth * 255.0;
  normalizedDepth.convertTo(normalizedDepth, CV_8UC1);
  return normalizedDepth;
}

} // namespace

auto project(const std::filesystem::path &dbPath, CloudConstPtr cloud)
    -> CloudLPtr {
  spdlog::trace("calling project");

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

  spdlog::info("Intializing RTAB-Map ...");
  spdlog::stopwatch timer;
  ParametersMap params;
  Rtabmap rtabmap;
  spdlog::debug("Database path: {}", dbPath);
  rtabmap.init(params, dbPath.c_str());
  rtabmap.setWorkingDirectory("./");
  spdlog::debug("RTAB-Map initialized in {:.3f}s", timer);

  // Save 3D map
  spdlog::info("Loading Graph");
  timer.reset();

  std::map<int, Transform> poses;
  std::multimap<int, Link> links;
  std::map<int, Signature> nodes;
  rtabmap.getGraph(poses /*poses*/, links /*constraints*/,
                   true
                   /*optimized*/,
                   true /*global*/, &nodes /*signatures*/,
                   true
                   /*withImages*/,
                   true /*withScan*/, true /*withUserData*/,
                   true /*withGrid*/ /*withWords*/
                   /*withGlobalDescriptors*/);
  spdlog::debug("Graph loaded in {:.3f}s", timer);
  timer.reset();

  // Open database connection for reading segmentation results
  sqlite3 *db_ = nullptr;
  if (sqlite3_open(dbPath.string().c_str(), &db_) != SQLITE_OK) {
    spdlog::error("Cannot open database: {}", sqlite3_errmsg(db_));
    sqlite3_close(db_);
    throw std::runtime_error("Cannot open database");
  }

  // Ceck if Segmentation table exists
  sqlite3_stmt *stmt;
  if (sqlite3_prepare_v2(db_,
                         "SELECT name FROM sqlite_master WHERE type='table' "
                         "AND name='Segmentation';",
                         -1, &stmt, nullptr) != SQLITE_OK) {
    spdlog::error("No Segmentation table found in database: {}",
                  sqlite3_errmsg(db_));
    sqlite3_close(db_);
    return labels;
  }

  sqlite3_exec(db_, "BEGIN TRANSACTION;", nullptr, nullptr, nullptr);
  if (sqlite3_prepare_v2(db_,
                         "SELECT label_image FROM Segmentation WHERE id=?;", -1,
                         &stmt, nullptr) != SQLITE_OK) {
    spdlog::error("Failed to prepare statement for Segmentation table: {}",
                  sqlite3_errmsg(db_));
    sqlite3_close(db_);
    return labels;
  }

  // INFO: Create vector of poses for openMP parallel processing
  std::vector<std::pair<int, Transform>> poseVector;
  for (std::map<int, Transform>::iterator iter = poses.begin();
       iter != poses.end(); ++iter) {
    poseVector.push_back(*iter);
  }

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
    auto logger = std::make_shared<spdmon::LoggerProgress>("Projecting labels",
                                                           poseVector.size());
    for (size_t i = 0; i < poseVector.size(); ++i) {
      spdlog::trace("Processing node {}/{}", i + 1, poseVector.size());

      auto [id, pose] = poseVector[i];

      // --- Load sensor data for this node
      Signature node = nodes.find(id)->second;
      SensorData data = node.sensorData();
      data.uncompressData();

      std::filesystem::path tmpPath = "./debth_debug";
      if (!std::filesystem::exists(tmpPath))
        std::filesystem::create_directory(tmpPath);

      // cv::imwrite(tmpPath / fmt::format("img_node_{}.png", id),
      //             data.imageRaw());

      // cv::imwrite(tmpPath / fmt::format("depth_node_{}.png", id),
      //             /*normalizeDepthImage(*/
      //             data.depthOrRightRaw());

      // --- Fetch label image
      cv::Mat labeledImage = getLabeledImage(stmt, id); // CV_32S
      // cv::imwrite(tmpPath / fmt::format("labeled_img_{}.png", id),
      //             colorizeLabels(labeledImage));
      if (labeledImage.empty()) {
        spdlog::warn("No label image for node {}, skipping projection", id);
        ++(*logger);
        continue;
      }

      rtabmap::CameraModel cm = data.cameraModels().back();

      // cm = scaledCameraModel(cm, labeledImage.size());
      cm = scaledCameraModel(cm, cm.imageSize() / 16);

      auto t_local = pose.inverse();
      auto t_cam = cm.localTransform();

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

      // add_camera(viewer, cm, pose);

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

      spdlog::trace("Filling Z-buffer holes for node {}", id);
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

      cm = scaledCameraModel(cm, labeledImage.size());
      {
        spdlog::trace("Assigning labels for node {}", id);
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
              pt_cam.z > 7.0f) // TODO: parameterize max depth
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

      ++(*logger);
      // cv::imwrite(tmpPath / fmt::format("temp_{}.png", id), temp);
    }
  }

  spdlog::info("Projection completed in {:.3f}s", timer);
  sqlite3_finalize(stmt);

  spdlog::trace("closing database");
  sqlite3_exec(db_, "END TRANSACTION;", nullptr, nullptr, nullptr);
  sqlite3_close(db_);
  spdlog::trace("database closed");

  return labels;
}

} // namespace ReUseX::vision
