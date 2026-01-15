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

struct CameraIntrinsics {
  double fx{0.0}, fy{0.0}, cx{0.0}, cy{0.0};
  int width{0}, height{0};
};

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

  // sqlite3_finalize(stmt);
  sqlite3_clear_bindings(stmt);
  sqlite3_reset(stmt);
  return labledImage;
}

Eigen::Matrix3d getintrinsicmatrix(const rtabmap::CameraModel &cm) {
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  cv::cv2eigen(cm.K(), K);
  return K;
}

inline cv::Mat buildZBufferFromCloud(const CloudConstPtr &cloud,
                                     const pcl::Indices &indices,
                                     const rtabmap::CameraModel &cm,
                                     const rtabmap::Transform &pose) {

  const double INF = std::numeric_limits<double>::infinity();
  cv::Mat zbuffer(cm.imageSize(), CV_64F, cv::Scalar(INF));

  // Eigen::Matrix4d rtab_T_pcl = Eigen::Matrix4d::Identity();
  // rtab_T_pcl.topLeftCorner<4, 3>() << 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0;
  // const Eigen::Matrix4d pcl_T_rtab = rtab_T_pcl.inverse();

  // const Eigen::Matrix4d world_T_base = pose.inverse().toEigen4d();
  // const Eigen::Matrix4d base_T_cam = cm.localTransform().toEigen4d();

  // const Eigen::Matrix4d world_T_cam = plc_T_rtab * world_T_base * base_T_cam;

  Eigen::Matrix4d map_T_base = pose.toEigen4d(); // map/world → base
  Eigen::Matrix4d base_T_cam = cm.localTransform().toEigen4d(); // base → camera
  Eigen::Matrix4d rtab_T_pcl;
  rtab_T_pcl << 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

  Eigen::Matrix4d map_T_pclCamera = map_T_base * base_T_cam * rtab_T_pcl;

  const Eigen::Matrix3d K = getintrinsicmatrix(cm);

  // const Eigen::Matrix4d pcl_T_cam = base_T_cam * pcl_T_rtab;

  // auto world_T_cam = cm.localTransform().toEigen3f();
  // base_T_cam(3, 3) = 1.0; // ensure bottom-right is 1.0

  for (const auto &idx : indices) {

    // Point in world frame
    Eigen::Vector4d p;
    p.head<3>() = cloud->points[idx].getVector3fMap().cast<double>();
    p[3] = 1.0;

    // Transform to PCL camera frame
    Eigen::Vector4d pc = map_T_pclCamera.inverse() * p;

    // Skip points behind camera
    if (pc[2] <= 0.0)
      continue;

    // Project to image
    Eigen::Vector3d uv = K * pc.head<3>();
    uv /= uv[2];

    int px = static_cast<int>(std::round(uv[0]));
    int py = static_cast<int>(std::round(uv[1]));

    if (px < 0 || py < 0 || px >= cm.imageSize().width ||
        py >= cm.imageSize().height)
      continue;

    double &cur = zbuffer.at<double>(py, px);
    if (pc[2] < cur)
      cur = pc[2]; // keep closest

    // // Get point in world coordinates
    // Eigen::Vector4d p = Eigen::Vector4d::Identity();
    // p.head<3>() = cloud->points[idx].getVector3fMap().cast<double>();
    // p[3] = 1.0;

    // // Apply world pose
    // p = world_T_cam * p;

    // // Project to image plane
    // p.head<3>() = K * p.head<3>();
    // const double dist = p[2];
    // p /= p[2]; // normalize by depth

    // const int px = static_cast<int>(std::round(p[0]));
    // const int py = static_cast<int>(std::round(p[1]));

    // // Check image bounds
    // if (px < 0 || py < 0 || px >= cm.imageSize().width ||
    //     py >= cm.imageSize().height) {
    //   continue; // out of image bounds
    // }

    // const double cur = zbuffer.at<double>(py, px);

    // if (dist < cur)
    //   zbuffer.at<double>(py, px) = dist; // keep closest point
  };

  return zbuffer;
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

      // --- Fetch label image
      cv::Mat labeledImage = getLabeledImage(stmt, id); // CV_32S

      rtabmap::CameraModel cm = data.cameraModels().front();

      // cm = scaledCameraModel(cm, labeledImage.size());
      cm = scaledCameraModel(cm, cm.imageSize() / 16);

      auto cam_T_world = (pose * rtabmapCam_T_pclCam).inverse();
      auto world_T_cam = pose.inverse() * cm.localTransform().inverse();

      // Frustum culling setup
      pcl::FrustumCulling<Cloud::PointType> fc;
      fc.setInputCloud(cloud);
      fc.setCameraPose(
          (pose * cm.localTransform().inverse() * rtabmapCam_T_pclCam)
              .toEigen4f());
      fc.setNearPlaneDistance(0.05f);
      fc.setFarPlaneDistance(50.0f); // wide enough to include valid points
      auto [fovYdeg, fovXdeg] = fovsFromCameraModel(cm);
      fc.setVerticalFOV(static_cast<float>(fovYdeg));
      fc.setHorizontalFOV(static_cast<float>(fovXdeg));

      // add_camera(viewer, cm, pose);

      pcl::Indices indices;
      fc.filter(indices);

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
          cloud_lf, pose.inverse()); // Mover cloud to link frame

      // --- Build Z-buffer from frustum points and fill holes
      cv::Mat zbuffer = rtabmap::util3d::projectCloudToCamera(
          cm.imageSize() /*labeledImage.size()*/, cm.K(), cloud_lf,
          cm.localTransform());
      // cv::Mat zbuffer = buildZBufferFromCloud(cloud, indices, cm, pose);
      spdlog::trace("Filling Z-buffer holes for node {}", id);
      // rtabmap::util2d::fillDepthHoles(zbuffer, 1, 0.50f);
      // rtabmap::util2d::fillRegisteredDepthHoles(zbuffer, true, false, false);

      rtabmap::util3d::fillProjectedCloudHoles(zbuffer, true, true);

      cv::resize(zbuffer, zbuffer, labeledImage.size());

      cv::Mat zbufferDisplay = zbuffer.clone();
      cv::normalize(zbufferDisplay, zbufferDisplay, 0.0, 1.0, cv::NORM_MINMAX);
      // cv::imshow("Z-Buffer", zbufferDisplay);

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

      // std::filesystem::path tmpPath = "./debth_debug";
      // if (!std::filesystem::exists(tmpPath)) {
      //   std::filesystem::create_directory(tmpPath);
      // }
      // cv::imwrite(tmpPath / fmt::format("zbuffer_node_{}.png", id),
      //             zbufferDisplay * 255.0);

      cm = scaledCameraModel(cm, labeledImage.size());
      {
        spdlog::trace("Assigning labels for node {}", id);
        // Assign labels to the points in the point cloud
        auto t = cm.localTransform().inverse();
        auto fx = cm.fx();
        auto fy = cm.fy();
        auto cx = cm.cx();
        auto cy = cm.cy();
        auto h = cm.imageHeight();
        auto w = cm.imageWidth();
#pragma omp parallel for shared(labels, cloud, indices, labeledImage, zbuffer) \
    firstprivate(t, fx, fy, cx, cy, w, h)
        for (int j = 0; j < static_cast<int>(indices.size()); ++j) {
          auto i = indices[j];

          const auto &pt = cloud_lf->points[i];

          auto pt_cam = rtabmap::util3d::transformPoint(pt, t);
          if (pt_cam.z <= 0.0f)
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
        }
      }

      ++(*logger);
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
