// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ReUseX/io/RTABMapDatabase.hpp>
#include <ReUseX/io/rtabmap.hpp>
#include <ReUseX/utils/fmt_formatter.hpp>

#include <spdmon/spdmon.hpp>

#include <sqlite3.h>

#include <fmt/format.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <pcl/common/colors.h>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/optimizer/OptimizerG2O.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#ifndef NDEBUG
#include <opencv2/highgui.hpp>
#include <pcl/common/colors.h>
namespace {
const cv::Mat &getGlasbeyLUT() {
  static cv::Mat lut = [] {
    cv::Mat m(1, 256, CV_8UC3);
    for (int i = 1; i < 255; ++i) {
      const auto &c = pcl::GlasbeyLUT::at(i);
      m.at<cv::Vec3b>(0, i) =
          cv::Vec3b(static_cast<uchar>(c.b), static_cast<uchar>(c.g),
                    static_cast<uchar>(c.r));
    }
    m.at<cv::Vec3b>(0, 255) = cv::Vec3b(0, 0, 0); // Background color (black)
    return m;
  }();
  return lut;
}

auto lut = getGlasbeyLUT();
} // namespace
#endif

using namespace rtabmap;

inline void
merge_clouds(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr lhs,
             const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rhs) {
  *lhs += *rhs;
}

#pragma omp declare reduction(                                                 \
        + : pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr : merge_clouds(       \
                omp_out, omp_in))                                              \
    initializer(omp_priv = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(       \
                        new pcl::PointCloud<pcl::PointXYZRGBNormal>()))

inline void merge_clouds(pcl::PointCloud<pcl::Label>::Ptr lhs,
                         const pcl::PointCloud<pcl::Label>::Ptr rhs) {
  *lhs += *rhs;
}

#pragma omp declare reduction(                                                 \
        + : pcl::PointCloud<pcl::Label>::Ptr : merge_clouds(omp_out, omp_in))  \
    initializer(omp_priv = pcl::PointCloud<pcl::Label>::Ptr(                   \
                        new pcl::PointCloud<pcl::Label>()))

namespace ReUseX::io {
auto import_rtabmap_database(const std::filesystem::path &database_path,
                             float resolution, float min_distance,
                             float max_distance, float sampling_factor)
    -> std::tuple<CloudPtr, CloudNPtr, CloudLPtr> {

  // TODO: Add path check for onnx model and package it with the application
  spdlog::info("Intializing RTAB-Map ...");
  spdlog::stopwatch timer;
  ParametersMap params;
  Rtabmap rtabmap;
  spdlog::debug("Database path: {}", database_path);
  rtabmap.init(params, database_path.c_str());
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

  spdlog::trace("Assembling point clouds from signatures");
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);

  // Open database connection for reading segmentation results
  sqlite3 *db_ = nullptr;
  if (sqlite3_open(database_path.string().c_str(), &db_) != SQLITE_OK) {
    spdlog::error("Cannot open database: {}", sqlite3_errmsg(db_));
    sqlite3_close(db_);
    throw std::runtime_error("Cannot open database");
  }

  // Ceck if Segmentation table exists
  sqlite3_stmt *stmt;
  bool skipInference = false;
  if (sqlite3_prepare_v2(db_,
                         "SELECT name FROM sqlite_master WHERE type='table' "
                         "AND name='Segmentation';",
                         -1, &stmt, nullptr) != SQLITE_OK) {
    spdlog::warn("No Segmentation table found in database: {}",
                 sqlite3_errmsg(db_));
    sqlite3_close(db_);
    skipInference = true;
  }

  if (!skipInference) {
    sqlite3_exec(db_, "BEGIN TRANSACTION;", nullptr, nullptr, nullptr);
    if (sqlite3_prepare_v2(db_,
                           "SELECT label_image FROM Segmentation WHERE id=?;",
                           -1, &stmt, nullptr) != SQLITE_OK) {
      spdlog::error("Failed to prepare statement for Segmentation table: {}",
                    sqlite3_errmsg(db_));
      sqlite3_close(db_);
      skipInference = true;
    }
  }

  { // scope logger variable

    // INFO: Create vector of poses for openMP parallel processing
    std::vector<std::pair<int, Transform>> poseVector;
    for (std::map<int, Transform>::iterator iter = poses.begin();
         iter != poses.end(); ++iter) {
      poseVector.push_back(*iter);
    }

    spdlog::trace("Number of poses: {}", poseVector.size());
    auto logger = spdmon::LoggerProgress("Assembling cloud", poseVector.size());

#ifdef NDEBUG
    // INFO: Create and OpenCV window for visualizing the results during
    // development
    cv::namedWindow("Images", cv::WINDOW_AUTOSIZE);
#endif

#ifndef NDEBUG
#pragma omp parallel for reduction(+ : cloud, labels)                          \
    shared(poses, nodes, logger)
#endif
    for (int i = 0; i < (int)poseVector.size(); ++i) {
      auto [id, pose] = poseVector[i];

      Signature node = nodes.find(id)->second;
      SensorData data = node.sensorData();
      data.uncompressData();

      spdlog::trace("Creating point cloud from sensor data");
      pcl::IndicesPtr validIndices(new pcl::Indices);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp =
          util3d::cloudRGBFromSensorData(data, sampling_factor, max_distance,
                                         min_distance, validIndices.get());
      spdlog::debug("Point cloud size: {}, valid: {}, size: {}x{}", tmp->size(),
                    validIndices->size(), tmp->width, tmp->height);

      pcl::PointCloud<pcl::Label>::Ptr labledCloud(
          new pcl::PointCloud<pcl::Label>);
      labledCloud->resize(tmp->size());

      // INFO: Initialize the labels to -1
      for (size_t i = 0; i < labledCloud->size(); ++i)
        labledCloud->points[i].label = -1;

      cv::Size cloud_size(tmp->width, tmp->height);

      cv::Mat image = data.imageRaw();
      cv::rotate(image, image, cv::ROTATE_90_CLOCKWISE);

      spdlog::trace("Set up labledImage");
      cv::Mat labledImage(image.size(), CV_32S, cv::Scalar(-1));

      if (!skipInference) {
        spdlog::trace("Querying label image for node_id {}", id);
        int idx = 0;
        sqlite3_bind_int(stmt, 1, id);
        if (sqlite3_step(stmt) == SQLITE_ROW) {
          const void *data = sqlite3_column_blob(stmt, idx);
          int datasize = sqlite3_column_bytes(stmt, idx++);

          // The blob is just a stream of bytes (compressed PNG)
          std::vector<uchar> buffer((uchar *)data, (uchar *)data + datasize);

          // Decode the PNG into a cv::Mat
          cv::Mat img = cv::imdecode(buffer, cv::IMREAD_UNCHANGED);
          if (img.empty()) {
            spdlog::error("Failed to decode image from database");
          } else {
            spdlog::debug("Decoded image: {}x{}, channels={}, type={}",
                          img.cols, img.rows, img.channels(), img.type());

            spdlog::trace("Convert label image to CV_32S");
            img.convertTo(labledImage, CV_32S);
            labledImage -= 1; // Move 0 to -1 so that it can be stored as signed
                              // 32-bit integer
          }
        }
        sqlite3_reset(stmt);
      }

      spdlog::trace("Resizing label image to cloud size");
      cv::resize(labledImage, labledImage, cloud_size, 0, 0, cv::INTER_NEAREST);

#ifndef NDEBUG
      cv::Mat debugImage(cloud_size, CV_8UC3);
      for (size_t i = 0; i < labledCloud->size(); ++i) {
        const auto rgb = tmp->at(i).getRGBVector3i();
        const uint8_t r = static_cast<uint8_t>(rgb[0]);
        const uint8_t g = static_cast<uint8_t>(rgb[1]);
        const uint8_t b = static_cast<uint8_t>(rgb[2]);
        debugImage.at<cv::Vec3b>(i) = cv::Vec3b(b, g, r);
      }

      // DEBUG: Show the images
      cv::Mat temp = labledImage.clone();
      temp = temp & 255;
      temp.convertTo(temp, CV_8U);
      // cv::normalize(temp, temp, 0, 255, cv::NORM_MINMAX, CV_8U);
      cv::cvtColor(temp, temp, cv::COLOR_GRAY2BGR);
      cv::LUT(temp, lut, temp);

      cv::addWeighted(debugImage, 0.5, temp, 0.5, 0, temp);

      cv::resize(temp, temp, cv::Size(), 4.0, 4.0, cv::INTER_NEAREST);

      cv::imshow("Annotation", temp);
      cv::waitKey(1);
#endif

      spdlog::trace("Set the values of the labled cloud");
      for (size_t i = 0; i < labledCloud->size(); ++i)
        labledCloud->points[i].label = labledImage.at<int>(i);

      pcl::ExtractIndices<pcl::Label> extract;
      extract.setInputCloud(labledCloud);
      extract.setIndices(validIndices);
      extract.setNegative(false);
      extract.filter(*labledCloud);

      spdlog::trace("Filterout NaN points");
      pcl::IndicesPtr nonNaNindices(new pcl::Indices);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpNoNaN(
          new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::removeNaNFromPointCloud(*tmp, *tmpNoNaN, *nonNaNindices);
      std::swap(tmp, tmpNoNaN);

      extract.setInputCloud(labledCloud);
      extract.setIndices(nonNaNindices);
      extract.setNegative(false);
      extract.filter(*labledCloud);

      if (!tmp->empty()) {

        spdlog::trace("Transforming point cloud to pose");
        tmp = util3d::transformPointCloud(tmp, pose);

        spdlog::trace("Setting sensor origin for normal estimation");
        float x, y, z;
        pose.getTranslation(x, y, z);
        tmp->sensor_origin_ = Eigen::Vector4f(x, y, z, 1.0f);

        spdlog::trace("Estimating normals for point cloud");
        pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
        ne.setInputCloud(tmp);
        ne.setRadiusSearch(0.1);
        pcl::PointCloud<pcl::Normal> normalsTmp;
        ne.compute(normalsTmp);

        spdlog::trace("Merging normals with point cloud");
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tmpCloud(
            new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        tmpCloud->resize(tmp->size());
        for (size_t i = 0; i < tmp->size(); ++i) {
          tmpCloud->at(i).x = tmp->at(i).x;
          tmpCloud->at(i).y = tmp->at(i).y;
          tmpCloud->at(i).z = tmp->at(i).z;
          tmpCloud->at(i).r = tmp->at(i).r;
          tmpCloud->at(i).g = tmp->at(i).g;
          tmpCloud->at(i).b = tmp->at(i).b;
          tmpCloud->at(i).normal_x = normalsTmp.at(i).normal_x;
          tmpCloud->at(i).normal_y = normalsTmp.at(i).normal_y;
          tmpCloud->at(i).normal_z = normalsTmp.at(i).normal_z;
        }
        // INFO: For some reason the following methods fail at voxwlization
        // pcl::concatenateFields(*tmp, normalsTmp, *tmpCloud);
        // pcl::copyPointCloud(*tmp, *tmpCloud);
        // pcl::copyPointCloud(normalsTmp, *tmpCloud);

        // spdlog::debug(
        //     "Point 0: XYZ({}, {}, {}), RGB({}, {}, {}), Normal({}, {},
        //     {})", tmpCloud->at(0).x, tmpCloud->at(0).y, tmpCloud->at(0).z,
        //     tmpCloud->at(0).r, tmpCloud->at(0).g, tmpCloud->at(0).b,
        //     tmpCloud->at(0).normal_x, tmpCloud->at(0).normal_y,
        //     tmpCloud->at(0).normal_z);

        spdlog::trace("Appending frame to the cloud");
        *cloud += *tmpCloud;
        *labels += *labledCloud;
      }

      ++logger;
    }
  }

  sqlite3_finalize(stmt);

#ifndef NDEBUG
  // INFO: Wait for user input before closing the visualization window
  cv::waitKey(5 * 1000); // Wait for 5 seconds
  cv::destroyWindow("Annotation");
  // cv::destroyAllWindows();
#endif

  // TODO: Add assertions
  if (!cloud->size()) {
    spdlog::error("Error: The cloud is empty.");
    rtabmap.close(false);
    return std::make_tuple(nullptr, nullptr, nullptr);
  }

  // INFO: Downsample the point cloud
  spdlog::info(
      "Voxel grid filtering of the assembled cloud (voxel={}, {} points)",
      resolution, (int)cloud->size());
  auto downsampledCloud = util3d::voxelize(cloud, resolution);
  spdlog::debug("Points after voxel grid filtering: {}",
                downsampledCloud->size());

  pcl::Indices filter_indices;
  pcl::removeNaNNormalsFromPointCloud(*downsampledCloud, *downsampledCloud,
                                      filter_indices);

  CloudPtr out_cloud(new Cloud);
  pcl::copyPointCloud(*downsampledCloud, *out_cloud);

  CloudNPtr out_normals(new CloudN);
  pcl::copyPointCloud(*downsampledCloud, *out_normals);

  spdlog::trace("Creating labels");
  // BUG: There is a segfault here
  pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;
  kdtree.setInputCloud(cloud); // Original cloud before downsampling

  CloudLPtr out_labels(new CloudL);
  out_labels->resize(downsampledCloud->size());

  auto const half_resolution = resolution / 2.0f;
  auto mostCommonLabel = [](std::unordered_map<int, int> &labelCount) {
    // Find the most common label
    int maxCount = 0;
    int mostCommonLabel = -1;
    for (const auto &pair : labelCount) {
      if (pair.second > maxCount) {
        maxCount = pair.second;
        mostCommonLabel = pair.first;
      }
    }
    return mostCommonLabel;
  };
  for (size_t i = 0; i < downsampledCloud->size(); ++i) {
    auto &point = downsampledCloud->at(i);

    std::vector<int> indices;
    std::vector<float> distances; // Unused
    kdtree.radiusSearch(point, half_resolution, indices, distances);

    std::unordered_map<int, int> labelCount;
    for (const auto &idx : indices) {
      int label = labels->points[idx].label;
      labelCount[label]++;
    }
    out_labels->points[i].label = mostCommonLabel(labelCount);
  }
  // TODO:Condider only writing files for specified output paths and removing
  // the default values

  //// INFO: Saving the trajectory
  // spdlog::info("Saving {} ...", trajectory_path_out);
  // if (poses.size() &&
  //     graph::exportPoses(trajectory_path_out, 0, poses, links)) {
  //   // const std::string & filePath,
  //   // int format, // 0=Raw, 1=RGBD-SLAM motion capture (10=without change
  //   of
  //   // coordinate frame, 11=10+ID), 2=KITTI, 3=TORO, 4=g2o
  //   // const std::map<int, Transform> & poses
  //   // const std::multimap<int, Link> & constraints required for formats 3
  //   and
  //   // 4 const std::map<int, double> & stamps required for format 1, 10 and
  //   11
  //   // const ParametersMap & parameters) optional for formats 3 and 4

  //  // Raw = KITTI Fromat
  //  // // Format: r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
  //} else {
  //  spdlog::error("Saving {} ... failed!", trajectory_path_out);
  //  rtabmap.close(false);
  //  return std::make_tuple(nullptr, nullptr, nullptr);
  //}

  sqlite3_exec(db_, "COMMIT;", nullptr, nullptr, nullptr);
  sqlite3_close(db_);
  rtabmap.close();

  // Filter outliers and noise from the point cloud
  spdlog::info("Filtering outliers from point cloud ({} points)", out_cloud->size());
  timer.reset();

  // Step 1: Statistical Outlier Removal
  // Removes points that are statistical outliers based on mean distance
  spdlog::trace("Applying statistical outlier removal");
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(out_cloud);
  sor.setMeanK(50);              // Number of neighbors to analyze
  sor.setStddevMulThresh(1.0);   // Standard deviation multiplier threshold

  pcl::IndicesPtr stat_inliers(new pcl::Indices);
  sor.filter(*stat_inliers);
  spdlog::debug("Statistical outlier removal kept {}/{} points",
                stat_inliers->size(), out_cloud->size());

  // Step 2: Radius Outlier Removal
  // Removes points that have fewer neighbors within a specified radius
  spdlog::trace("Applying radius outlier removal");
  pcl::RadiusOutlierRemoval<PointT> ror;
  ror.setInputCloud(out_cloud);
  ror.setIndices(stat_inliers);  // Apply on top of statistical filtering
  ror.setRadiusSearch(resolution * 2.0);  // Search radius based on voxel size
  ror.setMinNeighborsInRadius(5);         // Minimum neighbors required

  pcl::IndicesPtr filtered_indices(new pcl::Indices);
  ror.filter(*filtered_indices);
  spdlog::debug("Radius outlier removal kept {}/{} points",
                filtered_indices->size(), stat_inliers->size());

  // Step 3: Apply the same filtering to all three clouds (points, normals, labels)
  // This ensures they remain synchronized
  spdlog::trace("Extracting filtered points, normals, and labels");

  CloudPtr filtered_cloud(new Cloud);
  pcl::ExtractIndices<PointT> extract_points;
  extract_points.setInputCloud(out_cloud);
  extract_points.setIndices(filtered_indices);
  extract_points.setNegative(false);
  extract_points.filter(*filtered_cloud);

  CloudNPtr filtered_normals(new CloudN);
  pcl::ExtractIndices<NormalT> extract_normals;
  extract_normals.setInputCloud(out_normals);
  extract_normals.setIndices(filtered_indices);
  extract_normals.setNegative(false);
  extract_normals.filter(*filtered_normals);

  CloudLPtr filtered_labels(new CloudL);
  pcl::ExtractIndices<LabelT> extract_labels;
  extract_labels.setInputCloud(out_labels);
  extract_labels.setIndices(filtered_indices);
  extract_labels.setNegative(false);
  extract_labels.filter(*filtered_labels);

  // Verify all clouds have the same size
  if (filtered_cloud->size() != filtered_normals->size() ||
      filtered_cloud->size() != filtered_labels->size()) {
    spdlog::error("Filtered clouds have mismatched sizes: points={}, normals={}, labels={}",
                  filtered_cloud->size(), filtered_normals->size(), filtered_labels->size());
    return std::make_tuple(nullptr, nullptr, nullptr);
  }

  spdlog::info("Filtering complete in {:.3f}s: {}/{} points retained ({:.1f}%)",
               timer, filtered_cloud->size(), out_cloud->size(),
               100.0 * filtered_cloud->size() / out_cloud->size());

  return std::make_tuple(filtered_cloud, filtered_normals, filtered_labels);
}

bool checkRTABMapDatabase(std::filesystem::path const &dbPath) {
  spdlog::trace("Checking RTABMap database: {}", dbPath);

  try {
    RTABMapDatabase db(dbPath, /*readOnly=*/true);
    db.validateSchema();
    return db.isOpen();
  } catch (const std::exception &e) {
    spdlog::error("Database check failed: {}", e.what());
    return false;
  }
}

bool initRTABMapDatabase(std::filesystem::path const &dbPath) {
  spdlog::trace("Initializing RTABMap database: {}", dbPath);

  try {
    // Constructor creates Segmentation table if needed
    RTABMapDatabase db(dbPath, /*readOnly=*/false);
    spdlog::info("Database initialized successfully");
    return true;
  } catch (const std::exception &e) {
    spdlog::error("Database initialization failed: {}", e.what());
    return false;
  }
}

bool writeLabelsToRTABMapDatabase(std::filesystem::path const &dbPath,
                                  cv::Mat const &labels,
                                  std::optional<size_t> id) {
  spdlog::trace("Writing labels to RTABMap database: {}", dbPath);

  if (!id.has_value()) {
    spdlog::error("Node ID is required to write labels");
    return false;
  }

  try {
    RTABMapDatabase db(dbPath, /*readOnly=*/false);
    db.saveLabels(static_cast<int>(id.value()), labels, /*autoRotate=*/true);
    spdlog::debug("Labels written successfully for node {}", id.value());
    return true;
  } catch (const std::exception &e) {
    spdlog::error("Failed to write labels: {}", e.what());
    return false;
  }
}

cv::Mat readLabelsFromRTABMapDatabase(std::filesystem::path const &dbPath,
                                      size_t id) {
  spdlog::trace("Reading labels from RTABMap database: {} for node {}", dbPath,
                id);

  try {
    RTABMapDatabase db(dbPath, /*readOnly=*/true);
    cv::Mat labels = db.getLabels(static_cast<int>(id));

    if (labels.empty()) {
      spdlog::warn("No labels found for node {}", id);
    } else {
      spdlog::debug("Labels read successfully for node {}: {}x{}", id,
                    labels.cols, labels.rows);
    }

    return labels;
  } catch (const std::exception &e) {
    spdlog::error("Failed to read labels: {}", e.what());
    return cv::Mat();
  }
}

} // namespace ReUseX::io
