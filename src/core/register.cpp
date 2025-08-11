#include "register.hh"
#include "fmt_formatter.hh"
#include "parse_input_files.hh"
#include "report.hh"
#include "spdmon.hh"
#include "types/Filters.hh"
#include "types/Geometry/PointCloud.hh"
#include "visualizer/visualizer.hh"

#include <pcl/common/colors.h>
#include <pcl/common/transforms.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/features/shot.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/auto_io.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/elch.h>
#include <pcl/registration/lum.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>

#include <spdlog/common.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <g2o/core/block_solver.h>
#include <g2o/core/eigen_types.h>
#include <g2o/core/hyper_graph.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/slam3d/isometry3d_mappings.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <Eigen/src/Core/util/Memory.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>

// RTAB-Map includes
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <rtabmap/core/CameraRGB.h>
#include <rtabmap/core/CameraRGBD.h>
#include <rtabmap/core/CameraStereo.h>
#include <rtabmap/core/CameraThread.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryThread.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/RtabmapThread.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UFile.h>
#include <stdio.h>

#ifdef RTABMAP_PYTHON
#include "rtabmap/core/PythonInterface.h"
#endif

#include <omp.h>

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#define VISUALIZE 1
#define TEMP_EXTENSION "pcd"

namespace fs = std::filesystem;
using namespace ReUseX;
using namespace rtabmap;

fs::path
ReUseX::write_graph(fs::path path, Dataset &dataset,
                    std::vector<size_t> &indices, const size_t groupSize,
                    const std::string solverName, const std::string kernelName,
                    const int maxIterations, const double maxCorrespondence,
                    const double deltaValue,
                    const Eigen::Matrix<double, 6, 6> information_matrix) {

  // Spdlog::info("Entering Write Graph Function");
  // Spdlog::debug("Indecies: first: {} last: {}, count: {}, step: ~{}",
  //               indices.front(), indices.back(), indices.size(),
  //               ((indices.back() - indices.front()) / indices.size()) + 1);

  // Here is the pipeline that we will use:
  // CameraOpenni -> "CameraEvent" -> OdometryThread -> "OdometryEvent" ->
  // RtabmapThread -> "RtabmapEvent"

  // Create the OpenNI camera, it will send a CameraEvent at the rate specified.
  // Set transform to camera so z is up, y is left and x going forward

  // Camera *camera = 0;
  // camera = new rtabmap::CameraVideo(dataset.get_rgb_path());

  // cv::Mat K, D, R, P;
  // K = dataset.intrinsic_matrix().toMat();
  // const Transform &localTransform = opticalRotation();
  // CameraModel cameraModel("iPad", dataset.color_size(), K, D, R, P,
  //                         localTransform);

  // K is the camera intrinsic 3x3 CV_64FC1
  // D is the distortion coefficients 1x5 CV_64FC1
  // R is the rectification matrix 3x3 CV_64FC1 (computed from stereo or
  // Identity) P is the projection matrix 3x4 CV_64FC1 (computed from stereo or
  // equal to [K [0 0 1]'])

  // camera->setStartIndex(indices.front());
  // camera->setMaxFrames(indices.back());

  // if (!camera->init()) {
  //   spdlog::error("Camera init failed, using path \"{}\"",
  //                 dataset.get_rgb_path());
  //   exit(1);
  // }

  // SensorCaptureThread cameraThread(camera);

  // GUI stuff, there the handler will receive RtabmapEvent and construct the
  // map We give it the camera so the GUI can pause/resume the camera
  int argc = 0;

  // Create an odometry thread to process camera events, it will send
  // OdometryEvent.
  // OdometryThread odomThread(Odometry::create());

  ParametersMap params;
  // param.insert(ParametersPair(Parameters::kRGBDCreateOccupancyGrid(),
  // "true"));
  // // uncomment to create local occupancy grids

  // Create RTAB-Map to process OdometryEvent
  Rtabmap *rtabmap = new Rtabmap;
  rtabmap->init(params, "/home/mephisto/Documents/RTAB-Map/office.db");
  rtabmap->setWorkingDirectory("./");
  // rtabmap->setDatabasePath("/home/mephisto/Documents/RTAB-Map/map1_1Hz.db");

  // Set the time threshold
  // Time threshold : 700 ms, 0 ms means no limit
  // rtabmap->setTimeThreshold(0.0f);

  // RtabmapThread rtabmapThread(rtabmap); // ownership is transfered

  // Setup handlers
  // odomThread.registerToEventsManager();
  // rtabmapThread.registerToEventsManager();

  // The RTAB-Map is subscribed by default to CameraEvent, but we want
  // RTAB-Map to process OdometryEvent instead, ignoring the CameraEvent.
  // We can do that by creating a "pipe" between the camera and odometry, then
  // only the odometry will receive CameraEvent from that camera. RTAB-Map is
  // also subscribed to OdometryEvent by default, so no need to create a pipe
  // between odometry and RTAB-Map.
  // UEventsManager::createPipe(&cameraThread, &odomThread, "CameraEvent");

  // Let's start the threads
  // rtabmapThread.start();
  // odomThread.start();
  // cameraThread.start();

  // remove handlers
  // rtabmapThread.unregisterFromEventsManager();
  // odomThread.unregisterFromEventsManager();

  // Kill all threads
  // cameraThread.kill();
  // odomThread.join(true);
  // rtabmapThread.join(true);
  // rtabmapThread.kill();

  // Save 3D map
  spdlog::info("Saving rtabmap_cloud.pcd...");
  std::map<int, Signature> nodes;
  std::map<int, Transform> optimizedPoses;
  std::multimap<int, Link> links;
  rtabmap->getGraph(optimizedPoses, links, true, true, &nodes, true, true, true,
                    true);

  spdlog::info("Current backend  [{}]", cv::currentUIFramework());

  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGBL>);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

  for (std::map<int, Transform>::iterator iter = optimizedPoses.begin();
       iter != optimizedPoses.end(); ++iter) {
    Signature node = nodes.find(iter->first)->second;

    // uncompress data
    node.sensorData().uncompressData();

    cv::Mat image = node.sensorData().imageRaw();
    cv::Mat depth = node.sensorData().depthOrRightRaw();
    cv::Mat confidence = node.sensorData().depthConfidenceRaw();

    cv::imshow("Image", image);
    cv::waitKey(1);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp = util3d::cloudRGBFromSensorData(
        node.sensorData(),
        4,    // image decimation before creating the clouds
        4.0f, // maximum depth of the cloud
        0.0f);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpNoNaN(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*tmp, *tmpNoNaN, index);
    if (!tmpNoNaN->empty()) {

      // *cloud += *util3d::transformPointCloud(
      //     tmpNoNaN, iter->second); // transform the point cloud to its pose
      tmpNoNaN = util3d::transformPointCloud(tmpNoNaN, iter->second);
      pcl::PointCloud<pcl::PointXYZRGBL> tmpCloud;
      pcl::copyPointCloud(*tmpNoNaN, tmpCloud);

      // Set sensor origion for the normal estimation
      float x, y, z;
      iter->second.getTranslation(x, y, z);
      tmpCloud.sensor_origin_ = Eigen::Vector4f(x, y, z, 1.0f);
      spdlog::debug("Point origin: {}", tmpCloud.sensor_origin_);

      // Normal estimation
      pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
      ne.setInputCloud(tmpNoNaN);
      ne.setRadiusSearch(0.1);
      pcl::PointCloud<pcl::Normal> normalsTmp;
      ne.compute(normalsTmp);

      *cloud += tmpCloud;
      *normals += normalsTmp;
    }
  }
  cv::destroyAllWindows();

  if (cloud->size()) {
    printf(
        "Voxel grid filtering of the assembled cloud (voxel=%f, %d points)\n",
        0.01f, (int)cloud->size());

    pcl::VoxelGrid<pcl::PointXYZRGBL> voxelGrid;
    voxelGrid.setLeafSize(0.01f, 0.01f, 0.01f);
    voxelGrid.setInputCloud(cloud);
    voxelGrid.filter(*cloud);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices());
    voxelGrid.getRemovedIndices(*indices);

    spdlog::debug("Points after voxel grid filtering: {}", cloud->size());
    spdlog::debug("Removed {} points after voxel grid filtering",
                  indices->indices.size());

    pcl::ExtractIndices<pcl::Normal> extract;
    extract.setInputCloud(normals);
    extract.setIndices(indices);
    extract.setNegative(true);
    extract.filter(*normals);

    spdlog::debug("Normals size after filtering: {}", normals->size());

    // cloud = util3d::voxelize(cloud, 0.01f);

    spdlog::info("Saving rtabmap_cloud.pcd... done! ({})", cloud->size());
    pcl::io::savePCDFile("rtabmap_cloud.pcd", *cloud, true);

    spdlog::info("Saving rtabmap_normals.pcd... done! ({})", normals->size());
    pcl::io::savePCDFile("rtabmap_normals.pcd", *normals, true);
    // pcl::io::savePLYFile("rtabmap_cloud.ply", *cloud); // to save in PLY
    // pcl::io::savePLYFile("rtabmap_cloud.ply", *cloud); // to save in PLY
    // format
  } else {
    spdlog::error("Saving rtabmap_cloud.pcd... failed! The cloud is empty.");
  }

  // Save trajectory
  spdlog::info("Saving rtabmap_trajectory.txt ...");
  if (optimizedPoses.size() &&
      graph::exportPoses("rtabmap_trajectory.txt", 0, optimizedPoses, links)) {
    spdlog::info("Saving rtabmap_trajectory.txt... done!");
  } else {
    spdlog::error("Saving rtabmap_trajectory.txt... failed!");
  }

  rtabmap->close(false);

  return path;
}
