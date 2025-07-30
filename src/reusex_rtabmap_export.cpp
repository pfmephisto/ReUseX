#include "core/fmt_formatter.hh"
#include "core/spdmon.hh"
#include "types/Yolo.hh"

#include <CLI/CLI.hpp>
#include <fmt/format.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <pcl/common/io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/optimizer/OptimizerG2O.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>

#include <filesystem>

namespace fs = std::filesystem;
using namespace rtabmap;

struct Params {
  fs::path database_path;
  fs::path out_cloud_path = fs::current_path() / "cloud.pcd";
  fs::path out_normals_path = fs::current_path() / "normals.pcd";
  fs::path out_trajectory_path = fs::current_path() / "trajectory.txt";
  fs::path net_path = fs::current_path() / "yolov8x-seg.onnx";

  bool isCuda{false};
  bool skipInference{false};
  bool ascii{false};
  float min_distance = 0.00f;
  float max_distance = 4.00f;
  size_t sampling_factor = 4;
  float grid_size = 0.01f;

  int verbosity = 0;

  bool validate() {
    // INFO: Check for valid values
    if (verbosity < 0 || verbosity > 3)
      spdlog::warn("Verbosity must be between 0 and 3.");
    // 0 > trace, 1 > debug, 2 > info, 3 > warn, 4 > err, 5 > critical, 6 > of
    verbosity = std::max(verbosity, 0);
    verbosity = std::min(verbosity, 3);
    verbosity = 3 - verbosity;

    if (database_path.empty()) {
      spdlog::error("Database path is empty.");
      return false;
    }

    return true;
  };
};

std::string shell;
CLI::App *comp = nullptr;

std::unique_ptr<CLI::App> initApp(Params &params) {

  auto app = std::make_unique<CLI::App>(
      "This tool exports a rtab-map database to a pcl point cloud for "
      "use in reusex.");

  app->get_formatter()->column_width(40);

  app->add_option("database", params.database_path,
                  "Path to the RTAB-Map database file.")
      ->required()
      ->check(CLI::ExistingFile);

  app->add_option("cloud", params.out_cloud_path,
                  "Path to the output point cloud file")
      //->check(CLI::NonexistentPath)
      ->default_val(params.out_cloud_path);

  app->add_option("normals", params.out_normals_path,
                  "Path to the output normals file")
      //->check(CLI::NonexistentPath)
      ->default_val(params.out_normals_path);

  app->add_option("trajectory", params.out_trajectory_path,
                  "Path to the output trajectory file")
      //->check(CLI::NonexistentPath)
      ->default_val(params.out_trajectory_path);

  app->add_option("-n, --net", params.net_path, "Path to the YOLOv8 ONNX model")
      ->check(CLI::ExistingFile)
      ->default_val(params.net_path);

  app->add_option("-g,--grid", params.grid_size,
                  "Voxel grid size for downsampling the point cloud")
      ->default_val(params.grid_size);

  app->add_option("--min-distance", params.min_distance,
                  "Minimum distance points need to be from the camera")
      ->default_val(params.min_distance);

  app->add_option("--max-distance", params.max_distance,
                  "Maximum distance points can be from the camera")
      ->default_val(params.max_distance);

  app->add_option("--sampling-factor", params.sampling_factor,
                  "Factor for downsampling the individual frames")
      ->default_val(params.sampling_factor);

  app->add_flag("-c, --cuda", params.isCuda, "Use CUDA for YOLOv8 inference")
      ->default_val(params.isCuda);

  app->add_flag("-s, --skip-inference", params.skipInference,
                "Skip YOLOv8 inference and only export the point cloud "
                "(default: false, perform inference)")
      ->default_val(params.skipInference);

  app->add_flag("--ascii", params.ascii, "Save the point cloud in ASCII format")
      ->default_val(params.ascii);

  app->add_flag("-v,--verbose", params.verbosity,
                "Increase verbosity, use -vv & -vvv "
                "for more verbosity")
      ->multi_option_policy(CLI::MultiOptionPolicy::Sum);

  return app;
}

int main(int argc, char **argv) {

  Params config;
  auto app = initApp(config);
  argv = app->ensure_utf8(argv);
  CLI11_PARSE(*app, argc, argv);

  if (!config.validate())
    return 1;

  spdlog::set_level(static_cast<spdlog::level::level_enum>(config.verbosity));
  spdlog::trace("reusex_rtabmap_export started");

  // ParametersMap params;
  // Rtabmap rtabmap;
  spdlog::debug("Database path: {}", config.database_path);
  // rtabmap.init(params, config.database_path.c_str());
  // rtabmap.setWorkingDirectory("./");

  // Save 3D map
  spdlog::info("Processing database");
  spdlog::stopwatch timer;

  ParametersMap parameters;
  DBDriver *driver = DBDriver::create();
  if (driver->openConnection(config.database_path.c_str())) {
    parameters = driver->getLastParameters();
    driver->closeConnection(false);
  }
  delete driver;
  driver = 0;
  for (ParametersMap::iterator iter = parameters.begin();
       iter != parameters.end(); ++iter)
    spdlog::debug("Param {}={}", iter->first.c_str(), iter->second.c_str());

  std::shared_ptr<DBDriver> dbDriver(DBDriver::create(parameters));
  if (!dbDriver->openConnection(config.database_path.c_str())) {
    printf("Failed to open database \"%s\"!\n", config.database_path.c_str());
    return -1;
  }
  spdlog::info("Opening database \"{}\"... done ({}s).", config.database_path,
               timer);

  std::map<int, Transform> odomPoses;
  std::multimap<int, Link> links;

  dbDriver->getAllOdomPoses(odomPoses, true);
  dbDriver->getAllLinks(links, true, true);

  spdlog::trace("Initializing Optimizer");
  std::shared_ptr<Optimizer> optimizer(Optimizer::create(parameters));
  std::map<int, Transform> posesOut;
  std::multimap<int, Link> linksOut;

  optimizer->getConnectedGraph(odomPoses.lower_bound(1)->first, odomPoses,
                               links, posesOut, linksOut);
  spdlog::trace("Getting optimized poses and linkgs");
  std::map<int, Transform> optimizedPoses =
      optimizer->optimize(odomPoses.lower_bound(1)->first, posesOut, linksOut);

  if (optimizedPoses.empty()) {
    spdlog::error("The optimized graph is empty!? Aborting...");
    return -1;
  }

  if (false) {
    spdlog::info("Global bundle adjustment...\n");
    // UASSERT(optimizedPoses.lower_bound(1) != optimizedPoses.end());
    OptimizerG2O g2o(parameters);
    std::list<int> ids;
    for (std::map<int, Transform>::iterator iter =
             optimizedPoses.lower_bound(1);
         iter != optimizedPoses.end(); ++iter) {
      ids.push_back(iter->first);
    }
    std::list<Signature *> signatures;
    dbDriver->loadSignatures(ids, signatures);
    std::map<int, Signature> nodes;
    for (std::list<Signature *>::iterator iter = signatures.begin();
         iter != signatures.end(); ++iter) {
      nodes.insert(std::make_pair((*iter)->id(), *(*iter)));
    }
    optimizedPoses = ((Optimizer *)&g2o)
                         ->optimizeBA(optimizedPoses.lower_bound(1)->first,
                                      optimizedPoses, links, nodes, true);
    spdlog::info("Global bundle adjustment... done ({}s).", timer);
  }

  // INFO: Start old section

  // std::map<int, Transform> optimizedPoses;
  // std::multimap<int, Link> links;
  // rtabmap.getGraph(optimizedPoses /*poses*/, links /*constraints*/,
  //                 true /*optimized*/, true /*global*/, &nodes /*signatures*/,
  //                 true /*withImages*/, true /*withScan*/,
  //                 true /*withUserData*/,
  //                 true /*withGrid*/ /*withWords*/ /*withGlobalDescriptors*/);

  spdlog::trace("Assembling point clouds from signatures");
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);

  {
    spdlog::trace("Number of poses: {}", posesOut.size());
    auto logger = spdmon::LoggerProgress("Assembling cloud", posesOut.size());

    // TODO: Wratp this in a openMP parallel loop
    // and make the seperate steps tasks.
    // Figure out how to batch the calls to the Yolo model

    cv::dnn::Net model;
    if (!config.skipInference) {
      spdlog::trace("Initializing YOLOv8 model");
      model = cv::dnn::readNetFromONNX(config.net_path);
      model.setPreferableBackend((config.isCuda)
                                     ? cv::dnn::DNN_BACKEND_CUDA
                                     : cv::dnn::DNN_BACKEND_DEFAULT);
      model.setPreferableTarget((config.isCuda) ? cv::dnn::DNN_TARGET_CUDA
                                                // or DNN_TARGET_CUDA_FP16
                                                : cv::dnn::DNN_TARGET_CPU);
    }

    for (std::map<int, Transform>::iterator iter = optimizedPoses.begin();
         iter != optimizedPoses.end(); ++iter) {

      SensorData data;
      dbDriver->getNodeData(iter->first, data, true /*loadImages*/,
                            true /*loadScan*/, false, false);
      // Signature node = nodes.find(iter->first)->second;

      // uncompress data
      data.uncompressData();

      spdlog::trace("Creating point cloud from sensor data");
      pcl::IndicesPtr validIndices(new pcl::Indices);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp =
          util3d::cloudRGBFromSensorData(
              data, config.sampling_factor, config.max_distance,
              config.min_distance, validIndices.get());
      spdlog::debug("Point cloud size: {}, valid: {}, size: {}x{}", tmp->size(),
                    validIndices->size(), tmp->width, tmp->height);

      pcl::PointCloud<pcl::Label>::Ptr labledCloud(
          new pcl::PointCloud<pcl::Label>);
      labledCloud->resize(tmp->size());

      cv::Size cloud_size(tmp->width, tmp->height);

      spdlog::trace("Set up labledImage");
      cv::Mat image = data.imageRaw();
      cv::rotate(image, image, cv::ROTATE_90_CLOCKWISE);

      cv::Mat labledImage(image.size(), /*CV_32S*/ CV_64F, cv::Scalar(0));

      if (!config.skipInference) {

        static constexpr size_t netWidth = 640;
        static constexpr size_t netHeight = 640;

        spdlog::trace("Preprocessing image for YOLOv8");
        cv::Mat netInputImg;
        cv::Vec4d params;
        cv::Size netInputSize(netWidth, netHeight);
        ReUseX::LetterBox(image, netInputImg, params, netInputSize);
        cv::dnn::blobFromImage(netInputImg, netInputImg, 1.0 / 255,
                               netInputSize, cv::Scalar(0, 0, 0), true, false);

        spdlog::trace("Running YOLOv8 inference");
        model.setInput(netInputImg);
        std::vector<cv::Mat> net_output_img;
        std::vector<std::string> output_layer_names{"output0", "output1"};
        model.forward(net_output_img, output_layer_names);

        spdlog::trace("Postprocessing YOLOv8 output");
        std::vector<ReUseX::OutputParams> results =
            ReUseX::Postprocess(net_output_img, params, image.size());

        spdlog::trace("Annotating point cloud with masks");
        for (auto &result : results) {
          labledImage(result.box).setTo(result.id + 1, result.boxMask);
        }

        spdlog::trace("Rotate image back");
        cv::rotate(labledImage, labledImage, cv::ROTATE_90_COUNTERCLOCKWISE);

        spdlog::trace("Resize image to cloud size");
        cv::resize(labledImage, labledImage, cloud_size);
      }

      spdlog::trace("Set the values of the labled cloud");
      for (size_t i = 0; i < labledCloud->size(); ++i)
        labledCloud->points[i].label =
            std::round<size_t>(labledImage.at<double>(i));

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
        tmp = util3d::transformPointCloud(tmp, iter->second);

        spdlog::trace("Setting sensor origin for normal estimation");
        float x, y, z;
        iter->second.getTranslation(x, y, z);
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
        //     "Point 0: XYZ({}, {}, {}), RGB({}, {}, {}), Normal({}, {}, {})",
        //     tmpCloud->at(0).x, tmpCloud->at(0).y, tmpCloud->at(0).z,
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

  // TODO: Add assertions
  if (!cloud->size()) {
    spdlog::error("Error: The cloud is empty.");
    // rtabmap.close(false);
    return 1;
  }

  // INFO: Downsample the point cloud
  spdlog::info(
      "Voxel grid filtering of the assembled cloud (voxel={}, {} points)",
      config.grid_size, (int)cloud->size());
  auto downsampledCloud = util3d::voxelize(cloud, config.grid_size);
  spdlog::debug("Points after voxel grid filtering: {}",
                downsampledCloud->size());

  // INFO: Saving the point cloud with normals
  spdlog::info("Saving {} ({})", config.out_normals_path,
               downsampledCloud->size());
  pcl::io::savePCDFile(config.out_normals_path, *downsampledCloud,
                       !config.ascii);

  spdlog::trace("Creating point cloud with labels");
  pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;
  kdtree.setInputCloud(cloud);

  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pointLabels(
      new pcl::PointCloud<pcl::PointXYZRGBL>);
  pointLabels->resize(downsampledCloud->size());
  pcl::copyPointCloud(*downsampledCloud, *pointLabels);
  for (size_t i = 0; i < downsampledCloud->size(); ++i) {
    auto &point = downsampledCloud->at(i);
    std::vector<int> indices;
    std::vector<float> distances;
    kdtree.radiusSearch(point, config.grid_size / 2.0f, indices, distances);
    std::unordered_map<int, int> labelCount;
    for (const auto &idx : indices) {
      int label = labels->points[idx].label;
      labelCount[label]++;
    }
    // Find the most common label
    int maxCount = 0;
    int mostCommonLabel = -1;
    for (const auto &pair : labelCount) {
      if (pair.second > maxCount) {
        maxCount = pair.second;
        mostCommonLabel = pair.first;
      }
    }
    pointLabels->points[i].label = mostCommonLabel;
    // pointLabels->at(i).label = labels->points.at(i).label;
  }

  // INFO: Saving the point cloud with labels
  spdlog::info("Saving {} ({})", config.out_cloud_path, pointLabels->size());
  pcl::io::savePCDFile(config.out_cloud_path, *pointLabels, !config.ascii);

  // INFO: Saving the trajectory
  spdlog::info("Saving {} ...", config.out_trajectory_path);
  if (optimizedPoses.size() && graph::exportPoses(config.out_trajectory_path, 0,
                                                  optimizedPoses, linksOut)) {
    // const std::string & filePath,
    // int format, // 0=Raw, 1=RGBD-SLAM motion capture (10=without change of
    // coordinate frame, 11=10+ID), 2=KITTI, 3=TORO, 4=g2o
    // const std::map<int, Transform> & poses
    // const std::multimap<int, Link> & constraints required for formats 3 and 4
    // const std::map<int, double> & stamps required for format 1, 10 and 11
    // const ParametersMap & parameters) optional for formats 3 and 4

    // Raw = KITTI Fromat
    // // Format: r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
  } else {
    spdlog::error("Saving {} ... failed!", config.out_trajectory_path);
    // rtabmap.close(false);
    return 1;
  }

  // rtabmap.close();
  return 0;
}
