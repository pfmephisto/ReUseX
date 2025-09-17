#include "rux/annotate.hpp"

#include "ReUseX/Yolo.hpp"
#include "ReUseX/fmt_formatter.hpp"
#include "spdmon/spdmon.hpp"

#include <fmt/format.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <pcl/common/io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
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

void setup_subcommand_annotate(CLI::App &app) {

  // Create the option and subcommand objects.
  auto opt = std::make_shared<SubcommandAnnotateOptions>();
  auto *sub = app.add_subcommand(
      "annotate", "Annotate an RTAB-Map database and "
                  "export a point cloud with labels and normals, "
                  "and the trajectory.");

  sub->add_option("database", opt->database_path,
                  "Path to the RTAB-Map database file.")
      ->required()
      ->check(CLI::ExistingFile);

  sub->add_option("cloud", opt->out_cloud_path,
                  "Path to the output point cloud file")
      //->check(CLI::NonexistentPath)
      ->default_val(opt->out_cloud_path);

  sub->add_option("normals", opt->out_normals_path,
                  "Path to the output normals file")
      //->check(CLI::NonexistentPath)
      ->default_val(opt->out_normals_path);

  sub->add_option("trajectory", opt->out_trajectory_path,
                  "Path to the output trajectory file")
      //->check(CLI::NonexistentPath)
      ->default_val(opt->out_trajectory_path);

  sub->add_option("-n, --net", opt->net_path, "Path to the YOLOv8 ONNX model")
      //->check(CLI::ExistingFile)
      ->default_val(opt->net_path);

  sub->add_option("-g,--grid", opt->grid_size,
                  "Voxel grid size for downsampling the point cloud")
      ->default_val(opt->grid_size);

  sub->add_option("--min-distance", opt->min_distance,
                  "Minimum distance points need to be from the camera")
      ->default_val(opt->min_distance);

  sub->add_option("--max-distance", opt->max_distance,
                  "Maximum distance points can be from the camera")
      ->default_val(opt->max_distance);

  sub->add_option("--sampling-factor", opt->sampling_factor,
                  "Factor for downsampling the individual frames")
      ->default_val(opt->sampling_factor);

  sub->add_flag("-c, --cuda", opt->isCuda, "Use CUDA for YOLOv8 inference")
      ->default_val(opt->isCuda);

  sub->add_flag("-s, --skip-inference", opt->skipInference,
                "Skip YOLOv8 inference and only export the point cloud "
                "(default: false, perform inference)")
      ->default_val(opt->skipInference);

  sub->add_flag("--ascii", opt->ascii, "Save the point cloud in ASCII format")
      ->default_val(opt->ascii);

  sub->callback([opt]() {
    spdlog::trace("calling run_subcommand_annotate");
    return run_subcommand_annotate(*opt);
  });
}

int run_subcommand_annotate(SubcommandAnnotateOptions const &opt) {

  spdlog::info("Intializing RTAB-Map ...");
  spdlog::stopwatch timer;
  ParametersMap params;
  Rtabmap rtabmap;
  spdlog::debug("Database path: {}", opt.database_path);
  rtabmap.init(params, opt.database_path.c_str());
  rtabmap.setWorkingDirectory("./");
  spdlog::debug("RTAB-Map initialized in {:.3f}s", timer);

  // Save 3D map
  spdlog::info("Loading Graph");
  timer.reset();

  std::map<int, Transform> poses;
  std::multimap<int, Link> links;
  std::map<int, Signature> nodes;
  rtabmap.getGraph(poses /*poses*/, links /*constraints*/, true /*optimized*/,
                   true /*global*/, &nodes /*signatures*/, true /*withImages*/,
                   true /*withScan*/, true /*withUserData*/,
                   true /*withGrid*/ /*withWords*/ /*withGlobalDescriptors*/);
  spdlog::debug("Graph loaded in {:.3f}s", timer);

  spdlog::trace("Assembling point clouds from signatures");
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);

  { // scope logger variable

    // INFO: Initialize the Yolo model if image segmentation is enabled
    cv::dnn::Net model;
    if (!opt.skipInference) {
      spdlog::trace("Initializing YOLOv8 model");
      model = cv::dnn::readNetFromONNX(opt.net_path);
      model.setPreferableBackend((opt.isCuda) ? cv::dnn::DNN_BACKEND_CUDA
                                              : cv::dnn::DNN_BACKEND_DEFAULT);
      model.setPreferableTarget((opt.isCuda) ? cv::dnn::DNN_TARGET_CUDA
                                             // or DNN_TARGET_CUDA_FP16
                                             : cv::dnn::DNN_TARGET_CPU);
    }

    // INFO: Create vector of poses for openMP parallel processing
    std::vector<std::pair<int, Transform>> poseVector;
    for (std::map<int, Transform>::iterator iter = poses.begin();
         iter != poses.end(); ++iter) {
      poseVector.push_back(*iter);
    }

    spdlog::trace("Number of poses: {}", poseVector.size());
    auto logger = spdmon::LoggerProgress("Assembling cloud", poseVector.size());

#pragma omp parallel for reduction(+ : cloud, labels)                          \
    shared(model, poses, nodes, logger)
    for (int i = 0; i < (int)poseVector.size(); ++i) {
      auto [id, pose] = poseVector[i];

      Signature node = nodes.find(id)->second;
      SensorData data = node.sensorData();
      data.uncompressData();

      spdlog::trace("Creating point cloud from sensor data");
      pcl::IndicesPtr validIndices(new pcl::Indices);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp =
          util3d::cloudRGBFromSensorData(data, opt.sampling_factor,
                                         opt.max_distance, opt.min_distance,
                                         validIndices.get());
      spdlog::debug("Point cloud size: {}, valid: {}, size: {}x{}", tmp->size(),
                    validIndices->size(), tmp->width, tmp->height);

      pcl::PointCloud<pcl::Label>::Ptr labledCloud(
          new pcl::PointCloud<pcl::Label>);
      labledCloud->resize(tmp->size());

      // INFO: Initialize the labels to 0
      for (size_t i = 0; i < labledCloud->size(); ++i)
        labledCloud->points[i].label = 0;

      cv::Size cloud_size(tmp->width, tmp->height);

      spdlog::trace("Set up labledImage");
      cv::Mat image = data.imageRaw();
      cv::rotate(image, image, cv::ROTATE_90_CLOCKWISE);

      cv::Mat labledImage(image.size(), /*CV_32S*/ CV_64F, cv::Scalar(0));

      if (!opt.skipInference) {

        static constexpr size_t netWidth = 640;
        static constexpr size_t netHeight = 640;

        spdlog::trace("Preprocessing image for YOLOv8");
        cv::Mat netInputImg;
        cv::Vec4d params;
        cv::Size netInputSize(netWidth, netHeight);
        ReUseX::LetterBox(image, netInputImg, params, netInputSize);
        cv::dnn::blobFromImage(netInputImg, netInputImg, 1.0 / 255,
                               netInputSize, cv::Scalar(0, 0, 0), true, false);

        std::vector<cv::Mat> net_output_img;
#pragma omp critical
        {
          spdlog::trace("Running YOLOv8 inference");
          model.setInput(netInputImg);
          std::vector<std::string> output_layer_names{"output0", "output1"};
          model.forward(net_output_img, output_layer_names);
        }

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

  // TODO: Add assertions
  if (!cloud->size()) {
    spdlog::error("Error: The cloud is empty.");
    rtabmap.close(false);
    return 1;
  }

  // INFO: Downsample the point cloud
  spdlog::info(
      "Voxel grid filtering of the assembled cloud (voxel={}, {} points)",
      opt.grid_size, (int)cloud->size());
  auto downsampledCloud = util3d::voxelize(cloud, opt.grid_size);
  spdlog::debug("Points after voxel grid filtering: {}",
                downsampledCloud->size());

  pcl::Indices filter_indices;
  pcl::removeNaNNormalsFromPointCloud(*downsampledCloud, *downsampledCloud,
                                      filter_indices);

  // INFO: Saving the point cloud with normals
  spdlog::info("Saving {} ({})", opt.out_normals_path,
               downsampledCloud->size());
  pcl::io::savePCDFile(opt.out_normals_path, *downsampledCloud, !opt.ascii);

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
    kdtree.radiusSearch(point, opt.grid_size / 2.0f, indices, distances);
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
  spdlog::info("Saving {} ({})", opt.out_cloud_path, pointLabels->size());
  pcl::io::savePCDFile(opt.out_cloud_path, *pointLabels, !opt.ascii);

  // INFO: Saving the trajectory
  spdlog::info("Saving {} ...", opt.out_trajectory_path);
  if (poses.size() &&
      graph::exportPoses(opt.out_trajectory_path, 0, poses, links)) {
    // const std::string & filePath,
    // int format, // 0=Raw, 1=RGBD-SLAM motion capture (10=without change of
    // coordinate frame, 11=10+ID), 2=KITTI, 3=TORO, 4=g2o
    // const std::map<int, Transform> & poses
    // const std::multimap<int, Link> & constraints required for formats 3 and
    // 4 const std::map<int, double> & stamps required for format 1, 10 and 11
    // const ParametersMap & parameters) optional for formats 3 and 4

    // Raw = KITTI Fromat
    // // Format: r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
  } else {
    spdlog::error("Saving {} ... failed!", opt.out_trajectory_path);
    rtabmap.close(false);
    return 1;
  }

  rtabmap.close();
  return 0;
}
