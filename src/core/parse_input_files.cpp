#define PCL_NO_PRECOMPILE
#include "parse_input_files.hh"
#include "fmt_formatter.hh"
#include "io.hh"
#include "spdmon.hh"
#include "types/Accumulators.hh"
#include "types/Dataset.hh"
#include "types/Yolo.hh"

#include <pcl/common/impl/accumulators.hpp>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/memory.h>
#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>         //TODO: TEST
#include <pcl/range_image/range_image.h>        //TODO: TEST
#include <pcl/range_image/range_image_planar.h> //TODO: TEST
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/visualization/cloud_viewer.h>

#include <fmt/color.h>
#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/printf.h>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <opencv4/opencv2/imgproc.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <cassert>
#include <future>
#include <mutex>
#include <numeric>
#include <optional>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

// #define DISPLAY

namespace ReUseX {

template <typename PointCloud>
static void setHeader(PointCloud &cloud, Eigen::MatrixXd const &odometry,
                      std::optional<size_t> i) {

  uint64_t time_stamp = odometry(0, 0);
  std::string frame_id = fmt::format("{:06}", (int)odometry(0, 1));

  auto pos = odometry.block<1, 3>(0, 2);  // x,y,z
  auto quat = odometry.block<1, 4>(0, 5); // x,y,z,w

  Eigen::Vector4f origin(pos(0), pos(1), pos(2), 1);

  Eigen::Quaternionf orientation(quat(3), quat(0), quat(1), quat(2));

  cloud.header.seq = i.has_value() ? i.value() : 0;
  cloud.header.frame_id = frame_id;
  cloud.header.stamp = time_stamp;

  cloud.sensor_origin_ = origin;
  cloud.sensor_orientation_ = orientation;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
CreateCloud(DataItem const &data, double fx, double fy, double cx, double cy,
            std::optional<size_t const> i) {

  // We need at least the depth info
  auto fields = data.fields();
  assert(std::count(fields.begin(), fields.end(), Field::DEPTH) == 1);

  pcl::RangeImagePlanar::Ptr points =
      pcl::RangeImagePlanar::Ptr(new pcl::RangeImagePlanar);

  auto depth = data.get<Field::DEPTH>();
  points->setDepthImage(
      depth.data(), depth.rows(),
      depth.cols(), // ( Width, Height) The Eigne matrix seems reversed but it
                    // works when accessing the buffer
      cx, cy, fx, fy);

  // Convert the RangeImage to a Point Cloud
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud =
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(
          new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::copyPointCloud(*points, *cloud);

  auto width = points->width;
  auto height = points->height;

  // If there is color information add it
  bool has_color_values =
      std::find(fields.begin(), fields.end(), Field::COLOR) != fields.end();
  cv::Mat image;
  if (has_color_values) {
    image = data.get<Field::COLOR>();
    cv::resize(image, image, cv::Size(width, height));
  }

  // If there there are confidence values add them
  bool has_confidence_values = std::find(fields.begin(), fields.end(),
                                         Field::CONFIDENCE) != fields.end();
  Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> confidence;

  if (has_confidence_values)
    confidence = data.get<Field::CONFIDENCE>();

  // Set values
  if (has_color_values || has_confidence_values) {
#pragma omp parallel for shared(cloud, image, confidence)
    for (size_t idx = 0; idx < cloud->size(); idx++) {
      if (has_color_values) {
        cv::Vec3b vec = image.at<cv::Vec3b>(idx);
        cloud->at(idx).r = vec[0];
        cloud->at(idx).g = vec[1];
        cloud->at(idx).b = vec[2];
      }
      if (has_confidence_values)
        cloud->at(idx).a = (confidence(idx) == 2)   ? 255
                           : (confidence(idx) == 1) ? 50
                                                    : 0;
    }
  }

  // Set Header
  if (std::find(fields.begin(), fields.end(), Field::ODOMETRY) != fields.end())
    setHeader(*cloud, data.get<Field::ODOMETRY>(), i);

  // Get Pose
  auto pose = Eigen::Affine3f::Identity();
  pose.linear() = cloud->sensor_orientation_.toRotationMatrix();
  pose.translation() = cloud->sensor_origin_.template head<3>();

  // cloud->sensor_origin_ = Eigen::Vector4f(0, 0, 0, 1);
  // cloud->sensor_orientation_ = Eigen::Quaternionf::Identity();

  pcl::transformPointCloud(*cloud, *cloud, pose);

  return cloud;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
CreateCloud(DataItem const &data, Eigen::Matrix3d intrinsic_matrix,
            std::optional<size_t const> seq_index) {

  auto fx = intrinsic_matrix(0, 0);
  auto fy = intrinsic_matrix(1, 1);

  auto cx = intrinsic_matrix(0, 2);
  auto cy = intrinsic_matrix(1, 2);

  return CreateCloud(data, fx, fy, cx, cy, seq_index);
};

fs::path ParseDataset(Dataset const &dataset,
                      std::optional<fs::path> output_path_,
                      Eigen::Matrix3d const &intrinsic_matrix,
                      std::optional<std::vector<size_t> *> samples_) {

  // Set output path
  fs::path output_path = output_path_.has_value()
                             ? output_path_.value()
                             : fs::path("./") / dataset.name();

  // set the samples
  std::vector<size_t> samples;
  if (samples_.has_value())
    samples = *samples_.value();
  else {
    samples.resize(dataset.size());
    std::iota(samples.begin(), samples.end(), 0);
  }

  auto monitor = spdmon::LoggerProgress("Processing data", samples.size());
  spdlog::stopwatch sw;

#pragma omp parallel for firstprivate(output_path)                             \
    shared(dataset, intrinsic_matrix)
  for (size_t i = 0; i < samples.size(); i++) {

    auto data = dataset[samples[i]];

    auto cloud = CreateCloud(data, intrinsic_matrix, i);

    // Save cloud
    std::filesystem::create_directory(output_path);
    std::filesystem::path file_path(fmt::format(
        "{}/cloud_{}.pcd", output_path.c_str(), cloud->header.frame_id));

    save<pcl::PointXYZRGBA>(file_path, cloud);

    ++monitor;
  }
  spdlog::info("Elapsed time: {}s", sw);

  return output_path;
}

void parse_Dataset(Dataset const &dataset, std::string const &output_path,
                   int start, std::optional<int> stop_in, int step) {

  // TODO: Make output path optional.
  // And in case it is not set use a temp folder
  // auto path = fs::temp_directory_path() / "ReUseX";

  // TODO: Optionally clear the output folder before saving the files
  // if (fs::exists(path))
  //    fs::remove_all(path);

  auto fields = dataset.fields();

  bool check = true;
  check &= std::count(fields.begin(), fields.end(), Field::COLOR) == 1;
  check &= std::count(fields.begin(), fields.end(), Field::DEPTH) == 1;
  check &= std::count(fields.begin(), fields.end(), Field::CONFIDENCE) == 1;
  check &= std::count(fields.begin(), fields.end(), Field::ODOMETRY) == 1;
  // check &= std::count(fields.begin(), fields.end(), Field::POSES) == 1;

  if (!check) {
    fmt::print(fg(fmt::color::red),
               "Dataset does not contain all required fields [COLOR, DEPTH, "
               "CONFIDENCE, ODOMETRY]");
    return;
  }

  int stop;
  if (stop_in.has_value())
    stop = stop_in.value();
  else
    stop = dataset.size();

  if (start < 0)
    start = dataset.size() + start;
  if (stop < 0)
    stop = dataset.size() + stop;
  if (start < 0 || start >= dataset.size() || stop < 0 ||
      stop > dataset.size() || step == 0) {
    fmt::print(fg(fmt::color::red), "Invalid start, stop or step\n");
    return;
  }

  size_t n_frames = (stop - start) / step;
  double ratio = (double)n_frames / (double)dataset.size() * 100;

  // Print info
  fmt::print("Number of frames: ");
  fmt::print(fg(fmt::color::red), "{}", n_frames);
  fmt::print(fmt::emphasis::italic,
             " -> ( start: {}; step: {}; end: {} ) {:3.2f}% \n", start, step,
             stop, ratio);

  if (n_frames == 0)
    return;

#if DISPLAY
  pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
#endif

  auto monitor = spdmon::LoggerProgress("Processing data", n_frames);
  spdlog::stopwatch sw;

#pragma omp parallel for firstprivate(step, start, output_path) shared(dataset)
  for (size_t i = 0; i < n_frames; i++) {

    size_t index = start + (i * step);
    auto data = dataset[index];

    pcl::RangeImagePlanar::Ptr points =
        pcl::RangeImagePlanar::Ptr(new pcl::RangeImagePlanar);
    auto intrinsic_matrix = dataset.intrinsic_matrix();
    auto fx = intrinsic_matrix(0, 0);
    auto fy = intrinsic_matrix(1, 1);
    auto cx = intrinsic_matrix(2, 0);
    auto cy = intrinsic_matrix(2, 1);

    auto depth = data.get<Field::DEPTH>();
    points->setDepthImage(
        depth.data(), depth.rows(),
        depth.cols(), // ( Width, Height) The Eigne matrix seems reversed but it
                      // works when accessing the buffer
        cx, cy, fx, fy);

    // depth.normalize();
    // depth *= 255;
    // cv::Mat depthCV;
    // cv::eigen2cv(depth, depthCV);
    // cv::imshow("Depth", depthCV);
    // cv::waitKey(0);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud =
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(
            new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::copyPointCloud(*points, *cloud);

    auto width = points->width;
    auto height = points->height;
    // fmt::printf("Size: %dx%d\n", width, height);

    cv::Mat image = data.get<Field::COLOR>();

    cv::resize(image, image, cv::Size(width, height));

    auto confidence = data.get<Field::CONFIDENCE>();

#pragma omp parallel for shared(cloud, image, confidence)
    for (size_t idx = 0; idx < cloud->size(); idx++) {
      cv::Vec3b vec = image.at<cv::Vec3b>(idx);
      cloud->at(idx).r = vec[0];
      cloud->at(idx).g = vec[1];
      cloud->at(idx).b = vec[2];
      cloud->at(idx).a = (confidence(idx) == 2)   ? 255
                         : (confidence(idx) == 1) ? 50
                                                  : 0;
    }

    // cv::Mat confidenceCV;
    // confidence = (confidence.array() == 1).select(128, confidence); //
    // Replace 1 with 128 confidence = (confidence.array() == 2).select(255,
    // confidence); // Replace 2 with 255 cv::eigen2cv(confidence,
    // confidenceCV); cv::imshow("Confidence", confidenceCV); cv::waitKey(0);

    // Get Pose
    auto odometry = data.get<Field::ODOMETRY>();
    auto pose = Eigen::Affine3f::Identity();
    auto p = odometry.block<1, 3>(0, 2);
    auto q = odometry.block<1, 4>(0, 5);

    // Set the rotation using the quaternion
    Eigen::Quaternionf quat(/*W*/ q[3], /*X*/ q[0], /*Y*/ q[1], /*Z*/ q[2]);
    pose.linear() = quat.toRotationMatrix();
    pose.translation() = Eigen::Vector3f(p[0], p[1], p[2]);

    pcl::transformPointCloud(*cloud, *cloud, pose);

    setHeader(*cloud, data.get<Field::ODOMETRY>(), i);

    // cv::destroyAllWindows();

#if DISPLAY
    // Display the cloud
    auto points_xyz =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *points_xyz);
    viewer.showCloud(points_xyz, fmt::format("Cloud {}", i));
#endif

    // Save cloud
    std::filesystem::create_directory(output_path);
    std::filesystem::path file_path(
        fmt::format("{}/cloud_{}.pcd", output_path, cloud->header.frame_id));

    save<pcl::PointXYZRGBA>(file_path, cloud);

    ++monitor;
  }
  spdlog::info("Elapsed time: {}s", sw);

#if DISPLAY
  while (!viewer.wasStopped()) {
  }
#endif
}

} // namespace ReUseX
