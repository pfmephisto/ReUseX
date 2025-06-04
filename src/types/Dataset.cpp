#include "./Dataset.hh"
#include "core/lodepng.hh"

#include <fmt/printf.h>

#include <fstream>
#include <iostream>
#include <optional>
#include <string>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/opencv.hpp>

#define CHECK_EXSISTANCE(path)                                                 \
  if (!std::filesystem::exists(path)) {                                        \
    spdlog::error("{} does not exist", path);                                  \
  }

Eigen::MatrixXd read_csv(std::ifstream &stream, char delimiter = ',',
                         bool header = false) {

  std::string line;
  std::vector<double> values;
  int rows = 0;
  int cols = 0;

  if (header)
    std::getline(stream, line);

  while (getline(stream, line)) {
    std::stringstream line_stream(line);
    std::string cell;

    cols = 0;
    while (std::getline(line_stream, cell, delimiter)) {
      values.push_back(std::stod(cell));
      cols++;
    }
    rows++;
  }

  Eigen::MatrixXd matrix(rows, cols);
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      matrix(i, j) = values[i * cols + j];
    }
  }

  return matrix;
}

std::optional<cv::Mat> read_frame_at_index(std::filesystem::path const &path,
                                           int idx) {

  cv::VideoCapture cap(path.c_str());
  cap.set(cv::CAP_PROP_POS_FRAMES, idx);

  cv::Mat frame;
  bool sucsess = cap.read((cv::OutputArray)frame);

  if (!sucsess) {
    fmt::print("Failed to read frame at index: {}\n", idx);
    return {};
  }

  cv::cvtColor((cv::InputArray)frame, (cv::OutputArray)frame,
               cv::COLOR_BGR2RGB);

  return frame;
}

std::optional<Eigen::Matrix3d>
read_camera_matrix(std::filesystem::path const &path) {

  if (!std::filesystem::exists(path)) {
    fmt::print("Camera matrix file does not exist\n");
    fmt::print("Path: {}\n", path.c_str());
    return {};
  }

  std::ifstream camera_matrix_stream;
  camera_matrix_stream.open(path);

  auto matrix = read_csv(camera_matrix_stream, ',', false);

  Eigen::Matrix3d mat = Eigen::Matrix3d::Identity();

  // (int col, int row)  <-- Tyepd geometry
  // (int row, int col)  <-- Eigen

  mat(0, 0) = matrix(0, 0);

  mat(0, 1) = matrix(1, 0);
  mat(1, 0) = matrix(0, 1);
  mat(1, 1) = matrix(1, 1);

  mat(0, 2) = matrix(2, 0);
  mat(1, 2) = matrix(2, 1);
  mat(2, 0) = matrix(0, 2);
  mat(2, 1) = matrix(1, 2);
  mat(2, 2) = matrix(2, 2);

  // mat(0,3) = matrix(3,0);
  // mat(1,3) = matrix(3,1);
  // mat(2,3) = matrix(3,2);
  // mat(3,0) = matrix(0,3);
  // mat(3,1) = matrix(1,3);
  // mat(3,2) = matrix(2,3);
  // mat(3,3) = matrix(3,3);

  return mat;
}

std::optional<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>
read_odometry(std::filesystem::path const &path) {

  if (!std::filesystem::exists(path)) {
    fmt::print("Odometry file does not exist\n");
    fmt::print("Path: {}\n", path.c_str());

    return {};
  }

  std::ifstream odometry_matrix_stream;
  odometry_matrix_stream.open(path);

  return read_csv(odometry_matrix_stream, ',', true);
}

std::optional<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>
read_imu(std::filesystem::path const &path) {

  if (!std::filesystem::exists(path)) {
    fmt::print("IMU file does not exist\n");
    fmt::print("Path: {}\n", path.c_str());

    return {};
  }

  std::ifstream imu_matrix_stream;
  imu_matrix_stream.open(path);

  return read_csv(imu_matrix_stream, ',', true);
}

std::optional<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>>
read_depth_image(std::filesystem::path const &path) {

  std::vector<unsigned char> buffer1;
  unsigned width, height;

  // decode
  unsigned error = lodepng::decode(buffer1, width, height, path,
                                   LodePNGColorType::LCT_GREY, 16);

  // if there's an error, display it
  if (error)
    std::cout << "decoder error " << error << ": " << lodepng_error_text(error)
              << std::endl;
  if (error)
    return {};

  std::vector<float> buffer2;

  for (size_t i = 0; i < buffer1.size(); i += 2) {
    uint16_t value =
        (buffer1[i] << 8) | buffer1[i + 1]; // Combine two bytes into a uint16_t
    buffer2.push_back(value);               /// 1000.0f
  }

  Eigen::Map<
      Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>>
      matrix(buffer2.data(), width, height);
  return matrix / 1000.0f; // Convert to meters;
}

std::optional<Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>>
read_confidence_image(std::filesystem::path const &path) {

  std::vector<unsigned char> buffer;
  unsigned width, height;

  // decode
  unsigned error = lodepng::decode(buffer, width, height, path,
                                   LodePNGColorType::LCT_GREY, 8);

  // if there's an error, display it
  if (error)
    std::cout << "decoder error " << error << ": " << lodepng_error_text(error)
              << std::endl;
  if (error)
    return {};

  Eigen::Map<
      Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>>
      matrix(buffer.data(), width, height);
  return matrix;
}

Eigen::Transform<float, 3, Eigen::Affine>
create_pose(Eigen::Block<const Eigen::MatrixXd, 1, 3> const &p,
            Eigen::Block<const Eigen::MatrixXd, 1, 4> const &q) {

  // Create an Eigen::Transform object
  Eigen::Transform<float, 3, Eigen::Affine> transform;

  // Set the rotation using the quaternion
  Eigen::Quaternionf quat(q[3], q[0], q[1], q[2]);
  transform.linear() = quat.toRotationMatrix();

  // Set the translation
  transform.translation() = Eigen::Vector3f(p[0], p[1], p[2]);
  Eigen::Matrix4d mat;

  // Set the matrix
  return transform;
}

namespace ReUseX {

Dataset::Dataset(const std::filesystem::path &path) {

  // Check if directories and files exists
  assert(std::filesystem::exists(path) &&
         fmt::format("Directory does not exist: {}", path.string()).c_str());

  _path = path;

  spdlog::info("Creating Dataset: {}", _path.parent_path().filename().c_str());

  struct winsize size;
  ioctl(STDOUT_FILENO, TIOCGWINSZ, &size);

  spdlog::debug("Path: {}", trimPathMiddle(_path.c_str(), 51));

  _n_frames = get_number_of_frames(_path / "rgb.mp4");
  spdlog::debug("Number of frames: {}", _n_frames);

  spdlog::stopwatch sw;

  CHECK_EXSISTANCE(_path / "rgb.mp4");
  CHECK_EXSISTANCE(_path / "depth");
  CHECK_EXSISTANCE(_path / "confidence");
  CHECK_EXSISTANCE(_path / "odometry.csv");
  CHECK_EXSISTANCE(_path / "imu.csv");

  _odometry_data = read_odometry(_path / "odometry.csv").value();
  _imu_data = read_imu(_path / "imu.csv").value();

  //// Get depth image paths
  // std::transform(std::filesystem::directory_iterator(_path / "depth"),
  //                std::filesystem::directory_iterator(),
  //                std::back_inserter(_depth_paths),
  //                [](const auto &entry) { return entry.path(); });
  // std::sort(_depth_paths.begin(), _depth_paths.end());

  //// Get depth image paths
  // std::transform(std::filesystem::directory_iterator(_path / "confidence"),
  //                std::filesystem::directory_iterator(),
  //                std::back_inserter(_confidence_paths),
  //                [](const auto &entry) { return entry.path(); });
  // std::sort(_confidence_paths.begin(), _confidence_paths.end());

  spdlog::debug("Loaded fields in {}s", sw);
}

size_t Dataset::get_number_of_frames(const std::filesystem::path &path) {
  cv::VideoCapture cap(path);
  return cap.get(cv::CAP_PROP_FRAME_COUNT);
};

Eigen::Matrix<double, 3, 3> Dataset::intrinsic_matrix() const {
  auto matrix = read_camera_matrix(_path / "camera_matrix.csv").value();

  // Scale intrinsic matrix to match depth image size
  auto scale_x = (double)_depth_width / (double)_rgb_width;
  auto scale_y = (double)_depth_hight / (double)_rgb_hight;

  matrix(0, 0) = matrix(0, 0) * scale_x;
  matrix(1, 1) = matrix(1, 1) * scale_y;
  matrix(2, 0) = matrix(2, 0) * scale_x;
  matrix(2, 1) = matrix(2, 1) * scale_y;

  return matrix;
}

DataItem Dataset::operator[](int idx) const {
  // TODO: Check if idx is in range

  DataItem data;
  data.set<Field::INDEX>([idx]() { return idx; });
  for (auto field : _fields) {
    switch (field) {
    case Field::COLOR: {
      auto path = _path / "rgb.mp4";
      data.set<Field::COLOR>(
          [idx, path]() { return read_frame_at_index(path, idx).value(); });
      break;
    }
    case Field::DEPTH: {
      auto depth_path = _path / "depth" / fmt::format("{:06}.png", idx);
      data.set<Field::DEPTH>([depth_path]() {
        CHECK_EXSISTANCE(depth_path);
        return read_depth_image(depth_path).value();
      });
      break;
    }
    case Field::CONFIDENCE: {
      auto conf_path = _path / "confidence" / fmt::format("{:06}.png", idx);
      data.set<Field::CONFIDENCE>([conf_path]() {
        CHECK_EXSISTANCE(conf_path);
        return read_confidence_image(conf_path).value();
      });
      break;
    }
    case Field::ODOMETRY: {
      auto row = _odometry_data.row(idx);
      data.set<Field::ODOMETRY>([row]() { return row; });
      break;
    }
    case Field::IMU: {
      auto row = _imu_data.row(idx);
      data.set<Field::IMU>([row]() { return row; });
      break;
    }
    default:
      throw std::runtime_error("Unknown field");
    }
  }

  return data;
}

} // namespace ReUseX
