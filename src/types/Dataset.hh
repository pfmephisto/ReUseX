#pragma once
#include "./Data.hh"

#include <Eigen/Dense>

#include <fmt/format.h>

#include <filesystem>
#include <future>
#include <initializer_list>
#include <set>
#include <vector>

namespace ReUseX {

class Dataset {
private:
  std::filesystem::path _path = {};
  std::set<Field> _fields = {};
  std::vector<std::filesystem::path> _depth_paths = {};
  std::vector<std::filesystem::path> _confidence_paths = {};
  Eigen::MatrixXd _odometry_data = {};
  Eigen::MatrixXd _imu_data = {};
  size_t _n_frames = 0;
  int _rgb_width = 1920;
  int _rgb_hight = 1440;
  int _depth_width = 256;
  int _depth_hight = 192;
  std::shared_future<void> _asyncConstructor;

public:
  template <typename T>
  Dataset(const std::filesystem::path &path, T begin, T end) {
    // Check if directories and files exists
    assert(std::filesystem::exists(path) &&
           fmt::format("Directory does not exist: {}", path.string()).c_str());
    _path = path;

    _n_frames = get_number_of_frames(_path / "rgb.mp4");

    // TODO: Rather implement this as lazy loading.
    //_asyncConstructor = std::async(std::launch::async, [&]() {
    //      fmt::print("Inside async..");
    //	std::this_thread::sleep_for(std::chrono::seconds(2));
    //});

    // Load data
    for (auto it = begin; it != end; it++) {
      Field field = *it;
      set_field(field);
    };
  };

  Dataset(const std::filesystem::path &path,
          const std::initializer_list<Field> &fields)
      : Dataset(path, fields.begin(), fields.end()) {};

  Dataset(const std::string &path)
      : Dataset(std::filesystem::path(path),
                {Field::COLOR, Field::DEPTH, Field::CONFIDENCE, Field::ODOMETRY,
                 Field::IMU, Field::POSES}) {};

  // Getters
  std::set<Field> fields() const { return _fields; };
  Eigen::Matrix3d intrinsic_matrix() const;

  // Operators
  operator bool() const { return !_fields.empty(); };
  DataItem operator[](int idx) const;
  void display(std::string name, bool show = true) const;

  inline size_t size() const { return _n_frames; };
  inline cv::Size color_size() const {
    return cv::Size(_rgb_width, _rgb_hight);
  };
  inline cv::Size depth_size() const {
    return cv::Size(_depth_width, _depth_hight);
  };
  inline std::string name() const { return _path.parent_path().filename(); };

  inline auto get_odometry() const { return _odometry_data; };

private:
  void set_field(Field field);
  static size_t get_number_of_frames(const std::filesystem::path &path);
};

} // namespace ReUseX
