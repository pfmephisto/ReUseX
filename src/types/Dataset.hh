#pragma once
#include "./Data.hh"

#include <Eigen/Dense>

#include <fmt/format.h>

#include <filesystem>
#include <future>
#include <initializer_list>
#include <set>
#include <thread>
#include <vector>

#include <sys/ioctl.h> //ioctl() and TIOCGWINSZ
#include <unistd.h>    // for STDOUT_FILENO

#if defined(SPDLOG_FMT_EXTERNAL)
#include <spdlog/fmt/fmt.h>
#include <spdlog/fmt/ranges.h>
#include <spdlog/fmt/std.h>
#else
#include <spdlog/fmt/bundled/rangers.h>
#endif

#include <spdlog/spdlog.h>

static std::string trimPathMiddle(const std::string &path, int offset = 0) {
  struct winsize size;
  ioctl(STDOUT_FILENO, TIOCGWINSZ, &size);
  int termWidth = size.ws_col;

  int maxLen = termWidth;
  maxLen = std::max(maxLen - offset, 10);

  if (path.length() <= static_cast<size_t>(maxLen)) {
    return path;
  }

  const std::string ellipsis = "[...]";
  int keepLen = maxLen - ellipsis.length();
  int front = keepLen / 2;
  int back = keepLen - front;

  return path.substr(0, front) + ellipsis + path.substr(path.length() - back);
}

namespace ReUseX {

class Dataset {
    private:
  std::filesystem::path _path = {};
  std::set<Field> _fields = {
      Field::COLOR,    Field::DEPTH, Field::CONFIDENCE,
      Field::ODOMETRY, Field::IMU,
  };
  Eigen::MatrixXd _odometry_data = {};
  Eigen::MatrixXd _imu_data = {};
  size_t _n_frames = 0;
  int _rgb_width = 1920;
  int _rgb_hight = 1440;
  int _depth_width = 256;
  int _depth_hight = 192;

    public:
  Dataset(const std::filesystem::path &path);

  Dataset(const std::string &path) : Dataset(std::filesystem::path(path)) {};

  // Getters
  std::set<Field> fields() const { return _fields; };
  Eigen::Matrix3d intrinsic_matrix() const;

  // Operators
  operator bool() const { return !_fields.empty(); };
  DataItem operator[](int idx) const;

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
  static size_t get_number_of_frames(const std::filesystem::path &path);
};

} // namespace ReUseX
