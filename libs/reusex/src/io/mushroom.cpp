// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "io/mushroom.hpp"
#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"

#include <nlohmann/json.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <fstream>
#include <regex>
#include <stdexcept>
#include <string>

namespace reusex::io {

namespace {

/// Read a row-major nested-array matrix of size rows x cols.
template <int Rows, int Cols>
Eigen::Matrix<double, Rows, Cols> read_matrix(const nlohmann::json &m) {
  Eigen::Matrix<double, Rows, Cols> T;
  for (int r = 0; r < Rows; ++r)
    for (int c = 0; c < Cols; ++c)
      T(r, c) = m.at(r).at(c).get<double>();
  return T;
}

/// Minimal NumPy .npy reader for 2-D little-endian float32 C-order arrays —
/// the format MuSHRoom stores sensor depth in. Supports header versions 1-3.
cv::Mat load_npy_f32(const std::filesystem::path &path) {
  std::ifstream in(path, std::ios::binary);
  if (!in)
    throw std::runtime_error(
        fmt::format("load_npy_f32: cannot open {}", path.string()));

  char magic[6];
  in.read(magic, 6);
  if (std::string_view(magic, 6) != "\x93NUMPY")
    throw std::runtime_error(
        fmt::format("load_npy_f32: {} is not a .npy file", path.string()));

  uint8_t major = 0;
  uint8_t minor = 0;
  in.read(reinterpret_cast<char *>(&major), 1);
  in.read(reinterpret_cast<char *>(&minor), 1);

  uint32_t header_len = 0;
  if (major == 1) {
    uint16_t len16 = 0;
    in.read(reinterpret_cast<char *>(&len16), 2);
    header_len = len16;
  } else {
    in.read(reinterpret_cast<char *>(&header_len), 4);
  }

  std::string header(header_len, '\0');
  in.read(header.data(), header_len);

  if (header.find("'<f4'") == std::string::npos)
    throw std::runtime_error(fmt::format(
        "load_npy_f32: {} is not little-endian float32", path.string()));
  if (header.find("'fortran_order': False") == std::string::npos)
    throw std::runtime_error(
        fmt::format("load_npy_f32: {} is not C-order", path.string()));

  const std::regex shape_re(R"('shape':\s*\((\d+),\s*(\d+)\))");
  std::smatch match;
  if (!std::regex_search(header, match, shape_re))
    throw std::runtime_error(fmt::format(
        "load_npy_f32: cannot parse 2-D shape from {}", path.string()));
  const int rows = std::stoi(match[1]);
  const int cols = std::stoi(match[2]);

  cv::Mat depth(rows, cols, CV_32FC1);
  in.read(reinterpret_cast<char *>(depth.data),
          static_cast<std::streamsize>(sizeof(float)) * rows * cols);
  if (!in)
    throw std::runtime_error(
        fmt::format("load_npy_f32: truncated data in {}", path.string()));
  return depth;
}

/// Locate the frame directory: either @p dir itself holds meta_data.json or
/// a known MuSHRoom subdirectory does (long_capture layout).
std::filesystem::path resolve_frames_dir(const std::filesystem::path &dir) {
  if (std::filesystem::exists(dir / "meta_data.json"))
    return dir;
  for (const char *sub :
       {"sdf_dataset_all_interp_4", "sdf_dataset_train_interp_4"}) {
    if (std::filesystem::exists(dir / sub / "meta_data.json"))
      return dir / sub;
  }
  throw std::runtime_error(fmt::format(
      "import_mushroom: no meta_data.json under {} (expected the capture "
      "dir or its sdf_dataset_*_interp_4 subdirectory)",
      dir.string()));
}

} // namespace

std::size_t import_mushroom(ProjectDB &db,
                            const std::filesystem::path &capture_dir) {
  const auto frames_dir = resolve_frames_dir(capture_dir);
  const auto json_path = frames_dir / "meta_data.json";

  std::ifstream in(json_path);
  nlohmann::json meta;
  in >> meta;

  if (!meta.contains("frames") || meta.at("frames").empty())
    throw std::runtime_error(fmt::format(
        "import_mushroom: no frames listed in {}", json_path.string()));

  const std::string camera_model =
      meta.value("camera_model", std::string{"OPENCV"});
  if (camera_model != "OPENCV")
    reusex::warn("import_mushroom: camera_model '{}' (expected OPENCV); "
                 "poses are imported without axis conversion",
                 camera_model);

  // MuSHRoom's sdfstudio-style export normalizes the scene into a unit box.
  // worldtogt is a similarity transform (uniform scale s + translation) back
  // to the metric ground-truth-mesh frame. Applying it naively would bake
  // the scale into the rotation block, so decompose: R stays orthonormal,
  // t_metric = s * t_normalized + t_gt. Depth values scale by s alone.
  const Eigen::Matrix4d worldtogt = read_matrix<4, 4>(meta.at("worldtogt"));
  const double scale = worldtogt.block<3, 3>(0, 0).col(0).norm();
  const Eigen::Vector3d t_gt = worldtogt.block<3, 1>(0, 3);
  if (scale <= 0.0)
    throw std::runtime_error("import_mushroom: degenerate worldtogt scale");

  std::size_t imported = 0;
  int node_id = 1;
  for (const auto &frame : meta.at("frames")) {
    const auto rgb_path = frames_dir / frame.at("rgb_path").get<std::string>();
    const auto depth_path =
        frames_dir / frame.at("sensor_depth_path").get<std::string>();

    const cv::Mat color = cv::imread(rgb_path.string(), cv::IMREAD_COLOR);
    if (color.empty()) {
      reusex::warn("import_mushroom: skipping unreadable RGB {}",
                   rgb_path.string());
      continue;
    }

    // Depth: normalized float32 .npy -> metric meters -> CV_16UC1 mm.
    cv::Mat depth_norm = load_npy_f32(depth_path);
    cv::Mat depth_mm;
    depth_norm.convertTo(depth_mm, CV_16UC1, scale * 1000.0);

    // Intrinsics: per-frame 3x3 pixel matrix.
    const Eigen::Matrix3d K = read_matrix<3, 3>(frame.at("intrinsics"));
    core::SensorIntrinsics intr;
    intr.fx = K(0, 0);
    intr.fy = K(1, 1);
    intr.cx = K(0, 2);
    intr.cy = K(1, 2);
    intr.width = color.cols;
    intr.height = color.rows;

    // Pose: camtoworld is OpenCV-convention camera-to-world in normalized
    // coordinates; convert to the metric GT frame per the decomposition
    // above. Stored world pose is optical-to-world (identity local).
    const Eigen::Matrix4d c2w_norm = read_matrix<4, 4>(frame.at("camtoworld"));
    Eigen::Matrix4d c2w = Eigen::Matrix4d::Identity();
    c2w.block<3, 3>(0, 0) = c2w_norm.block<3, 3>(0, 0);
    c2w.block<3, 1>(0, 3) = scale * c2w_norm.block<3, 1>(0, 3) + t_gt;

    std::array<double, 16> pose{};
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        pose[static_cast<std::size_t>(r * 4 + c)] = c2w(r, c);

    // MuSHRoom provides no confidence channel; treat all pixels as fully
    // confident (ReUseX convention: CV_8UC1, higher = better).
    const cv::Mat confidence(depth_mm.rows, depth_mm.cols, CV_8UC1,
                             cv::Scalar(2));

    db.save_sensor_frame(node_id, color, depth_mm, confidence, pose, intr);
    ++imported;
    ++node_id;
  }

  if (imported == 0)
    throw std::runtime_error(
        fmt::format("import_mushroom: no frames could be imported from {}",
                    frames_dir.string()));

  reusex::info("import_mushroom: imported {} sensor frames from {} "
               "(metric scale {:.4f}, GT-mesh frame)",
               imported, frames_dir.string(), scale);
  return imported;
}

} // namespace reusex::io
