// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "io/colmap.hpp"
#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <tuple>
#include <unordered_map>

namespace reusex::io {

namespace {

constexpr double kEps = 1e-9;

/// Build a row-major double[16] → Eigen::Matrix4d (no SE(3) assumptions).
Eigen::Matrix4d to_matrix4d(const std::array<double, 16> &m) {
  Eigen::Matrix4d out;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      out(r, c) = m[r * 4 + c];
  return out;
}

/// Hash key for grouping cameras by identical intrinsics. Pose is excluded;
/// only the static optical model matters.
struct IntrinsicsKey {
  int width;
  int height;
  // Quantize floating-point fields to avoid jitter creating spurious cameras.
  int64_t fx_q, fy_q, cx_q, cy_q;
};
bool operator==(const IntrinsicsKey &a, const IntrinsicsKey &b) {
  return a.width == b.width && a.height == b.height && a.fx_q == b.fx_q &&
         a.fy_q == b.fy_q && a.cx_q == b.cx_q && a.cy_q == b.cy_q;
}
struct IntrinsicsKeyHash {
  std::size_t operator()(const IntrinsicsKey &k) const noexcept {
    auto mix = [](std::size_t h, std::size_t v) {
      return h ^ (v + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2));
    };
    std::size_t h = std::hash<int>{}(k.width);
    h = mix(h, std::hash<int>{}(k.height));
    h = mix(h, std::hash<int64_t>{}(k.fx_q));
    h = mix(h, std::hash<int64_t>{}(k.fy_q));
    h = mix(h, std::hash<int64_t>{}(k.cx_q));
    h = mix(h, std::hash<int64_t>{}(k.cy_q));
    return h;
  }
};

IntrinsicsKey to_key(const core::SensorIntrinsics &i) {
  // 1e-3 px resolution is plenty for grouping; well below intrinsics noise.
  auto q = [](double v) -> int64_t {
    return static_cast<int64_t>(std::llround(v * 1000.0));
  };
  return IntrinsicsKey{i.width, i.height, q(i.fx), q(i.fy), q(i.cx), q(i.cy)};
}

/// Compose `T_wc = T_wb * T_bc`, then invert to `T_cw`, then split into
/// (quaternion (qw,qx,qy,qz), translation (tx,ty,tz)) as COLMAP expects.
struct ColmapPose {
  double qw, qx, qy, qz;
  double tx, ty, tz;
};
ColmapPose to_colmap_pose(const std::array<double, 16> &worldPose,
                          const std::array<double, 16> &localTransform) {
  Eigen::Matrix4d T_wb = to_matrix4d(worldPose);
  Eigen::Matrix4d T_bc = to_matrix4d(localTransform);
  Eigen::Matrix4d T_wc = T_wb * T_bc;
  // SE(3): inverse is [R^T  -R^T t; 0 1]
  Eigen::Matrix3d R_wc = T_wc.block<3, 3>(0, 0);
  Eigen::Vector3d t_wc = T_wc.block<3, 1>(0, 3);
  Eigen::Matrix3d R_cw = R_wc.transpose();
  Eigen::Vector3d t_cw = -R_cw * t_wc;

  Eigen::Quaterniond q(R_cw);
  q.normalize();
  // COLMAP convention: positive scalar part.
  if (q.w() < 0.0) {
    q.coeffs() *= -1.0;
  }
  return ColmapPose{q.w(), q.x(), q.y(), q.z(), t_cw.x(), t_cw.y(), t_cw.z()};
}

void write_cameras_txt(const std::filesystem::path &path,
                       const std::vector<std::tuple<int, core::SensorIntrinsics>>
                           &cameras_by_id) {
  std::ofstream out(path);
  if (!out)
    throw std::runtime_error("Failed to open " + path.string() +
                             " for writing");
  out << "# Camera list with one line of data per camera:\n";
  out << "#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n";
  out << "# Number of cameras: " << cameras_by_id.size() << "\n";
  for (const auto &[cam_id, intr] : cameras_by_id) {
    out << cam_id << " PINHOLE " << intr.width << " " << intr.height << " "
        << fmt::format("{:.10g} {:.10g} {:.10g} {:.10g}", intr.fx, intr.fy,
                       intr.cx, intr.cy)
        << "\n";
  }
}

struct FrameRecord {
  int image_id;
  int camera_id;
  ColmapPose pose;
  std::string image_name;
};

void write_images_txt(const std::filesystem::path &path,
                      const std::vector<FrameRecord> &frames) {
  std::ofstream out(path);
  if (!out)
    throw std::runtime_error("Failed to open " + path.string() +
                             " for writing");
  out << "# Image list with two lines of data per image:\n";
  out << "#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n";
  out << "#   POINTS2D[] as (X, Y, POINT3D_ID)\n";
  out << "# Number of images: " << frames.size() << "\n";
  for (const auto &f : frames) {
    out << f.image_id << " "
        << fmt::format("{:.17g} {:.17g} {:.17g} {:.17g} ", f.pose.qw,
                       f.pose.qx, f.pose.qy, f.pose.qz)
        << fmt::format("{:.10g} {:.10g} {:.10g} ", f.pose.tx, f.pose.ty,
                       f.pose.tz)
        << f.camera_id << " " << f.image_name << "\n";
    // No 2D feature observations — required blank line per COLMAP format.
    out << "\n";
  }
}

void write_points3D_txt(const std::filesystem::path &path,
                        const ProjectDB &db,
                        const ColmapExportOptions &opt) {
  std::ofstream out(path);
  if (!out)
    throw std::runtime_error("Failed to open " + path.string() +
                             " for writing");
  out << "# 3D point list with one line of data per point:\n";
  out << "#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, "
         "POINT2D_IDX)\n";

  std::size_t written = 0;
  if (opt.include_lidar_points && db.has_point_cloud(opt.lidar_cloud_name)) {
    // Loading a seed cloud is best-effort: a missing chunk column, type
    // mismatch, or any other read failure should not abort the COLMAP
    // export — points3D.txt may legitimately be empty.
    reusex::CloudPtr cloud;
    try {
      cloud = db.point_cloud_xyzrgb(opt.lidar_cloud_name);
    } catch (const std::exception &e) {
      core::warn("Could not load LiDAR seed cloud '{}': {} — writing empty "
                 "points3D.txt",
                 opt.lidar_cloud_name, e.what());
    }
    if (cloud && !cloud->empty()) {
      const std::size_t total = cloud->size();
      // Uniform stride so coverage is preserved across the whole cloud.
      const std::size_t stride =
          (opt.max_lidar_points == 0 || total <= opt.max_lidar_points)
              ? 1
              : (total + opt.max_lidar_points - 1) / opt.max_lidar_points;
      out << "# Number of points: ~" << (total / stride) << "\n";
      for (std::size_t i = 0; i < total; i += stride) {
        const auto &p = (*cloud)[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
          continue;
        // POINT3D_ID is 1-based; track is empty (no 2D observations).
        out << (written + 1) << " " << fmt::format("{:.6g}", p.x) << " "
            << fmt::format("{:.6g}", p.y) << " " << fmt::format("{:.6g}", p.z)
            << " " << static_cast<int>(p.r) << " " << static_cast<int>(p.g)
            << " " << static_cast<int>(p.b) << " 0.0\n";
        ++written;
      }
    }
  }
  if (written == 0)
    out << "# Number of points: 0\n";
  core::info("Wrote {} seed points3D entries", written);
}

} // anonymous namespace

void export_colmap_scene(const ProjectDB &db,
                         const std::filesystem::path &out_dir,
                         const ColmapExportOptions &opt) {
  namespace fs = std::filesystem;

  auto frame_ids = db.sensor_frame_ids();
  if (frame_ids.empty())
    throw std::runtime_error(
        "export_colmap_scene: ProjectDB has no sensor frames");

  core::info("Exporting COLMAP scene to {} ({} frames)", out_dir.string(),
             frame_ids.size());

  const fs::path images_dir = out_dir / "images";
  const fs::path sparse_dir = out_dir / "sparse" / "0";
  fs::create_directories(images_dir);
  fs::create_directories(sparse_dir);

  // Camera grouping: identical intrinsics share a camera_id.
  std::unordered_map<IntrinsicsKey, int, IntrinsicsKeyHash> camera_ids;
  std::vector<std::tuple<int, core::SensorIntrinsics>> cameras_in_order;
  int next_cam_id = 1;

  std::vector<FrameRecord> frame_records;
  frame_records.reserve(frame_ids.size());

  for (int node_id : frame_ids) {
    if (!db.has_sensor_frame(node_id))
      continue;

    cv::Mat img = db.sensor_frame_image(node_id);
    if (img.empty()) {
      core::debug("Node {} has no color image, skipping", node_id);
      continue;
    }

    auto intr = db.sensor_frame_intrinsics(node_id);
    if (intr.width <= 0 || intr.height <= 0 || intr.fx <= kEps ||
        intr.fy <= kEps) {
      core::warn("Node {} has invalid intrinsics, skipping", node_id);
      continue;
    }
    // COLMAP requires camera dimensions to match the image. ProjectDB stores
    // RGB at original RTABMap resolution alongside intrinsics, but if the two
    // disagree we correct the camera to the actual image we are writing.
    if (img.cols != intr.width || img.rows != intr.height) {
      const double sx = static_cast<double>(img.cols) / intr.width;
      const double sy = static_cast<double>(img.rows) / intr.height;
      intr.fx *= sx;
      intr.fy *= sy;
      intr.cx *= sx;
      intr.cy *= sy;
      intr.width = img.cols;
      intr.height = img.rows;
    }

    IntrinsicsKey key = to_key(intr);
    int cam_id;
    if (auto it = camera_ids.find(key); it != camera_ids.end()) {
      cam_id = it->second;
    } else {
      cam_id = next_cam_id++;
      camera_ids.emplace(key, cam_id);
      cameras_in_order.emplace_back(cam_id, intr);
    }

    std::array<double, 16> pose = db.sensor_frame_pose(node_id);
    ColmapPose cpose = to_colmap_pose(pose, intr.local_transform);

    const std::string image_name = fmt::format("{:08d}.jpg", node_id);
    const fs::path image_path = images_dir / image_name;
    std::vector<int> jpeg_params{cv::IMWRITE_JPEG_QUALITY, opt.jpeg_quality};
    if (!cv::imwrite(image_path.string(), img, jpeg_params))
      throw std::runtime_error("Failed to write " + image_path.string());

    frame_records.push_back(
        FrameRecord{node_id, cam_id, cpose, image_name});
  }

  if (frame_records.empty())
    throw std::runtime_error(
        "export_colmap_scene: no usable sensor frames after filtering");

  write_cameras_txt(sparse_dir / "cameras.txt", cameras_in_order);
  write_images_txt(sparse_dir / "images.txt", frame_records);
  write_points3D_txt(sparse_dir / "points3D.txt", db, opt);

  core::info("COLMAP export complete: {} images, {} cameras",
             frame_records.size(), cameras_in_order.size());
}

} // namespace reusex::io
