// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later
//
// End-to-end test of the OpenMVS-backed densify pipeline on a small
// synthetic scene with known geometry.
//
// We ray-cast a textured plane from a handful of poses we control, feed
// the resulting images + poses + intrinsics through ProjectDB, run
// densify_from_images, and assert that the reconstructed cloud lies
// close to the known plane. Catches regressions in pose convention,
// K matrix scaling, single-platform topology, etc. — anything that
// silently produces a "ball of noise" cloud on real data.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <core/ProjectDB.hpp>
#include <core/SensorIntrinsics.hpp>
#include <geometry/densify.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <random>

using namespace reusex;
namespace fs = std::filesystem;

namespace {

struct TempDir {
  fs::path path;
  TempDir() {
    path = fs::temp_directory_path() /
           ("test_densify_" +
            std::to_string(reinterpret_cast<uintptr_t>(this)));
    fs::create_directories(path);
  }
  ~TempDir() noexcept {
    std::error_code ec;
    fs::remove_all(path, ec);
  }
};

std::array<double, 16> mat4_to_row_major(const Eigen::Matrix4d &m) {
  std::array<double, 16> out{};
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      out[r * 4 + c] = m(r, c);
  return out;
}

/// Build a camera-to-world transform (T_wc) for a camera at position
/// `eye` looking at `target`, with `up` indicating the world up axis.
/// OpenCV convention: camera X=right, Y=down, Z=forward.
Eigen::Matrix4d look_at_camera(const Eigen::Vector3d &eye,
                               const Eigen::Vector3d &target,
                               const Eigen::Vector3d &up_world) {
  Eigen::Vector3d forward = (target - eye).normalized();      // +Z
  Eigen::Vector3d right = forward.cross(up_world).normalized(); // +X
  Eigen::Vector3d down = forward.cross(right).normalized();    // +Y
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.col(0).head<3>() = right;
  T.col(1).head<3>() = down;
  T.col(2).head<3>() = forward;
  T.col(3).head<3>() = eye;
  return T;
}

/// Deterministic RGB noise texture sampled on a regular grid. Index by
/// world-space (x, y) coordinates so all cameras see consistent colors.
cv::Vec3b sample_texture(double world_x, double world_y, double cell_size) {
  const int64_t cx = static_cast<int64_t>(std::floor(world_x / cell_size));
  const int64_t cy = static_cast<int64_t>(std::floor(world_y / cell_size));
  std::seed_seq seed{static_cast<uint32_t>(cx & 0xFFFFFFFF),
                     static_cast<uint32_t>(cy & 0xFFFFFFFF), 0x9E3779B9u};
  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> dist(40, 240);
  return cv::Vec3b(static_cast<uchar>(dist(rng)),
                   static_cast<uchar>(dist(rng)),
                   static_cast<uchar>(dist(rng)));
}

/// Render an image of a textured plane at world `z = z_plane`, viewed
/// through the given camera. R_wc maps camera-frame points to world;
/// C_w is the camera center in world coords. Pixels for which the ray
/// misses the plane (or hits behind the camera) get a flat dark color.
cv::Mat render_plane(const Eigen::Matrix3d &R_wc, const Eigen::Vector3d &C_w,
                     double fx, double fy, double cx, double cy, int w, int h,
                     double z_plane, double cell_size) {
  cv::Mat img(h, w, CV_8UC3);
  for (int v = 0; v < h; ++v) {
    auto *row = img.ptr<cv::Vec3b>(v);
    for (int u = 0; u < w; ++u) {
      Eigen::Vector3d d_cam((u - cx) / fx, (v - cy) / fy, 1.0);
      Eigen::Vector3d d_w = R_wc * d_cam;
      if (std::abs(d_w.z()) < 1e-6) {
        row[u] = cv::Vec3b(30, 30, 30);
        continue;
      }
      const double t = (z_plane - C_w.z()) / d_w.z();
      if (t <= 0) {
        row[u] = cv::Vec3b(30, 30, 30);
        continue;
      }
      const Eigen::Vector3d hit = C_w + t * d_w;
      row[u] = sample_texture(hit.x(), hit.y(), cell_size);
    }
  }
  return img;
}

/// Plant @p num_cameras synthetic frames around a circle of radius
/// `circle_radius`, all looking at the centre of the textured plane at
/// z=z_plane. We compute the desired camera-in-world pose T_wc, then
/// store a body-in-world pose T_wb and a non-trivial body-to-camera
/// offset T_bc = `local_transform` such that T_wb * T_bc = T_wc. This
/// mirrors the RTABMap convention where ProjectDB stores the *body*
/// pose and intrinsics carry the camera offset — densify must compose
/// the two correctly to put the camera at T_wc.
struct SceneConfig {
  int num_cameras = 8;
  int img_w = 320;
  int img_h = 240;
  double fx = 250.0;
  double z_plane = 5.0;
  double circle_radius = 1.0;
  double cell_size = 0.15;
  /// Body→camera offset applied during scene construction. Non-identity
  /// means the body and camera frames differ — exactly what we want to
  /// stress-test the composition code path. Use identity4() to disable.
  Eigen::Matrix4d local_transform = Eigen::Matrix4d::Identity();
};

void plant_synthetic_scene(ProjectDB &db, const SceneConfig &cfg) {
  core::SensorIntrinsics intr;
  intr.fx = cfg.fx;
  intr.fy = cfg.fx;
  intr.cx = (cfg.img_w - 1) / 2.0;
  intr.cy = (cfg.img_h - 1) / 2.0;
  intr.width = cfg.img_w;
  intr.height = cfg.img_h;
  intr.local_transform = mat4_to_row_major(cfg.local_transform);

  const Eigen::Matrix4d T_bc = cfg.local_transform;
  const Eigen::Matrix4d T_cb = T_bc.inverse();

  const Eigen::Vector3d target(0.0, 0.0, cfg.z_plane);
  const Eigen::Vector3d up(0.0, -1.0, 0.0);

  for (int i = 0; i < cfg.num_cameras; ++i) {
    const double angle =
        (2.0 * M_PI * i) / static_cast<double>(cfg.num_cameras);
    const Eigen::Vector3d eye(cfg.circle_radius * std::cos(angle),
                              cfg.circle_radius * std::sin(angle), 0.0);

    // The CAMERA looks at the plane from `eye`. Render uses this.
    const Eigen::Matrix4d T_wc = look_at_camera(eye, target, up);
    const Eigen::Matrix3d R_wc = T_wc.block<3, 3>(0, 0);
    const Eigen::Vector3d C_w = T_wc.block<3, 1>(0, 3);

    cv::Mat color = render_plane(R_wc, C_w, intr.fx, intr.fy, intr.cx, intr.cy,
                                 cfg.img_w, cfg.img_h, cfg.z_plane,
                                 cfg.cell_size);

    // ProjectDB stores T_wb (body in world), not T_wc. Compute the body
    // pose that, when composed with T_bc (local_transform), recovers
    // T_wc:  T_wb = T_wc * T_cb. If densify gets the composition right,
    // it will reconstruct the same T_wc inside OpenMVS.
    const Eigen::Matrix4d T_wb = T_wc * T_cb;

    cv::Mat depth(cfg.img_h, cfg.img_w, CV_16UC1, cv::Scalar(0));
    cv::Mat confidence(cfg.img_h, cfg.img_w, CV_8UC1, cv::Scalar(2));

    db.save_sensor_frame(i + 1, color, depth, confidence,
                         mat4_to_row_major(T_wb), intr);
  }
}

} // namespace

/// Run the synthetic densify against the provided scene config and
/// assert the reconstructed cloud sits on the ground-truth plane.
void run_and_check(const SceneConfig &cfg) {
  TempDir tdb_dir;
  ProjectDB db(tdb_dir.path / "synthetic.rux");
  plant_synthetic_scene(db, cfg);

  reusex::geometry::DensifyParams params;
  params.seed_cloud_name = "";   // no LiDAR seed in synthetic scene
  params.chunk_size = 0;         // single-pass fusion
  params.gpu_index = -2;         // force CPU PatchMatch (Nix sandbox)
  params.max_threads = 4;        // bound build-time concurrency
  params.resolution_level = 0;   // full resolution
  params.max_resolution = 0;     // no cap
  params.geometric_consistency = true;

  reusex::geometry::densify_from_images(db, params);

  REQUIRE(db.has_point_cloud("dense"));
  auto cloud = db.point_cloud_xyzrgb("dense");
  REQUIRE(cloud);
  REQUIRE(cloud->size() > 1000);

  std::size_t on_plane = 0;
  double sum_z = 0.0;
  for (const auto &pt : *cloud) {
    sum_z += pt.z;
    if (std::abs(pt.z - cfg.z_plane) < 0.3)
      ++on_plane;
  }
  const double mean_z = sum_z / cloud->size();
  INFO("Reconstructed " << cloud->size() << " points, mean Z = " << mean_z
                        << " (expected " << cfg.z_plane << ")");
  REQUIRE(on_plane > cloud->size() / 2);
  REQUIRE(std::abs(mean_z - cfg.z_plane) < 0.3);
}

TEST_CASE("densify_from_images reconstructs a textured plane",
          "[densify][synthetic][slow]") {
  SECTION("identity local_transform (body == camera)") {
    SceneConfig cfg;
    cfg.local_transform = Eigen::Matrix4d::Identity();
    run_and_check(cfg);
  }

  SECTION("non-identity local_transform (RTABMap-style body→camera offset)") {
    // Synthesise a meaningful body→camera offset:
    //  - body sits 30 cm behind the camera along the optical axis
    //  - 7° pitch difference between body forward and camera forward
    // This is the scenario densify needs to compose correctly: the
    // ProjectDB-stored pose is the BODY pose, and intrinsics carry the
    // T_bc offset. If the composition is wrong, the reconstructed
    // plane lands off-axis or at the wrong depth and the asserts fail.
    Eigen::AngleAxisd pitch(7.0 * M_PI / 180.0, Eigen::Vector3d::UnitX());
    Eigen::Matrix4d T_bc = Eigen::Matrix4d::Identity();
    T_bc.block<3, 3>(0, 0) = pitch.toRotationMatrix();
    T_bc.block<3, 1>(0, 3) = Eigen::Vector3d(0.05, -0.02, 0.30);

    SceneConfig cfg;
    cfg.local_transform = T_bc;
    run_and_check(cfg);
  }
}
