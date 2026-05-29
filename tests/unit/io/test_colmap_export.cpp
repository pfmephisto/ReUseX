// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <core/ProjectDB.hpp>
#include <core/SensorIntrinsics.hpp>
#include <io/colmap.hpp>

#include <opencv2/core.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace reusex;
using Catch::Matchers::WithinAbs;
namespace fs = std::filesystem;

namespace {

struct TempDir {
  fs::path path;
  TempDir() {
    path = fs::temp_directory_path() /
           ("test_colmap_" + std::to_string(reinterpret_cast<uintptr_t>(this)));
    fs::create_directories(path);
  }
  ~TempDir() noexcept {
    std::error_code ec;
    fs::remove_all(path, ec);
  }
};

std::array<double, 16> identity4() {
  return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
}

std::array<double, 16> from_eigen(const Eigen::Matrix4d &m) {
  std::array<double, 16> out{};
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      out[r * 4 + c] = m(r, c);
  return out;
}

cv::Mat make_image(int w, int h, cv::Vec3b fill = {100, 150, 200}) {
  cv::Mat img(h, w, CV_8UC3, fill);
  return img;
}

core::SensorIntrinsics make_intrinsics(double fx, double fy, double cx,
                                       double cy, int w, int h,
                                       std::array<double, 16> local = identity4()) {
  core::SensorIntrinsics intr;
  intr.fx = fx;
  intr.fy = fy;
  intr.cx = cx;
  intr.cy = cy;
  intr.width = w;
  intr.height = h;
  intr.local_transform = local;
  return intr;
}

void seed_sensor_frame(ProjectDB &db, int id,
                       const std::array<double, 16> &pose,
                       const core::SensorIntrinsics &intr) {
  cv::Mat color = make_image(intr.width, intr.height);
  cv::Mat depth(intr.height, intr.width, CV_16UC1, cv::Scalar(1000));
  cv::Mat conf(intr.height, intr.width, CV_8UC1, cv::Scalar(3));
  db.save_sensor_frame(id, color, depth, conf, pose, intr);
}

std::vector<std::string> read_lines(const fs::path &p) {
  std::ifstream in(p);
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(in, line))
    lines.push_back(line);
  return lines;
}

std::vector<std::string> data_lines(const fs::path &p) {
  auto lines = read_lines(p);
  std::vector<std::string> out;
  for (auto &l : lines)
    if (!l.empty() && l[0] != '#')
      out.push_back(l);
  return out;
}

} // namespace

TEST_CASE("export_colmap_scene writes the expected directory layout",
          "[io][colmap]") {
  TempDir tdb_dir;
  TempDir out_dir;
  ProjectDB db(tdb_dir.path / "p.rux");

  auto intr = make_intrinsics(500.0, 500.0, 256.0, 192.0, 512, 384);
  for (int id = 1; id <= 3; ++id) {
    auto pose = identity4();
    pose[3] = id * 0.5; // translate along X
    seed_sensor_frame(db, id, pose, intr);
  }

  reusex::io::export_colmap_scene(db, out_dir.path);

  REQUIRE(fs::exists(out_dir.path / "sparse" / "0" / "cameras.txt"));
  REQUIRE(fs::exists(out_dir.path / "sparse" / "0" / "images.txt"));
  REQUIRE(fs::exists(out_dir.path / "sparse" / "0" / "points3D.txt"));
  REQUIRE(fs::exists(out_dir.path / "images" / "00000001.jpg"));
  REQUIRE(fs::exists(out_dir.path / "images" / "00000003.jpg"));
}

TEST_CASE("identical intrinsics collapse into one camera",
          "[io][colmap]") {
  TempDir tdb_dir;
  TempDir out_dir;
  ProjectDB db(tdb_dir.path / "p.rux");

  auto intr = make_intrinsics(500.0, 500.0, 256.0, 192.0, 512, 384);
  for (int id = 1; id <= 5; ++id)
    seed_sensor_frame(db, id, identity4(), intr);

  reusex::io::export_colmap_scene(db, out_dir.path);

  auto cams = data_lines(out_dir.path / "sparse" / "0" / "cameras.txt");
  REQUIRE(cams.size() == 1);
  // First token is camera_id, second is model name.
  std::istringstream ss(cams[0]);
  int cam_id;
  std::string model;
  int w, h;
  double fx, fy, cx, cy;
  ss >> cam_id >> model >> w >> h >> fx >> fy >> cx >> cy;
  REQUIRE(model == "PINHOLE");
  REQUIRE(w == 512);
  REQUIRE(h == 384);
  REQUIRE_THAT(fx, WithinAbs(500.0, 1e-6));
  REQUIRE_THAT(cx, WithinAbs(256.0, 1e-6));
}

TEST_CASE("differing intrinsics produce distinct cameras",
          "[io][colmap]") {
  TempDir tdb_dir;
  TempDir out_dir;
  ProjectDB db(tdb_dir.path / "p.rux");

  seed_sensor_frame(db, 1, identity4(),
                    make_intrinsics(500.0, 500.0, 256.0, 192.0, 512, 384));
  seed_sensor_frame(db, 2, identity4(),
                    make_intrinsics(600.0, 600.0, 320.0, 240.0, 640, 480));

  reusex::io::export_colmap_scene(db, out_dir.path);

  auto cams = data_lines(out_dir.path / "sparse" / "0" / "cameras.txt");
  REQUIRE(cams.size() == 2);
}

TEST_CASE("pose inversion: identity local_transform → quaternion identity, "
          "translation negated",
          "[io][colmap]") {
  TempDir tdb_dir;
  TempDir out_dir;
  ProjectDB db(tdb_dir.path / "p.rux");

  // Pure translation in world: pose places camera at (1, 2, 3) with identity
  // rotation. local_transform = identity, so T_wc = T_wb = pure translation.
  // Then T_cw has R = I, t = -(1, 2, 3). Quaternion should be (1,0,0,0).
  auto intr = make_intrinsics(500.0, 500.0, 256.0, 192.0, 512, 384);
  std::array<double, 16> pose = identity4();
  pose[3] = 1.0;
  pose[7] = 2.0;
  pose[11] = 3.0;

  seed_sensor_frame(db, 42, pose, intr);
  reusex::io::export_colmap_scene(db, out_dir.path);

  auto imgs = data_lines(out_dir.path / "sparse" / "0" / "images.txt");
  // Each image: 1 header line + 1 empty 2D-feature line == 2 data slots in
  // raw lines; data_lines() also strips comments. We only filtered comments,
  // so the empty 2D-feature line is in there too — find the header line.
  REQUIRE_FALSE(imgs.empty());
  std::istringstream ss(imgs[0]);
  int img_id;
  double qw, qx, qy, qz, tx, ty, tz;
  int cam_id;
  std::string name;
  ss >> img_id >> qw >> qx >> qy >> qz >> tx >> ty >> tz >> cam_id >> name;

  REQUIRE(img_id == 42);
  REQUIRE_THAT(qw, WithinAbs(1.0, 1e-9));
  REQUIRE_THAT(qx, WithinAbs(0.0, 1e-9));
  REQUIRE_THAT(qy, WithinAbs(0.0, 1e-9));
  REQUIRE_THAT(qz, WithinAbs(0.0, 1e-9));
  REQUIRE_THAT(tx, WithinAbs(-1.0, 1e-9));
  REQUIRE_THAT(ty, WithinAbs(-2.0, 1e-9));
  REQUIRE_THAT(tz, WithinAbs(-3.0, 1e-9));
}

TEST_CASE("pose composition: pose * local_transform = camera-in-world",
          "[io][colmap]") {
  TempDir tdb_dir;
  TempDir out_dir;
  ProjectDB db(tdb_dir.path / "p.rux");

  // Build a known T_wb and T_bc; the exporter should produce T_cw such that
  // T_cw^{-1} == T_wb * T_bc.
  Eigen::AngleAxisd aa_wb(0.3, Eigen::Vector3d::UnitZ());
  Eigen::Matrix4d T_wb = Eigen::Matrix4d::Identity();
  T_wb.block<3, 3>(0, 0) = aa_wb.toRotationMatrix();
  T_wb.block<3, 1>(0, 3) = Eigen::Vector3d(0.5, -0.2, 1.0);

  Eigen::AngleAxisd aa_bc(0.7, Eigen::Vector3d::UnitX());
  Eigen::Matrix4d T_bc = Eigen::Matrix4d::Identity();
  T_bc.block<3, 3>(0, 0) = aa_bc.toRotationMatrix();
  T_bc.block<3, 1>(0, 3) = Eigen::Vector3d(0.0, 0.05, 0.0);

  Eigen::Matrix4d T_wc_expected = T_wb * T_bc;

  auto intr = make_intrinsics(500.0, 500.0, 256.0, 192.0, 512, 384,
                              from_eigen(T_bc));
  seed_sensor_frame(db, 7, from_eigen(T_wb), intr);

  reusex::io::export_colmap_scene(db, out_dir.path);

  auto imgs = data_lines(out_dir.path / "sparse" / "0" / "images.txt");
  REQUIRE_FALSE(imgs.empty());
  std::istringstream ss(imgs[0]);
  int img_id;
  double qw, qx, qy, qz, tx, ty, tz;
  int cam_id;
  std::string name;
  ss >> img_id >> qw >> qx >> qy >> qz >> tx >> ty >> tz >> cam_id >> name;

  Eigen::Quaterniond q_cw(qw, qx, qy, qz);
  Eigen::Matrix4d T_cw = Eigen::Matrix4d::Identity();
  T_cw.block<3, 3>(0, 0) = q_cw.toRotationMatrix();
  T_cw.block<3, 1>(0, 3) = Eigen::Vector3d(tx, ty, tz);

  Eigen::Matrix4d T_wc_round_trip = T_cw.inverse();

  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      REQUIRE_THAT(T_wc_round_trip(r, c),
                   WithinAbs(T_wc_expected(r, c), 1e-6));
}

TEST_CASE("LiDAR seed populates points3D.txt with sub-sampling",
          "[io][colmap]") {
  TempDir tdb_dir;
  TempDir out_dir;
  ProjectDB db(tdb_dir.path / "p.rux");

  seed_sensor_frame(db, 1, identity4(),
                    make_intrinsics(500.0, 500.0, 256.0, 192.0, 512, 384));

  // Build a 1000-point synthetic cloud.
  auto cloud = std::make_shared<Cloud>();
  cloud->resize(1000);
  for (std::size_t i = 0; i < 1000; ++i) {
    (*cloud)[i].x = static_cast<float>(i) * 0.01f;
    (*cloud)[i].y = 0.0f;
    (*cloud)[i].z = 0.0f;
    (*cloud)[i].r = (*cloud)[i].g = (*cloud)[i].b = 128;
    (*cloud)[i].a = 255;
  }
  db.save_point_cloud("cloud", *cloud);

  reusex::io::ColmapExportOptions opt;
  opt.include_lidar_points = true;
  opt.max_lidar_points = 100;
  reusex::io::export_colmap_scene(db, out_dir.path, opt);

  auto pts = data_lines(out_dir.path / "sparse" / "0" / "points3D.txt");
  // Stride = ceil(1000/100) = 10 → exactly 100 points.
  REQUIRE(pts.size() == 100);
}

TEST_CASE("disabling lidar seed produces an empty points3D.txt",
          "[io][colmap]") {
  TempDir tdb_dir;
  TempDir out_dir;
  ProjectDB db(tdb_dir.path / "p.rux");

  seed_sensor_frame(db, 1, identity4(),
                    make_intrinsics(500.0, 500.0, 256.0, 192.0, 512, 384));
  auto cloud = std::make_shared<Cloud>();
  cloud->resize(10);
  db.save_point_cloud("cloud", *cloud);

  reusex::io::ColmapExportOptions opt;
  opt.include_lidar_points = false;
  reusex::io::export_colmap_scene(db, out_dir.path, opt);

  auto pts = data_lines(out_dir.path / "sparse" / "0" / "points3D.txt");
  REQUIRE(pts.empty());
}

TEST_CASE("empty ProjectDB throws", "[io][colmap]") {
  TempDir tdb_dir;
  TempDir out_dir;
  ProjectDB db(tdb_dir.path / "p.rux");

  REQUIRE_THROWS_AS(reusex::io::export_colmap_scene(db, out_dir.path),
                    std::runtime_error);
}
