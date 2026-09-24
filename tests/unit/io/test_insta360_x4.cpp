// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/insta360_x4.hpp>
#include <reusex/io/panoramas.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "../../support/temp_path.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;
using reusex::io::import_panoramas;
using reusex::io::ImportPanoramasOptions;
using reusex::io::is_insta360_dual_fisheye;
using reusex::io::stitch_insta360_x4;
using reusex::test_support::TempDir;

namespace {

// Reference input size (native X4 resolution).
constexpr int kRefW = 5888;
constexpr int kRefH = 2944;

// Create a synthetic dual-fisheye image: solid colour per half so we can
// verify that the correct half maps to the correct equirect region.
cv::Mat make_dual_fisheye(int w, int h,
                          cv::Scalar front_colour, // left half
                          cv::Scalar back_colour)  // right half
{
  cv::Mat img(h, w, CV_8UC3);
  img(cv::Rect(0, 0, w / 2, h)).setTo(front_colour);
  img(cv::Rect(w / 2, 0, w / 2, h)).setTo(back_colour);
  return img;
}

// Write a minimal JPEG to path (cv::imencode handles the encoding).
void write_jpeg(const fs::path &path, int rows, int cols,
                cv::Scalar colour = cv::Scalar(128, 64, 200)) {
  cv::Mat img(rows, cols, CV_8UC3, colour);
  std::vector<uint8_t> buf;
  cv::imencode(".jpg", img, buf);
  std::ofstream f(path, std::ios::binary);
  f.write(reinterpret_cast<const char *>(buf.data()),
          static_cast<std::streamsize>(buf.size()));
}

void write_insp(const fs::path &path, int w = kRefW, int h = kRefH) {
  cv::Mat img(h, w, CV_8UC3, cv::Scalar(100, 150, 200));
  std::vector<uint8_t> buf;
  cv::imencode(".jpg", img, buf);
  std::ofstream f(path, std::ios::binary);
  f.write(reinterpret_cast<const char *>(buf.data()),
          static_cast<std::streamsize>(buf.size()));
}

} // anonymous namespace

// ─── is_insta360_dual_fisheye ────────────────────────────────────────────────

TEST_CASE("IsDualFisheye_InspExtension_ReturnsTrue", "[io][insta360]") {
  CHECK(is_insta360_dual_fisheye("IMG_0001.insp"));
  CHECK(is_insta360_dual_fisheye("IMG_0001.INSP"));
  CHECK(is_insta360_dual_fisheye("/some/dir/capture.insp"));
}

TEST_CASE("IsDualFisheye_JpgExtension_ReturnsFalse", "[io][insta360]") {
  CHECK_FALSE(is_insta360_dual_fisheye("pano.jpg"));
  CHECK_FALSE(is_insta360_dual_fisheye("pano.jpeg"));
  CHECK_FALSE(is_insta360_dual_fisheye("pano.png"));
  CHECK_FALSE(is_insta360_dual_fisheye(""));
}

// ─── stitch_insta360_x4: dimension contract ──────────────────────────────────

TEST_CASE("StitchX4_ValidInput_OutputIs2to1", "[io][insta360]") {
  // Use a small downscale (divide by 8) to keep the test fast.
  const int w = kRefW / 8, h = kRefH / 8;
  const cv::Mat dual(h, w, CV_8UC3, cv::Scalar(128, 128, 128));
  const cv::Mat result = stitch_insta360_x4(dual);

  CHECK(result.cols == 2 * result.rows);
  CHECK(result.channels() == 3);
  CHECK(result.type() == CV_8UC3);
}

TEST_CASE("StitchX4_FullResolution_OutputIs2to1", "[io][insta360]") {
  const cv::Mat dual(kRefH, kRefW, CV_8UC3, cv::Scalar(80, 160, 240));
  const cv::Mat result = stitch_insta360_x4(dual);

  // Width must be exactly 2× height.
  REQUIRE(result.cols == 2 * result.rows);
  // Output height should be close to the reference 2711 (may differ by 1
  // due to rounding when scaling from reference resolution).
  CHECK(std::abs(result.rows - 2711) <= 2);
}

TEST_CASE("StitchX4_InvalidDimensions_Throws", "[io][insta360]") {
  // Square input (not 2:1)
  const cv::Mat square(512, 512, CV_8UC3);
  CHECK_THROWS_AS(stitch_insta360_x4(square), std::runtime_error);

  // 4:3 input
  const cv::Mat rect(480, 640, CV_8UC3);
  CHECK_THROWS_AS(stitch_insta360_x4(rect), std::runtime_error);

  // Empty input
  CHECK_THROWS_AS(stitch_insta360_x4(cv::Mat{}), std::runtime_error);
}

// ─── stitch_insta360_x4: coverage ────────────────────────────────────────────

TEST_CASE("StitchX4_UniformWhiteInput_MostOutputPixelsNonZero",
          "[io][insta360]") {
  // Both lenses = white → every equirect pixel covered by at least one lens
  // should be white (or close). Pixels exactly at max-radius cutoff may still
  // be black, but the vast majority should be non-zero.
  const int w = kRefW / 8, h = kRefH / 8;
  const cv::Mat dual(h, w, CV_8UC3, cv::Scalar(255, 255, 255));
  const cv::Mat result = stitch_insta360_x4(dual);

  int nonzero = 0;
  for (int v = 0; v < result.rows; ++v)
    for (int u = 0; u < result.cols; ++u)
      if (result.at<cv::Vec3b>(v, u) != cv::Vec3b(0, 0, 0))
        ++nonzero;

  const int total = result.rows * result.cols;
  // Expect >90 % coverage (both lenses cover ≈200° each → full sphere).
  CHECK(nonzero > total * 9 / 10);
}

// ─── stitch_insta360_x4: map caching ─────────────────────────────────────────

TEST_CASE("StitchX4_SecondCallSameDims_UsesCachedMaps", "[io][insta360]") {
  // Two calls with the same dimensions should produce identical results.
  const int w = kRefW / 8, h = kRefH / 8;
  const cv::Mat dual(h, w, CV_8UC3, cv::Scalar(100, 150, 200));

  const cv::Mat r1 = stitch_insta360_x4(dual);
  const cv::Mat r2 = stitch_insta360_x4(dual);

  cv::Mat diff;
  cv::absdiff(r1, r2, diff);
  CHECK(cv::countNonZero(diff.reshape(1)) == 0);
}

// ─── stitch_insta360_x4: geometry sanity ─────────────────────────────────────

TEST_CASE("StitchX4_FrontLensWhiteBackBlack_FrontRegionBright",
          "[io][insta360]") {
  // Front lens (left half) = white, back lens (right half) = black.
  // The front optical axis is at yaw=28.82°, pitch=15.47°.
  // In a 5422×2711 equirectangular:
  //   u_center ≈ (28.82/360 + 0.5) * W ≈ 0.580 * W
  //   v_center ≈ (0.5 - 15.47/180) * H ≈ 0.414 * H
  // A region around that point should be bright (front lens coverage).
  const int w = kRefW / 8, h = kRefH / 8;
  const cv::Mat dual =
      make_dual_fisheye(w, h, cv::Scalar(255, 255, 255), // front = white
                        cv::Scalar(0, 0, 0));            // back  = black
  const cv::Mat result = stitch_insta360_x4(dual);

  const int out_w = result.cols;
  const int out_h = result.rows;

  // Expected front-lens optical axis pixel (with generous ±5% tolerance).
  const int u_exp = static_cast<int>((28.82 / 360.0 + 0.5) * out_w);
  const int v_exp = static_cast<int>((0.5 - 15.47 / 180.0) * out_h);

  // Sample the 3×3 neighbourhood; at least one pixel should be clearly bright.
  const int margin = std::max(2, out_w / 100);
  int bright_count = 0;
  for (int dv = -margin; dv <= margin; ++dv) {
    for (int du = -margin; du <= margin; ++du) {
      const int uu = u_exp + du;
      const int vv = v_exp + dv;
      if (uu < 0 || uu >= out_w || vv < 0 || vv >= out_h)
        continue;
      const auto px = result.at<cv::Vec3b>(vv, uu);
      if (px[0] > 128 || px[1] > 128 || px[2] > 128)
        ++bright_count;
    }
  }
  CHECK(bright_count > 0);
}

TEST_CASE("StitchX4_BackLensWhiteFrontBlack_BackRegionBright",
          "[io][insta360]") {
  // Back lens optical axis at yaw=-151.18°, pitch=-15.47°
  //   u_center ≈ (-151.18/360 + 0.5) * W ≈ 0.080 * W  (left side of equirect)
  //   v_center ≈ (0.5 + 15.47/180) * H ≈ 0.586 * H
  const int w = kRefW / 8, h = kRefH / 8;
  const cv::Mat dual =
      make_dual_fisheye(w, h, cv::Scalar(0, 0, 0),  // front = black
                        cv::Scalar(255, 255, 255)); // back  = white
  const cv::Mat result = stitch_insta360_x4(dual);

  const int out_w = result.cols;
  const int out_h = result.rows;

  const double lon_back = -151.18 * M_PI / 180.0;
  const int u_exp = static_cast<int>(((lon_back + M_PI) / (2 * M_PI)) * out_w);
  const int v_exp = static_cast<int>((0.5 + 15.47 / 180.0) * out_h);

  const int margin = std::max(2, out_w / 100);
  int bright_count = 0;
  for (int dv = -margin; dv <= margin; ++dv) {
    for (int du = -margin; du <= margin; ++du) {
      const int uu = (u_exp + du + out_w) % out_w; // equirect wraps
      const int vv = v_exp + dv;
      if (vv < 0 || vv >= out_h)
        continue;
      const auto px = result.at<cv::Vec3b>(vv, uu);
      if (px[0] > 128 || px[1] > 128 || px[2] > 128)
        ++bright_count;
    }
  }
  CHECK(bright_count > 0);
}

// ─── import_panoramas: .insp discovery and stitching ─────────────────────────

TEST_CASE("ImportPanoramas_InspFile_IsStitchedAndImported",
          "[io][insta360][panoramas]") {
  TempDir tmp("test_insta360_import");
  const auto scan_dir = tmp.path / "scan";
  fs::create_directories(scan_dir / "360");

  // Write a fake .insp (valid 2:1 dual-fisheye JPEG content, small resolution)
  write_insp(scan_dir / "360" / "IMG_001.insp", kRefW / 8, kRefH / 8);

  reusex::ProjectDB db(tmp.path / "project.rux");
  const auto count = import_panoramas(db, scan_dir);

  CHECK(count == 1);
  CHECK(db.panoramic_image_count() == 1);

  auto panos = db.list_panoramic_images();
  REQUIRE(panos.size() == 1);
  // Stored under the stem name with .jpg extension.
  CHECK(panos[0].filename == "IMG_001.jpg");
}

TEST_CASE("ImportPanoramas_InspWithStitchFalse_IsSkipped",
          "[io][insta360][panoramas]") {
  TempDir tmp("test_insta360_skip");
  const auto scan_dir = tmp.path / "scan";
  fs::create_directories(scan_dir / "360");
  write_insp(scan_dir / "360" / "IMG_001.insp", kRefW / 8, kRefH / 8);

  ImportPanoramasOptions opts;
  opts.stitch_insp = false;

  reusex::ProjectDB db(tmp.path / "project.rux");
  const auto count = import_panoramas(db, scan_dir, opts);

  CHECK(count == 0);
  CHECK(db.panoramic_image_count() == 0);
}

TEST_CASE("ImportPanoramas_InspAndEquirectMixed_BothImported",
          "[io][insta360][panoramas]") {
  TempDir tmp("test_insta360_mixed");
  const auto scan_dir = tmp.path / "scan";
  fs::create_directories(scan_dir / "360");

  write_insp(scan_dir / "360" / "IMG_001.insp", kRefW / 8, kRefH / 8);
  write_jpeg(scan_dir / "360" / "panorama.jpg", 2, 4); // equirect (2:1)

  reusex::ProjectDB db(tmp.path / "project.rux");
  const auto count = import_panoramas(db, scan_dir);

  CHECK(count == 2);
  CHECK(db.panoramic_image_count() == 2);
}

TEST_CASE("ImportPanoramas_InspIdempotent_SecondCallSkips",
          "[io][insta360][panoramas]") {
  TempDir tmp("test_insta360_idempotent");
  const auto scan_dir = tmp.path / "scan";
  fs::create_directories(scan_dir / "360");
  write_insp(scan_dir / "360" / "IMG_001.insp", kRefW / 8, kRefH / 8);

  reusex::ProjectDB db(tmp.path / "project.rux");

  const auto first = import_panoramas(db, scan_dir);
  const auto second = import_panoramas(db, scan_dir);

  CHECK(first == 1);
  CHECK(second == 0);
  CHECK(db.panoramic_image_count() == 1);
}
