// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Validates the equirectangular <-> perspective reprojection used by the 360
// panorama alignment and SAM3-on-360 features: bearing round-trips, the virtual
// view basis, and the label-stitch centrality argmax.

#include <reusex/geometry/EquirectProjection.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

using Catch::Matchers::WithinAbs;
using namespace reusex::geometry;

namespace {
constexpr double kPi = 3.14159265358979323846;
}

TEST_CASE("pixel<->bearing round-trips", "[geometry][equirect]") {
  const cv::Size sz(2048, 1024);
  // sample a grid of longitudes/latitudes, avoiding the exact poles
  for (double yaw = -170; yaw <= 170; yaw += 40) {
    for (double pitch = -80; pitch <= 80; pitch += 40) {
      const double th = yaw * kPi / 180.0, ph = pitch * kPi / 180.0;
      const Eigen::Vector3d d(std::sin(th) * std::cos(ph), -std::sin(ph),
                              std::cos(th) * std::cos(ph));
      const Eigen::Vector2d uv = bearing_to_pixel(sz, d);
      const Eigen::Vector3d d2 = pixel_to_bearing(sz, uv.x(), uv.y());
      REQUIRE_THAT((d - d2).norm(), WithinAbs(0.0, 1e-3));
    }
  }
}

TEST_CASE("forward bearing maps to image centre", "[geometry][equirect]") {
  const cv::Size sz(2048, 1024);
  // d(0,0) must land at the horizontal centre, vertical centre
  const Eigen::Vector2d uv = bearing_to_pixel(sz, Eigen::Vector3d(0, 0, 1));
  REQUIRE_THAT(uv.x(), WithinAbs(sz.width / 2.0, 1.0));
  REQUIRE_THAT(uv.y(), WithinAbs(sz.height / 2.0, 1.0));
}

TEST_CASE("virtual view basis points at (yaw,pitch)", "[geometry][equirect]") {
  cv::Mat equirect(256, 512, CV_8UC3, cv::Scalar(40, 80, 120));
  const double yaw = 35.0, pitch = -18.0;
  PerspectiveView v = extract_perspective(equirect, yaw, pitch, 90.0, 128, 128);

  // optical axis (3rd column) must equal the bearing at (yaw,pitch)
  const double th = yaw * kPi / 180.0, ph = pitch * kPi / 180.0;
  const Eigen::Vector3d expect(std::sin(th) * std::cos(ph), -std::sin(ph),
                               std::cos(th) * std::cos(ph));
  REQUIRE_THAT((v.R_pano_from_view.col(2) - expect).norm(),
               WithinAbs(0.0, 1e-9));
  // basis is orthonormal
  REQUIRE_THAT((v.R_pano_from_view.transpose() * v.R_pano_from_view -
                Eigen::Matrix3d::Identity())
                   .norm(),
               WithinAbs(0.0, 1e-9));
  // a flat equirect renders a flat view
  REQUIRE(v.image.size() == cv::Size(128, 128));
}

TEST_CASE("label stitch picks the most central tile", "[geometry][equirect]") {
  cv::Mat equirect(512, 1024, CV_8UC3, cv::Scalar(0, 0, 0));
  // 4 equator views + up + down, wide overlap
  auto views = overlapping_views(equirect, 4, 120.0, 128);
  REQUIRE(views.size() == 6);

  // paint each tile a constant unique label == its index
  std::vector<cv::Mat> tiles;
  for (size_t k = 0; k < views.size(); ++k)
    tiles.emplace_back(views[k].image.rows, views[k].image.cols, CV_32S,
                       cv::Scalar(static_cast<int>(k)));

  cv::Mat labels = stitch_labels_to_equirect(equirect.size(), views, tiles);
  REQUIRE(labels.type() == CV_32S);

  // at the exact centre bearing of equator view i, that view must win
  const cv::Size sz = equirect.size();
  for (int i = 0; i < 4; ++i) {
    const Eigen::Vector3d f = views[i].R_pano_from_view.col(2);
    const Eigen::Vector2d uv = bearing_to_pixel(sz, f);
    const int lbl =
        labels.at<int>(static_cast<int>(uv.y()), static_cast<int>(uv.x()));
    REQUIRE(lbl == i);
  }
  // whole sphere is covered (no background left)
  double mn, mx;
  cv::minMaxLoc(labels, &mn, &mx);
  REQUIRE(mn >= 0);
}
