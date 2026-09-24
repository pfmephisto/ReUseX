// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Synthetic integration tests for the ICP refine callback (#465).
//
// Uses make_icp_refine_fn() (rux_lib / rux_gui_lib injection point) against
// a real ProjectDB with synthetic sensor frames whose geometry and relative
// pose are known, so the recovered transform can be checked within tolerance.
//
// Lives in tests/unit/rux_app/ → reusex_unit_tests_vision (heavy binary that
// links rux_lib and therefore has PCL and the ICP implementation).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <gui_icp.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>

#include <opencv2/core.hpp>

#include "../../support/temp_path.hpp"

#include <array>
#include <cmath>
#include <cstdint>

using reusex::test_support::TempPath;

namespace {

constexpr int kW = 160;
constexpr int kH = 120;

reusex::core::SensorIntrinsics make_intr() {
  reusex::core::SensorIntrinsics k;
  k.fx = k.fy = kW / 2.0;
  k.cx = kW / 2.0;
  k.cy = kH / 2.0;
  k.width = kW;
  k.height = kH;
  // local_transform stays identity (default)
  return k;
}

std::array<double, 16> identity_pose() {
  return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
}

/// Pose with a translation of tx in x (row-major 4×4).
std::array<double, 16> translate_pose(double tx) {
  return {1, 0, 0, tx, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
}

/// Depth image of a flat scene at z_mm millimetres everywhere.
cv::Mat flat_depth(uint16_t z_mm) {
  return cv::Mat(kH, kW, CV_16UC1, cv::Scalar(z_mm));
}

cv::Mat dummy_color() {
  return cv::Mat(kH, kW, CV_8UC3, cv::Scalar(128, 128, 128));
}

cv::Mat full_confidence() { return cv::Mat(kH, kW, CV_8UC1, cv::Scalar(255)); }

} // namespace

// ---------------------------------------------------------------------------

TEST_CASE("IcpRefine_EmptyDepth_ThrowsOnRefine", "[gui][icp][app]") {
  // A frame with no depth stored → IcpRefineFn should throw (caller catches
  // and maps to 422).
  const TempPath project("gui_icp_nodepth");
  {
    reusex::ProjectDB db(project.path, /*readOnly=*/false);
    const auto intr = make_intr();
    // Save a frame with NO depth (empty Mat).
    db.save_sensor_frame(1, dummy_color(), cv::Mat{}, full_confidence(),
                         identity_pose(), intr, 0.0);
    db.save_sensor_frame(2, dummy_color(), flat_depth(1000), full_confidence(),
                         translate_pose(0.15), intr, 1.0);
  }

  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto fn = rux::make_icp_refine_fn();

  CHECK_THROWS_AS(fn(db, 1, 2), std::runtime_error);
}

TEST_CASE("IcpRefine_FlatPlane_ConvergesAndRecoversTranslation",
          "[gui][icp][app]") {
  // Frame A at origin, frame B translated 0.15 m in x. Both frames see a flat
  // plane at z = 1 m (1000 mm). The back-projected world-space clouds share
  // the same point positions in the overlap region, so ICP should converge to
  // T_delta ≈ identity and yield T_rel ≈ T_B^{-1} = translate(−0.15 m).
  const TempPath project("gui_icp_flatplane");
  const auto intr = make_intr();

  {
    reusex::ProjectDB db(project.path, /*readOnly=*/false);
    db.save_sensor_frame(1, dummy_color(), flat_depth(1000), full_confidence(),
                         identity_pose(), intr, 0.0);
    db.save_sensor_frame(2, dummy_color(), flat_depth(1000), full_confidence(),
                         translate_pose(0.15), intr, 1.0);
  }

  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto fn = rux::make_icp_refine_fn();
  const auto result = fn(db, 1, 2);

  // ICP must converge on this simple geometry.
  CHECK(result.converged);

  // RMS error should be very small (< 5 cm) when clouds perfectly overlap.
  CHECK(result.fitness < 0.05);

  // Most source points should match the target within 5 cm.
  CHECK(result.inlier_fraction > 0.5);

  // Relative pose: element [0][3] (= relative_pose[3]) is the x-translation
  // in the "to" frame's coordinate system. Frame A is 0.15 m behind B in x.
  CHECK(std::abs(result.relative_pose[3] - (-0.15)) < 0.03);

  // Diagonal should be close to 1 (rotation near identity).
  CHECK(std::abs(result.relative_pose[0] - 1.0) < 0.05);
  CHECK(std::abs(result.relative_pose[5] - 1.0) < 0.05);
  CHECK(std::abs(result.relative_pose[10] - 1.0) < 0.05);
}
