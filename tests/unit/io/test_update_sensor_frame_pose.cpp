// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <opencv2/core.hpp>

#include <array>
#include <filesystem>

using Catch::Matchers::WithinAbs;

TEST_CASE("update_sensor_frame_pose round-trips the transform", "[io][db]") {
  namespace fs = std::filesystem;
  fs::path db_path =
      fs::temp_directory_path() / "reusex_update_pose_test.rux";
  fs::remove(db_path);

  {
    reusex::ProjectDB db(db_path);

    // Minimal sensor frame: a tiny color image, identity pose, default intr.
    cv::Mat color(4, 4, CV_8UC3, cv::Scalar(10, 20, 30));
    std::array<double, 16> identity = {1, 0, 0, 0, 0, 1, 0, 0,
                                       0, 0, 1, 0, 0, 0, 0, 1};
    reusex::core::SensorIntrinsics intr;
    intr.fx = intr.fy = 100.0;
    intr.cx = intr.cy = 2.0;
    intr.width = intr.height = 4;
    db.save_sensor_frame(7, color, cv::Mat(), cv::Mat(), identity, intr);

    // Overwrite only the pose.
    std::array<double, 16> pose = {0, -1, 0, 1.5, 1, 0, 0, -2.5,
                                   0, 0, 1, 3.5, 0, 0, 0, 1};
    db.update_sensor_frame_pose(7, pose);

    auto read_back = db.sensor_frame_pose(7);
    for (int i = 0; i < 16; ++i)
      REQUIRE_THAT(read_back[i], WithinAbs(pose[i], 1e-12));

    // Other data untouched: color blob still readable, intrinsics intact.
    REQUIRE_FALSE(db.sensor_frame_image(7).empty());
    REQUIRE_THAT(db.sensor_frame_intrinsics(7).fx, WithinAbs(100.0, 1e-9));

    // Updating a missing frame throws.
    REQUIRE_THROWS(db.update_sensor_frame_pose(999, identity));
  }

  fs::remove(db_path);
}
