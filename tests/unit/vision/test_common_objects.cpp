// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <vision/common/create_object.hpp>
#include <vision/common/object.hpp>
#include <opencv2/opencv.hpp>

using namespace ReUseX::vision::common::object;
using Catch::Matchers::WithinAbs;

TEST_CASE("Box geometric calculations", "[vision][box]") {
  Box box(10.0f, 20.0f, 30.0f, 40.0f); // left=10, top=20, right=30, bottom=40

  REQUIRE_THAT(box.width(), WithinAbs(20.0f, 0.001f));  // 30 - 10
  REQUIRE_THAT(box.height(), WithinAbs(20.0f, 0.001f)); // 40 - 20
  REQUIRE_THAT(box.area(), WithinAbs(400.0f, 0.001f));  // 20 * 20
  REQUIRE_THAT(box.center_x(), WithinAbs(20.0f, 0.001f)); // (10 + 30) / 2
  REQUIRE_THAT(box.center_y(), WithinAbs(30.0f, 0.001f)); // (20 + 40) / 2
}

TEST_CASE("Box default constructor", "[vision][box]") {
  Box box;

  REQUIRE_THAT(box.left, WithinAbs(0.0f, 0.001f));
  REQUIRE_THAT(box.top, WithinAbs(0.0f, 0.001f));
  REQUIRE_THAT(box.right, WithinAbs(0.0f, 0.001f));
  REQUIRE_THAT(box.bottom, WithinAbs(0.0f, 0.001f));
}

TEST_CASE("PosePoint construction and assignment", "[vision][pose]") {
  PosePoint pt1(1.5f, 2.5f, 0.8f);
  REQUIRE_THAT(pt1.x, WithinAbs(1.5f, 0.001f));
  REQUIRE_THAT(pt1.y, WithinAbs(2.5f, 0.001f));
  REQUIRE_THAT(pt1.vis, WithinAbs(0.8f, 0.001f));

  PosePoint pt2;
  pt2 = pt1;
  REQUIRE_THAT(pt2.x, WithinAbs(pt1.x, 0.001f));
  REQUIRE_THAT(pt2.y, WithinAbs(pt1.y, 0.001f));
  REQUIRE_THAT(pt2.vis, WithinAbs(pt1.vis, 0.001f));
}

TEST_CASE("Obb area calculation", "[vision][obb]") {
  Obb obb(5.0f, 5.0f, 10.0f, 20.0f, 45.0f); // center=(5,5), size=(10,20), angle=45°

  REQUIRE_THAT(obb.area(), WithinAbs(200.0f, 0.001f)); // 10 * 20
}

TEST_CASE("Depth statistics with empty matrix", "[vision][depth]") {
  Depth depth;
  depth.depth = cv::Mat(); // Empty

  REQUIRE_THAT(depth.average_depth(), WithinAbs(0.0f, 0.001f));
  REQUIRE_THAT(depth.min_depth(), WithinAbs(0.0f, 0.001f));
  REQUIRE_THAT(depth.max_depth(), WithinAbs(0.0f, 0.001f));
}

TEST_CASE("Depth statistics with valid data", "[vision][depth]") {
  Depth depth;
  depth.depth = cv::Mat(2, 2, CV_32F);
  depth.depth.at<float>(0, 0) = 1.0f;
  depth.depth.at<float>(0, 1) = 2.0f;
  depth.depth.at<float>(1, 0) = 3.0f;
  depth.depth.at<float>(1, 1) = 4.0f;

  REQUIRE_THAT(depth.average_depth(), WithinAbs(2.5f, 0.001f)); // (1+2+3+4)/4
  REQUIRE_THAT(depth.min_depth(), WithinAbs(1.0f, 0.001f));
  REQUIRE_THAT(depth.max_depth(), WithinAbs(4.0f, 0.001f));

  float val_00 = depth.point_depth(0, 0);
  float val_11 = depth.point_depth(1, 1);
  REQUIRE_THAT(val_00, WithinAbs(1.0f, 0.001f));
  REQUIRE_THAT(val_11, WithinAbs(4.0f, 0.001f));
}

TEST_CASE("create_box factory function", "[vision][factory]") {
  // create_box takes individual coordinates
  auto obj = create_box(10.0f, 20.0f, 30.0f, 40.0f, 0.9f, 5, "person");

  REQUIRE(obj.type == ObjectType::detection);
  REQUIRE_THAT(obj.box.left, WithinAbs(10.0f, 0.001f));
  REQUIRE_THAT(obj.box.top, WithinAbs(20.0f, 0.001f));
  REQUIRE_THAT(obj.score, WithinAbs(0.9f, 0.001f));
  REQUIRE(obj.class_id == 5);
  REQUIRE(obj.class_name == "person");
}

TEST_CASE("create_obb_box factory function", "[vision][factory]") {
  // create_obb_box takes individual OBB parameters
  float cx = 5.0f, cy = 5.0f, w = 10.0f, h = 20.0f, angle = 0.0f;
  auto obj = create_obb_box(cx, cy, w, h, angle, 0.85f, 3, "box");

  REQUIRE(obj.type == ObjectType::obb);
  REQUIRE(obj.obb.has_value());
  REQUIRE_THAT(obj.obb->cx, WithinAbs(5.0f, 0.001f));
  REQUIRE_THAT(obj.obb->cy, WithinAbs(5.0f, 0.001f));
  REQUIRE_THAT(obj.score, WithinAbs(0.85f, 0.001f));
  REQUIRE(obj.class_id == 3);
  REQUIRE(obj.class_name == "box");

  // Verify AABB computed from OBB
  REQUIRE(obj.box.width() > 0);
  REQUIRE(obj.box.height() > 0);
}
