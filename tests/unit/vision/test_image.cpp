// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <vision/common/image.hpp>
#include <opencv2/opencv.hpp>

using namespace reusex::vision::common::tensor;

TEST_CASE("cvimg creates Image view from cv::Mat", "[vision][image]") {
  // Create test Mat (3x3 BGR image)
  cv::Mat mat(3, 3, CV_8UC3);
  mat.setTo(cv::Scalar(255, 0, 0)); // Blue

  Image img = cvimg(mat);

  REQUIRE(img.width == 3);
  REQUIRE(img.height == 3);
  REQUIRE(img.bgrptr != nullptr);

  // Verify zero-copy (data pointer should match)
  REQUIRE(img.bgrptr == mat.data);

  // Verify it's BGR format
  const uint8_t* data = static_cast<const uint8_t*>(img.bgrptr);
  REQUIRE(data[0] == 255); // Blue channel
  REQUIRE(data[1] == 0);   // Green channel
  REQUIRE(data[2] == 0);   // Red channel
}

TEST_CASE("cvimg handles empty Mat", "[vision][image]") {
  cv::Mat empty_mat;
  Image img = cvimg(empty_mat);

  REQUIRE(img.width == 0);
  REQUIRE(img.height == 0);
  REQUIRE(img.bgrptr == nullptr);
}

TEST_CASE("cvimg handles different Mat types", "[vision][image]") {
  // Test with 1x1 image
  cv::Mat small_mat(1, 1, CV_8UC3);
  small_mat.at<cv::Vec3b>(0, 0) = cv::Vec3b(10, 20, 30);

  Image img = cvimg(small_mat);

  REQUIRE(img.width == 1);
  REQUIRE(img.height == 1);
  const uint8_t* data = static_cast<const uint8_t*>(img.bgrptr);
  REQUIRE(data[0] == 10);
  REQUIRE(data[1] == 20);
  REQUIRE(data[2] == 30);
}
