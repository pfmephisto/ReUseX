// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/label_semantics.hpp>

#include <opencv2/core.hpp>

#include <stdexcept>

using namespace reusex::core;

TEST_CASE("Label constants match the encoding contract", "[label_semantics]") {
  REQUIRE(kUnlabeled == 0u);
  REQUIRE(kBackgroundApi == -1);
  REQUIRE(kMaxStorableLabel == 65534);
}

TEST_CASE("is_valid_label distinguishes unlabeled from valid",
          "[label_semantics]") {
  REQUIRE_FALSE(is_valid_label(0u));
  REQUIRE(is_valid_label(1u));
  REQUIRE(is_valid_label(65534u));
  REQUIRE(is_valid_label(65535u));
  REQUIRE(is_valid_label(4000000000u));
}

TEST_CASE("label_to_index converts 1-based labels and throws on unlabeled",
          "[label_semantics]") {
  REQUIRE(label_to_index(1u) == 0u);
  REQUIRE(label_to_index(2u) == 1u);
  REQUIRE(label_to_index(65534u) == 65533u);
  REQUIRE_THROWS_AS(label_to_index(0u), std::out_of_range);
}

TEST_CASE("api <-> point label scalar conversions", "[label_semantics]") {
  SECTION("background maps to unlabeled and back") {
    REQUIRE(api_to_point_label(kBackgroundApi) == kUnlabeled);
    REQUIRE(point_label_to_api(kUnlabeled) == kBackgroundApi);
  }

  SECTION("valid class ids pass through") {
    REQUIRE(api_to_point_label(0) == 0u); // API class 0 is a valid class id
    REQUIRE(api_to_point_label(5) == 5u);
    REQUIRE(point_label_to_api(5u) == 5);
  }

  SECTION("negative non-sentinel throws") {
    REQUIRE_THROWS_AS(api_to_point_label(-2), std::out_of_range);
  }

  SECTION("round-trip through api for valid labels") {
    for (uint32_t p : {1u, 2u, 100u, 65534u, 65535u}) {
      REQUIRE(api_to_point_label(point_label_to_api(p)) == p);
    }
  }
}

TEST_CASE("api <-> storage scalar conversions with boundary labels",
          "[label_semantics]") {
  SECTION("background") {
    REQUIRE(api_to_storage(kBackgroundApi) == 0u);
    REQUIRE(storage_to_api(0u) == kBackgroundApi);
  }

  SECTION("boundary labels round-trip") {
    // API label L stores as L+1; background (-1) stores as 0.
    REQUIRE(api_to_storage(0) == 1u);
    REQUIRE(storage_to_api(1u) == 0);

    REQUIRE(api_to_storage(1) == 2u);
    REQUIRE(storage_to_api(2u) == 1);

    REQUIRE(api_to_storage(65534) == 65535u);
    REQUIRE(storage_to_api(65535u) == 65534);
  }

  SECTION("overflow throws instead of wrapping") {
    REQUIRE_THROWS_AS(api_to_storage(65535), std::out_of_range);
    REQUIRE_THROWS_AS(api_to_storage(100000), std::out_of_range);
  }

  SECTION("below-background throws") {
    REQUIRE_THROWS_AS(api_to_storage(-2), std::out_of_range);
  }
}

TEST_CASE("cv::Mat storage <-> api round-trip", "[label_semantics]") {
  // Build an API image (CV_32S) with background and boundary labels.
  cv::Mat api(2, 2, CV_32SC1);
  api.at<int>(0, 0) = kBackgroundApi; // -1 background
  api.at<int>(0, 1) = 0;              // class 0
  api.at<int>(1, 0) = 1;              // class 1
  api.at<int>(1, 1) = 65534;          // max storable

  cv::Mat storage = api_mat_to_storage(api);
  REQUIRE(storage.type() == CV_16UC1);
  REQUIRE(storage.at<uint16_t>(0, 0) == 0u);
  REQUIRE(storage.at<uint16_t>(0, 1) == 1u);
  REQUIRE(storage.at<uint16_t>(1, 0) == 2u);
  REQUIRE(storage.at<uint16_t>(1, 1) == 65535u);

  cv::Mat api_rt = storage_mat_to_api(storage);
  REQUIRE(api_rt.type() == CV_32SC1);
  REQUIRE(api_rt.at<int>(0, 0) == kBackgroundApi);
  REQUIRE(api_rt.at<int>(0, 1) == 0);
  REQUIRE(api_rt.at<int>(1, 0) == 1);
  REQUIRE(api_rt.at<int>(1, 1) == 65534);
}

TEST_CASE("cv::Mat api_mat_to_storage rejects overflowing labels",
          "[label_semantics]") {
  cv::Mat api(1, 2, CV_32SC1);
  api.at<int>(0, 0) = 1;
  api.at<int>(0, 1) = 65535; // one past the storable max -> must throw
  REQUIRE_THROWS_AS(api_mat_to_storage(api), std::out_of_range);
}

TEST_CASE("cv::Mat converters validate input type and emptiness",
          "[label_semantics]") {
  SECTION("empty storage returns empty") {
    REQUIRE(storage_mat_to_api(cv::Mat()).empty());
  }
  SECTION("empty api throws") {
    REQUIRE_THROWS_AS(api_mat_to_storage(cv::Mat()), std::invalid_argument);
  }
  SECTION("wrong storage type throws") {
    cv::Mat wrong(2, 2, CV_8UC1, cv::Scalar(0));
    REQUIRE_THROWS_AS(storage_mat_to_api(wrong), std::invalid_argument);
  }
  SECTION("wrong api type throws") {
    cv::Mat wrong(2, 2, CV_32FC1, cv::Scalar(0));
    REQUIRE_THROWS_AS(api_mat_to_storage(wrong), std::invalid_argument);
  }
}
