// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Covers the video-annotation resume/sequence-boundary logic (issues #233,
// #234): IDataset::node_id(), IDataset::filter_annotated_prefix() and the
// is_sequence_boundary() reset predicate.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <vision/annotate.hpp>
#include <vision/tensor_rt/Dataset.hpp>

#include <opencv2/core.hpp>

#include <memory>
#include <vector>

using reusex::ProjectDB;
using reusex::vision::is_sequence_boundary;
using reusex::vision::tensor_rt::TensorRTDataset;

namespace {
// Populate an in-memory project with sensor frames for every id in @p ids and a
// segmentation image for every id in @p annotated.
std::shared_ptr<ProjectDB> make_project(const std::vector<int> &ids,
                                        const std::vector<int> &annotated) {
  auto db = std::make_shared<ProjectDB>(":memory:");
  cv::Mat color(4, 4, CV_8UC3, cv::Scalar(0, 0, 0));
  for (int id : ids)
    db->save_sensor_frame(id, color);
  cv::Mat labels(4, 4, CV_32S, cv::Scalar(0)); // API convention (0 = class 0)
  for (int id : annotated)
    db->save_segmentation_image(id, labels);
  return db;
}
} // namespace

TEST_CASE("node_id maps dataset index to ascending node id",
          "[vision][annotate]") {
  auto db = make_project({5, 10, 15, 20, 25}, {});
  TensorRTDataset ds(db);

  REQUIRE(ds.size() == 5);
  REQUIRE(ds.node_id(0) == 5);
  REQUIRE(ds.node_id(2) == 15);
  REQUIRE(ds.node_id(4) == 25);
  REQUIRE_THROWS_AS(ds.node_id(5), std::out_of_range);
}

TEST_CASE("filter_annotated_prefix skips only the leading contiguous run",
          "[vision][annotate]") {
  // Leading prefix {5,10} annotated, plus a later one {20}. Only {5,10} should
  // be skipped; {20} must remain because dropping it mid-sequence would gap the
  // tracker memory bank.
  auto db = make_project({5, 10, 15, 20, 25}, {5, 10, 20});
  TensorRTDataset ds(db);

  auto skipped = ds.filter_annotated_prefix();
  REQUIRE(skipped == 2);
  REQUIRE(ds.size() == 3);
  REQUIRE(ds.node_id(0) == 15);
  REQUIRE(ds.node_id(1) == 20); // later-annotated frame is kept
  REQUIRE(ds.node_id(2) == 25);
}

TEST_CASE("filter_annotated_prefix with no leading annotations is a no-op",
          "[vision][annotate]") {
  auto db = make_project({5, 10, 15}, {10}); // gap at the very first frame
  TensorRTDataset ds(db);

  REQUIRE(ds.filter_annotated_prefix() == 0);
  REQUIRE(ds.size() == 3);
}

TEST_CASE("filter_annotated_prefix skips the whole list when all annotated",
          "[vision][annotate]") {
  auto db = make_project({5, 10, 15}, {5, 10, 15});
  TensorRTDataset ds(db);

  REQUIRE(ds.filter_annotated_prefix() == 3);
  REQUIRE(ds.size() == 0);
}

TEST_CASE("filter_annotated still drops all annotated frames (unchanged)",
          "[vision][annotate]") {
  auto db = make_project({5, 10, 15, 20, 25}, {5, 10, 20});
  TensorRTDataset ds(db);

  REQUIRE(ds.filter_annotated() == 3);
  REQUIRE(ds.size() == 2);
  REQUIRE(ds.node_id(0) == 15);
  REQUIRE(ds.node_id(1) == 25);
}

TEST_CASE("is_sequence_boundary resets at index 0 and non-increasing node ids",
          "[vision][annotate]") {
  // First frame is always a boundary.
  REQUIRE(is_sequence_boundary(0, 5, 0));

  // Monotonically increasing ids within one scan: no reset, even across gaps
  // (dropped frames leave forward jumps that must NOT reset).
  REQUIRE_FALSE(is_sequence_boundary(1, 10, 5));
  REQUIRE_FALSE(
      is_sequence_boundary(2, 25, 10)); // large forward gap: still fine

  // A second concatenated sequence restarts numbering -> id drops -> reset.
  REQUIRE(is_sequence_boundary(3, 1, 25));
  // Repeated id (delta 0) is also a boundary.
  REQUIRE(is_sequence_boundary(4, 7, 7));
}
