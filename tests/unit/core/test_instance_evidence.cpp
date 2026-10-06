// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Instance evidence (Kortlægning A4/A5): instance centroids, the batched
// photo count per survey part, and panoramas ranked around a point with the
// equirect (u, v) the point lands on.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/SensorIntrinsics.hpp>
#include <core/frame_visibility.hpp>
#include <core/instance_evidence.hpp>

#include "../../support/temp_path.hpp"

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <pcl/point_types.h>

#include <array>
#include <atomic>
#include <cmath>
#include <vector>

using reusex::ProjectDB;
using namespace reusex::core;
using namespace reusex::test_support;

namespace {

reusex::core::SensorIntrinsics pinhole() {
  reusex::core::SensorIntrinsics i;
  i.fx = i.fy = 100.0;
  i.cx = i.cy = 64.0;
  i.width = i.height = 128;
  i.local_transform = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  return i;
}

std::array<double, 16> at(double x, double y, double z) {
  return {1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1};
}

void frame(ProjectDB &db, int id, double x, double y, double z) {
  cv::Mat color(128, 128, CV_8UC3, cv::Scalar(40, 90, 160));
  db.save_sensor_frame(id, color, cv::Mat(), cv::Mat(), at(x, y, z), pinhole(),
                       static_cast<double>(id), -1);
}

/// Base cloud + instance cloud: instance 1 around (0,0,2), instance 2 around
/// (0.6,0,2), instance 3 far behind every camera at (0,0,-10); one
/// unlabeled point. Instances get guids so survey parts can link to them.
void seed_instances(ProjectDB &db) {
  reusex::Cloud positions;
  reusex::CloudL labels;
  auto add = [&](float x, float y, float z, std::uint32_t label) {
    pcl::PointXYZRGB p;
    p.x = x;
    p.y = y;
    p.z = z;
    positions.push_back(p);
    pcl::Label l;
    l.label = label;
    labels.push_back(l);
  };
  add(-0.1F, 0.0F, 2.0F, 1);
  add(0.1F, 0.0F, 2.0F, 1);
  add(0.6F, -0.1F, 2.0F, 2);
  add(0.6F, 0.1F, 2.0F, 2);
  add(0.0F, 0.0F, -10.0F, 3);
  add(5.0F, 5.0F, 5.0F, 0);
  db.save_point_cloud("cloud", positions, "test", "{}");
  db.save_point_cloud("instances", labels, "test", "{}");
  db.save_instances("instances", {{1, "guid-inst-1", 3, 2},
                                  {2, "guid-inst-2", 3, 2},
                                  {3, "guid-inst-3", 3, 1}});
}

std::vector<std::uint8_t> tiny_jpeg() {
  cv::Mat img(8, 16, CV_8UC3, cv::Scalar(10, 20, 30));
  std::vector<std::uint8_t> out;
  REQUIRE(cv::imencode(".jpg", img, out));
  return out;
}

} // namespace

TEST_CASE("InstanceCentroids_AveragesEachInstance_SkipsUnlabeled",
          "[core][evidence]") {
  const TempPath tmp("instance_centroids");
  ProjectDB db(tmp.path);
  seed_instances(db);

  const auto c = instance_centroids(db, "instances");
  REQUIRE(c.size() == 3);
  CHECK(c.at(1).point_count == 2);
  CHECK(c.at(1).centroid.x() == Catch::Approx(0.0).margin(1e-6));
  CHECK(c.at(1).centroid.z() == Catch::Approx(2.0));
  CHECK(c.at(2).centroid.x() == Catch::Approx(0.6));
  CHECK(c.count(0) == 0);

  CHECK_THROWS_AS(instance_centroids(db, "nope"), std::invalid_argument);
  CHECK_THROWS_AS(instance_centroids(db, "instances", "nope"),
                  std::invalid_argument);
}

TEST_CASE("VisibleFramesBatch_MatchesPerPointQuery", "[core][evidence]") {
  const TempPath tmp("visible_frames_batch");
  ProjectDB db(tmp.path);
  frame(db, 1, 0.0, 0.0, 0.0);
  frame(db, 2, 0.3, 0.0, 0.0);
  frame(db, 3, 0.0, 0.0, 5.0);
  frame(db, 4, 5.0, 0.0, 0.0);

  const std::vector<Eigen::Vector3d> points{
      {0.0, 0.0, 2.0}, {0.6, 0.0, 2.0}, {0.0, 0.0, -10.0}};
  const auto batch = visible_frames_batch(db, points);
  REQUIRE(batch.size() == points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto single = visible_frames(db, points[i]);
    REQUIRE(batch[i].size() == single.size());
    for (std::size_t k = 0; k < single.size(); ++k) {
      CHECK(batch[i][k].frame_id == single[k].frame_id);
      CHECK(batch[i][k].centrality == Catch::Approx(single[k].centrality));
    }
  }
}

namespace {

/// Frame looking down +z from (x, 0, 0) at a wall `wall` metres away: a
/// constant depth image at HALF the intrinsics' resolution (64x64 for a
/// 128x128 camera), so the depth lookup must scale u, v.
void frame_with_wall(ProjectDB &db, int id, double x, double wall) {
  cv::Mat color(128, 128, CV_8UC3, cv::Scalar(40, 90, 160));
  cv::Mat depth(64, 64, CV_16UC1,
                cv::Scalar(static_cast<double>(
                    static_cast<std::uint16_t>(wall * 1000.0))));
  db.save_sensor_frame(id, color, depth, cv::Mat(), at(x, 0.0, 0.0), pinhole(),
                       static_cast<double>(id), -1);
}

} // namespace

TEST_CASE("VisibleFramesOccluded_DepthDecidesVisibility", "[core][evidence]") {
  const TempPath tmp("visible_frames_occluded");
  ProjectDB db(tmp.path);
  frame_with_wall(db, 1, 0.0, 2.0); // wall at z = 2
  frame(db, 2, 0.3, 0.0, 0.0);      // no depth image

  const VisibilityProbe on_wall{{0.0, 0.0, 2.05}, {}};
  const VisibilityProbe behind_wall{{0.0, 0.0, 4.0}, {}};
  // Centroid in the air in front of the wall (z = 1.5); one sample on it.
  const VisibilityProbe hollow{{0.0, 0.0, 1.5}, {{0.1, 0.0, 2.0}}};

  SECTION("a point on the depth surface is seen, one behind it is not") {
    const auto r = visible_frames_occluded(db, {on_wall, behind_wall, hollow});
    REQUIRE(r.size() == 3);
    REQUIRE(r[0].size() == 2); // frame 1 by depth, frame 2 without depth
    CHECK(r[0][0].frame_id == 1);
    REQUIRE(r[1].size() == 1); // behind the wall: only the depth-less frame
    CHECK(r[1][0].frame_id == 2);
    // The centroid alone fails the depth test; its sample passes.
    REQUIRE(r[2].size() == 2);
  }

  SECTION("a raised cancel flag stops the pass") {
    std::atomic<bool> cancel{true};
    CHECK_THROWS_AS(visible_frames_occluded(db, {on_wall}, {}, &cancel),
                    OperationCancelled);
  }

  SECTION("frames without depth can be dropped instead") {
    OcclusionQuery q;
    q.keep_frames_without_depth = false;
    const auto r = visible_frames_occluded(db, {on_wall, behind_wall}, q);
    CHECK(r[0].size() == 1);
    CHECK(r[1].empty());
  }

  SECTION("the tolerance is the option, not a constant") {
    OcclusionQuery q;
    q.keep_frames_without_depth = false;
    q.depth_tolerance = 0.01; // 5 cm off the wall is now too far
    CHECK(visible_frames_occluded(db, {on_wall}, q)[0].empty());
    q.depth_tolerance =
        2.5; // and with a huge tolerance, the wall hides nothing
    CHECK(visible_frames_occluded(db, {behind_wall}, q)[0].size() == 1);
  }

  SECTION("without samples a hollow instance is not seen through depth") {
    OcclusionQuery q;
    q.keep_frames_without_depth = false;
    const VisibilityProbe centroid_only{hollow.anchor, {}};
    CHECK(visible_frames_occluded(db, {centroid_only}, q)[0].empty());
    CHECK(visible_frames_occluded(db, {hollow}, q)[0].size() == 1);
  }
}

TEST_CASE("InstanceProbes_SamplesSpreadOverThePoints", "[core][evidence]") {
  const TempPath tmp("instance_probes");
  ProjectDB db(tmp.path);
  seed_instances(db);
  const auto probes = instance_probes(db, "instances", "cloud", 8);
  // Instances with fewer points than requested keep all of them.
  CHECK(probes.at(1).samples.size() == 2);
  CHECK(probes.at(1).centroid.point_count == 2);
  CHECK(probes.at(3).samples.size() == 1);
  CHECK(instance_probes(db, "instances", "cloud", 0).at(1).samples.empty());
  const auto one = instance_probes(db, "instances", "cloud", 1).at(2);
  REQUIRE(one.samples.size() == 1);
}

// The shared numeric case with the frontend (`src/test/panorama.test.ts`,
// "bearingToUv pins the C++ equirect_uv case"): bearing (1, -1, 1) has
// theta = pi/4 and phi = asin(1/sqrt(3)), so u = 0.625, v = 0.3040867.
TEST_CASE("EquirectUv_MatchesFrontendBearingToUv", "[core][evidence]") {
  const auto uv = equirect_uv(Eigen::Vector3d(1.0, -1.0, 1.0));
  CHECK(uv.x() == Catch::Approx(0.625));
  CHECK(uv.y() == Catch::Approx(0.3040867).margin(1e-6));

  // Forward is the image centre; up (-y) is the top row.
  CHECK(equirect_uv({0, 0, 1}).x() == Catch::Approx(0.5));
  CHECK(equirect_uv({0, 0, 1}).y() == Catch::Approx(0.5));
  CHECK(equirect_uv({0, -1, 0}).y() == Catch::Approx(0.0).margin(1e-9));
  CHECK(equirect_uv({1, 0, 0}).x() == Catch::Approx(0.75));
}

TEST_CASE("PanoramasForPoint_RanksPlaceablePanoramasByDistance",
          "[core][evidence]") {
  const TempPath tmp("panoramas_for_point");
  ProjectDB db(tmp.path);
  frame(db, 10, 3.0, 0.0, 0.0); // matched frame for the levelled panorama
  const auto jpeg = tiny_jpeg();
  db.save_panoramic_image("near.jpg", jpeg, 1.0, -1);     // id 1: resected
  db.save_panoramic_image("levelled.jpg", jpeg, 2.0, 10); // id 2: frame pose
  db.save_panoramic_image("lost.jpg", jpeg, 3.0, -1);     // id 3: unplaceable
  db.save_panoramic_image("far.jpg", jpeg, 4.0, -1);      // id 4: too far
  db.save_panorama_pose(1, at(0.0, 0.0, 0.0), 50, 0.5);
  db.save_panorama_pose(4, at(100.0, 0.0, 0.0), 50, 0.5);

  const Eigen::Vector3d point(1.0, -1.0, 1.0);
  const auto hits = panoramas_for_point(db, point);
  REQUIRE(hits.size() == 2);

  // Resected at the origin, identity rotation: bearing = the point itself.
  CHECK(hits[0].panorama_id == 1);
  CHECK(hits[0].heading == PanoramaHeading::resected);
  CHECK(hits[0].distance == Catch::Approx(std::sqrt(3.0)));
  CHECK(hits[0].u == Catch::Approx(0.625));
  CHECK(hits[0].v == Catch::Approx(0.3040867).margin(1e-6));

  // Levelled at (3,0,0): world offset (-2,-1,1). Panorama frame:
  // x = -world y = 1, y = -world z = -1, z = world x = -2.
  CHECK(hits[1].panorama_id == 2);
  CHECK(hits[1].node_id == 10);
  CHECK(hits[1].heading == PanoramaHeading::levelled);
  CHECK(hits[1].distance == Catch::Approx(std::sqrt(6.0)));
  const auto expected = equirect_uv(Eigen::Vector3d(1.0, -1.0, -2.0));
  CHECK(hits[1].u == Catch::Approx(expected.x()));
  CHECK(hits[1].v == Catch::Approx(expected.y()));

  SECTION("max_distance 0 keeps every placeable panorama") {
    PanoramaQuery q;
    q.max_distance = 0.0;
    const auto all = panoramas_for_point(db, point, q);
    REQUIRE(all.size() == 3);
    CHECK(all[2].panorama_id == 4);
  }
}
