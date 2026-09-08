// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Integration test against a REAL scan (#204).
//
// Every other test in this suite feeds the pipeline synthetic geometry: clean
// planes, Gaussian noise, exact poses. That catches logic errors but is blind
// to the class of bug that only shows up on sensor data -- depth-filter
// behaviour on real confidence maps, a pose-convention flip, a label
// projection artefact. This test runs the first two pipeline stages
//
//     sensor frames -> reconstruct_point_clouds -> segment_planes
//
// against ten consecutive frames of a real iOS-LiDAR capture and asserts the
// result stays inside bounds measured from a known-good run.
//
// The fixture is `tests/fixtures/scans/office_corridor.rux`, tracked in
// git-lfs. Provenance, licence and the exact trimming commands are documented
// in `tests/fixtures/scans/README.md`.
//
// ── Where the bounds come from ─────────────────────────────────────────────
//
// Reference run (Release, GCC 15.2, PCL from the flake, 2026-09-08). Repeating
// it three times gave bit-identical output, so the pipeline is deterministic
// on a fixed toolchain and every band below is slack for a *toolchain* change,
// not for run-to-run jitter:
//
//     reconstructed points   68761
//     planes detected        5      (2 horizontal, 3 vertical)
//     labeled points         49243  (71.6 %)
//     bbox extent            3.64 x 3.97 x 2.94 m
//
// The bands are deliberately narrow. Any behavioural regression this test
// exists to catch -- a broken depth/confidence filter, a mis-scaled voxel
// grid, an inverted pose convention, segmentation collapsing or exploding --
// moves these numbers by tens of percent or destroys the plane orientations
// outright, so a few percent of slack costs no sensitivity while surviving a
// PCL/compiler bump in the flake.

#include "../support/temp_path.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>
#include <reusex/segmentation/reconstruct.hpp>
#include <reusex/segmentation/segment_planes.hpp>
#include <reusex/types/point_types.hpp>

#include <pcl/common/common.h>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <string>

namespace fs = std::filesystem;

namespace {

/// tests/integration/<this file> -> tests/fixtures/scans/office_corridor.rux
fs::path fixture_path() {
  return fs::path(__FILE__).parent_path().parent_path() / "fixtures" / "scans" /
         "office_corridor.rux";
}

/// True when `p` is an unexpanded git-lfs pointer rather than the real blob.
///
/// A clone without `git lfs pull` leaves a ~130-byte text stub in place of the
/// database. Opening that as sqlite3 fails with a confusing "file is not a
/// database", so detect it explicitly and skip with an actionable message.
bool is_lfs_pointer(const fs::path &p) {
  std::error_code ec;
  const auto size = fs::file_size(p, ec);
  if (ec || size > 1024) {
    return false;
  }
  std::ifstream in(p, std::ios::binary);
  std::string head(64, '\0');
  in.read(head.data(), static_cast<std::streamsize>(head.size()));
  head.resize(static_cast<std::size_t>(in.gcount()));
  return head.rfind("version https://git-lfs", 0) == 0;
}

/// Count points per label, ignoring label == 0 (unlabeled, STANDARDS §3).
std::map<std::uint32_t, std::size_t>
label_histogram(const reusex::CloudL &labels) {
  std::map<std::uint32_t, std::size_t> hist;
  for (const auto &pt : labels.points) {
    if (pt.label > 0) {
      ++hist[pt.label];
    }
  }
  return hist;
}

} // namespace

TEST_CASE("Real-scan fixture: frames reconstruct and segment within bounds",
          "[integration][fixture]") {
  const auto fixture = fixture_path();

  if (!fs::exists(fixture)) {
    SKIP("Fixture missing: " << fixture
                             << " -- run `git lfs pull` to fetch it.");
  }
  if (is_lfs_pointer(fixture)) {
    SKIP("Fixture is an unexpanded git-lfs pointer: "
         << fixture << " -- run `git lfs pull` to fetch it.");
  }

  // Work on a private copy: the stages below write clouds back into the
  // project, and the fixture in the source tree must stay pristine. The temp
  // name is unique per process so `ctest --parallel` cannot collide (#262).
  const reusex::test_support::TempDir work("reusex_scan_fixture");
  const fs::path project = work.path / "office_corridor.rux";
  fs::copy_file(fixture, project);

  reusex::ProjectDB db(project);
  REQUIRE(db.is_open());

  // ── Stage 0: the frames are intact and carry all five components ────────
  const auto frame_ids = db.sensor_frame_ids();
  REQUIRE(frame_ids.size() == 10);

  for (const int id : frame_ids) {
    INFO("sensor frame " << id);
    REQUIRE_FALSE(db.sensor_frame_image(id).empty());
    REQUIRE_FALSE(db.sensor_frame_depth(id).empty());
    REQUIRE_FALSE(db.sensor_frame_confidence(id).empty());

    const auto intrinsics = db.sensor_frame_intrinsics(id);
    CHECK(intrinsics.width == 720);
    CHECK(intrinsics.height == 960);
    CHECK(intrinsics.fx > 0.0);
    CHECK(intrinsics.fy > 0.0);

    // A pose must be a real rigid transform, not a zero-filled placeholder:
    // the bottom row is exactly [0 0 0 1] and the translation is finite.
    const auto pose = db.sensor_frame_pose(id);
    CHECK(pose[12] == 0.0);
    CHECK(pose[13] == 0.0);
    CHECK(pose[14] == 0.0);
    CHECK(pose[15] == 1.0);
    CHECK(std::isfinite(pose[3]));
    CHECK(std::isfinite(pose[7]));
    CHECK(std::isfinite(pose[11]));
  }

  // ── Stage 1: back-project depth into a fused cloud ──────────────────────
  //
  // Every parameter is pinned rather than defaulted. A default that moves is a
  // legitimate product decision; it must not silently retune this test's
  // bounds (STANDARDS §4/§6). These are the pipeline defaults except for a
  // 2 cm voxel and no pixel subsampling, which buy enough points for plane
  // segmentation to resolve the individual walls -- at the 5 cm/factor-4
  // defaults ten frames yield only ~6.3 k points and 3 merged planes, too
  // coarse to notice a regression.
  reusex::geometry::ReconstructionParams params;
  params.resolution = 0.02F;
  params.min_distance = 0.0F;
  params.max_distance = 4.0F;
  params.sampling_factor = 1;
  params.confidence_threshold = 2;

  reusex::geometry::reconstruct_point_clouds(db, params);

  const auto cloud = db.point_cloud_xyzrgb("cloud");
  const auto normals = db.point_cloud_normal("normals");
  REQUIRE(cloud != nullptr);
  REQUIRE(normals != nullptr);
  REQUIRE(cloud->size() == normals->size());

  // Reference: 68761 points. ±5 % -- a depth/confidence filter or voxel-size
  // regression moves this far further than that.
  INFO("reconstructed points: " << cloud->size());
  CHECK(cloud->size() > 65'300);
  CHECK(cloud->size() < 72'200);

  // The ten frames walk ~1.2 m along a corridor with a 4 m depth clip, so the
  // fused cloud is a room-sized box. The vertical extent is the corridor's
  // floor-to-ceiling height (measured 2.94 m) and is the single most useful
  // number here: it is wrong by a factor, or collapses, the moment the pose
  // convention, the intrinsics or the depth scaling regress.
  reusex::PointT bb_min;
  reusex::PointT bb_max;
  pcl::getMinMax3D(*cloud, bb_min, bb_max);
  const float extent_x = bb_max.x - bb_min.x;
  const float extent_y = bb_max.y - bb_min.y;
  const float extent_z = bb_max.z - bb_min.z;
  INFO("bbox extent: " << extent_x << " x " << extent_y << " x " << extent_z);
  CHECK(extent_z > 2.6F);
  CHECK(extent_z < 3.3F);
  CHECK(extent_x > 3.0F);
  CHECK(extent_x < 4.6F);
  CHECK(extent_y > 3.2F);
  CHECK(extent_y < 4.8F);

  // ── Stage 2: planar segmentation ────────────────────────────────────────
  reusex::geometry::SegmentPlanesOptions plane_options;
  plane_options.noise_seed = 42; // STANDARDS §6
  const auto [plane_labels, plane_centroids, plane_normals] =
      reusex::geometry::segment_planes(cloud, normals, plane_options);

  REQUIRE(plane_labels != nullptr);
  REQUIRE(plane_normals != nullptr);
  REQUIRE(plane_labels->size() == cloud->size());

  const auto hist = label_histogram(*plane_labels);
  REQUIRE(plane_normals->size() == hist.size());

  // Reference: 5 planes -- floor, ceiling and three wall facets (the corridor
  // wall, its window reveals, and the end wall). ±1 is real slack: below 4
  // segmentation has collapsed and merged distinct surfaces, above 6 it is
  // fragmenting.
  INFO("planes found: " << hist.size());
  CHECK(hist.size() >= 4);
  CHECK(hist.size() <= 6);

  std::size_t labeled = 0;
  for (const auto &[label, count] : hist) {
    labeled += count;
  }
  const double labeled_fraction =
      static_cast<double>(labeled) / static_cast<double>(cloud->size());

  // Reference: 71.6 %. The unlabeled remainder is clutter and the far end of
  // the corridor; a drop below 60 % means the planar surfaces themselves
  // stopped being recovered.
  INFO("labeled fraction: " << labeled_fraction);
  CHECK(labeled_fraction > 0.60);

  // The strongest assertion in this test. In the capture's world frame Z is
  // up, so a real interior must decompose into horizontal surfaces (floor,
  // ceiling) and vertical ones (walls) -- nothing in between. An axis-flipped
  // pose convention (the ARKitScenes class of bug, #224) or gross drift fans
  // the walls out and destroys this split while leaving the point count and
  // the plane count untouched.
  int horizontal = 0;
  int vertical = 0;
  for (const auto &n : plane_normals->points) {
    const float nz = std::abs(n.normal_z);
    if (nz > 0.95F) {
      ++horizontal;
    } else if (nz < 0.20F) {
      ++vertical;
    }
    INFO("plane normal z: " << n.normal_z);
    // Every plane in a corridor is one or the other; a slanted plane means
    // the reconstruction is smearing surfaces together.
    CHECK((nz > 0.95F || nz < 0.20F));
  }
  INFO("horizontal planes: " << horizontal << ", vertical: " << vertical);
  CHECK(horizontal == 2); // floor + ceiling
  CHECK(vertical >= 2);   // at least two walls

  // Floor and ceiling must be separated by the room height, independently of
  // the bbox (which outliers can inflate).
  REQUIRE(plane_centroids != nullptr);
  REQUIRE(plane_centroids->size() == plane_normals->size());
  float lowest_horizontal = std::numeric_limits<float>::max();
  float highest_horizontal = std::numeric_limits<float>::lowest();
  for (std::size_t i = 0; i < plane_normals->size(); ++i) {
    if (std::abs(plane_normals->points[i].normal_z) > 0.95F) {
      const float z = plane_centroids->points[i].z;
      lowest_horizontal = std::min(lowest_horizontal, z);
      highest_horizontal = std::max(highest_horizontal, z);
    }
  }
  const float room_height = highest_horizontal - lowest_horizontal;
  INFO("floor-to-ceiling separation: " << room_height);
  CHECK(room_height > 2.5F);
  CHECK(room_height < 3.3F);
}
