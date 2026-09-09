// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>

#include "../../support/temp_path.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <opencv2/core.hpp>

#include <sqlite3.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <string>

using Catch::Matchers::WithinAbs;

namespace fs = std::filesystem;

namespace {

std::array<double, 16> identity16() {
  return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
}

/// A minimal but complete sensor frame: 4x4 colour image, well-formed
/// intrinsics, and whatever pose the caller wants stored.
void seed_frame(reusex::ProjectDB &db, int node_id,
                const std::array<double, 16> &pose) {
  cv::Mat color(4, 4, CV_8UC3, cv::Scalar(10, 20, 30));
  reusex::core::SensorIntrinsics intr;
  intr.fx = intr.fy = 100.0;
  intr.cx = intr.cy = 2.0;
  intr.width = intr.height = 4;
  db.save_sensor_frame(node_id, color, cv::Mat(), cv::Mat(), pose, intr);
}

/// Overwrite a frame's `transform` column with raw SQL.
///
/// This helper has to exist because no public API can produce the rows
/// `has_sensor_frame_pose()` was written to reject: `save_sensor_frame` and
/// `update_sensor_frame_pose` both bind a well-formed 128-byte blob, so a NULL
/// transform, a truncated blob, an all-zeros matrix or a NaN is unreachable
/// through C++. Real projects get exactly those rows from importers that never
/// had poses to write, which is the whole point of #330 — so the fixture has to
/// write the row directly.
///
/// Pass `data == nullptr` to store SQL NULL. The caller must have closed its
/// `ProjectDB` first: the database runs in WAL mode, so reopening after the
/// write is what makes the change visible to a new connection.
void write_raw_transform(const fs::path &db_path, int node_id, const void *data,
                         int n_bytes) {
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(db_path.string().c_str(), &raw) == SQLITE_OK);
  sqlite3_stmt *stmt = nullptr;
  const char *sql = "UPDATE sensor_frames SET transform = ? WHERE node_id = ?;";
  REQUIRE(sqlite3_prepare_v2(raw, sql, -1, &stmt, nullptr) == SQLITE_OK);
  if (data)
    sqlite3_bind_blob(stmt, 1, data, n_bytes, SQLITE_TRANSIENT);
  else
    sqlite3_bind_null(stmt, 1);
  sqlite3_bind_int(stmt, 2, node_id);
  const int rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);
  const int changed = sqlite3_changes(raw);
  sqlite3_close(raw);
  REQUIRE(rc == SQLITE_DONE);
  REQUIRE(changed == 1);
}

void clear_sensor_frame_pose(const fs::path &db_path, int node_id) {
  write_raw_transform(db_path, node_id, nullptr, 0);
}

void set_raw_sensor_frame_pose(const fs::path &db_path, int node_id,
                               const std::array<double, 16> &m) {
  write_raw_transform(db_path, node_id, m.data(),
                      static_cast<int>(m.size() * sizeof(double)));
}

/// Seed one frame with a valid pose, then close the project so a raw-SQL
/// mutation is visible to the connection opened next.
void seed_and_close(const fs::path &db_path, int node_id) {
  reusex::ProjectDB db(db_path);
  seed_frame(db, node_id, identity16());
}

void require_identity_pose(const reusex::ProjectDB &db, int node_id) {
  const auto pose = db.sensor_frame_pose(node_id);
  const auto expected = identity16();
  for (std::size_t i = 0; i < 16; ++i)
    REQUIRE_THAT(pose[i], WithinAbs(expected[i], 1e-12));
}

} // namespace

TEST_CASE("UpdateSensorFramePose_ExistingAndMissingFrame_"
          "RoundTripsPoseAndThrowsOnMissingId",
          "[io][db]") {
  namespace fs = std::filesystem;
  fs::path db_path = fs::temp_directory_path() / "reusex_update_pose_test.rux";
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
                                   0, 0,  1, 3.5, 0, 0, 0, 1};
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

// ===========================================================================
// has_sensor_frame_pose (#330)
// ===========================================================================
//
// `sensor_frame_pose()` returns identity for a frame that has no pose at all,
// so consumers (gsplat's TrainingViews, the COLMAP exporter) could not tell a
// poseless frame from one legitimately at the world origin and silently placed
// a camera there. `has_sensor_frame_pose()` is the accessor that answers the
// question; these cases pin down both halves of its contract — what it must
// reject, and the identity pose it must NOT reject.

TEST_CASE("HasSensorFramePose_FrameSavedWithNonIdentityPose_ReturnsTrue",
          "[io][db]") {
  reusex::test_support::TempPath tmp("test_has_pose");
  reusex::ProjectDB db(tmp.path);

  const std::array<double, 16> pose = {0, -1, 0, 1.5, 1, 0, 0, -2.5,
                                       0, 0,  1, 3.5, 0, 0, 0, 1};
  seed_frame(db, 7, pose);

  REQUIRE(db.has_sensor_frame_pose(7));
}

TEST_CASE("HasSensorFramePose_FrameSavedWithIdentityPose_ReturnsTrue",
          "[io][db]") {
  // The regression that would make the whole #330 fix wrong: a scan may
  // legitimately place a frame at the origin with no rotation, and an identity
  // matrix that was actually written IS a pose. Only the absence of a usable
  // blob means "no pose".
  reusex::test_support::TempPath tmp("test_has_pose");
  reusex::ProjectDB db(tmp.path);

  seed_frame(db, 7, identity16());

  REQUIRE(db.has_sensor_frame_pose(7));
}

TEST_CASE("HasSensorFramePose_NullTransformBlob_ReturnsFalse", "[io][db]") {
  reusex::test_support::TempPath tmp("test_has_pose");
  seed_and_close(tmp.path, 7);
  clear_sensor_frame_pose(tmp.path, 7);

  reusex::ProjectDB db(tmp.path);
  REQUIRE(db.has_sensor_frame(7));            // the row is still there...
  REQUIRE_FALSE(db.has_sensor_frame_pose(7)); // ...it just has no pose.
}

TEST_CASE("HasSensorFramePose_AllZeroTransform_ReturnsFalse", "[io][db]") {
  // The placeholder an importer writes when it has the column but not the
  // data. The bottom row is [0 0 0 0], and even if it were [0 0 0 1] the
  // rotation block is singular — det(R) == 0.
  reusex::test_support::TempPath tmp("test_has_pose");
  seed_and_close(tmp.path, 7);

  std::array<double, 16> zeros{};
  SECTION("entirely zero") { /* zeros as constructed */ }
  SECTION("zero rotation block with a well-formed bottom row") {
    zeros[15] = 1.0;
  }
  set_raw_sensor_frame_pose(tmp.path, 7, zeros);

  reusex::ProjectDB db(tmp.path);
  REQUIRE_FALSE(db.has_sensor_frame_pose(7));
}

TEST_CASE("HasSensorFramePose_NonFiniteTransform_ReturnsFalse", "[io][db]") {
  // NaN and Inf survive the BLOB round trip untouched and would poison every
  // downstream matrix product without ever throwing.
  reusex::test_support::TempPath tmp("test_has_pose");
  seed_and_close(tmp.path, 7);

  auto pose = identity16();
  SECTION("NaN in the rotation block") {
    pose[0] = std::numeric_limits<double>::quiet_NaN();
  }
  SECTION("NaN in the translation") {
    pose[7] = std::numeric_limits<double>::quiet_NaN();
  }
  SECTION("+Inf in the translation") {
    pose[3] = std::numeric_limits<double>::infinity();
  }
  SECTION("-Inf in the rotation block") {
    pose[5] = -std::numeric_limits<double>::infinity();
  }
  set_raw_sensor_frame_pose(tmp.path, 7, pose);

  reusex::ProjectDB db(tmp.path);
  REQUIRE_FALSE(db.has_sensor_frame_pose(7));
}

TEST_CASE("HasSensorFramePose_MalformedTransformBlob_ReturnsFalse",
          "[io][db]") {
  reusex::test_support::TempPath tmp("test_has_pose");
  seed_and_close(tmp.path, 7);

  SECTION("blob shorter than 16 doubles") {
    const auto pose = identity16();
    write_raw_transform(tmp.path, 7, pose.data(),
                        static_cast<int>(12 * sizeof(double)));
  }
  SECTION("blob longer than 16 doubles") {
    std::array<double, 20> padded{};
    const auto pose = identity16();
    for (std::size_t i = 0; i < 16; ++i)
      padded[i] = pose[i];
    write_raw_transform(tmp.path, 7, padded.data(),
                        static_cast<int>(padded.size() * sizeof(double)));
  }
  SECTION("bottom row is not [0 0 0 1]") {
    // Not an affine 4x4 at all — a garbled or differently laid-out blob.
    auto pose = identity16();
    pose[12] = 0.4;
    pose[15] = 2.0;
    set_raw_sensor_frame_pose(tmp.path, 7, pose);
  }

  reusex::ProjectDB db(tmp.path);
  REQUIRE_FALSE(db.has_sensor_frame_pose(7));
}

TEST_CASE("HasSensorFramePose_UnknownNodeId_ReturnsFalse", "[io][db]") {
  reusex::test_support::TempPath tmp("test_has_pose");
  reusex::ProjectDB db(tmp.path);
  seed_frame(db, 7, identity16());

  REQUIRE_FALSE(db.has_sensor_frame(999));
  REQUIRE_FALSE(db.has_sensor_frame_pose(999));
}

TEST_CASE("SensorFramePose_UnreadableStoredPose_StillReturnsIdentity",
          "[io][db]") {
  // The legacy fallback is unchanged and load-bearing: callers that do not
  // care about the distinction still get a usable matrix rather than an
  // exception. #330 adds a way to ask, it does not change what
  // sensor_frame_pose() answers.
  //
  // Note the scope: the fallback fires only when there is nothing to read —
  // no row, a NULL blob, or a blob that is not 16 doubles. A blob of the
  // RIGHT SIZE is memcpy'd out verbatim however meaningless its contents,
  // which is the sibling case below.
  reusex::test_support::TempPath tmp("test_has_pose");
  seed_and_close(tmp.path, 7);

  int node_id = 7;
  SECTION("NULL transform") { clear_sensor_frame_pose(tmp.path, 7); }
  SECTION("truncated blob") {
    const auto pose = identity16();
    write_raw_transform(tmp.path, 7, pose.data(),
                        static_cast<int>(3 * sizeof(double)));
  }
  SECTION("no such row") { node_id = 999; }

  reusex::ProjectDB db(tmp.path);
  REQUIRE_FALSE(db.has_sensor_frame_pose(node_id));
  require_identity_pose(db, node_id);
}

TEST_CASE("SensorFramePose_WellSizedButInvalidPose_ReturnsStoredValueVerbatim",
          "[io][db]") {
  // This is the case that makes has_sensor_frame_pose() necessary rather than
  // merely tidy. An all-zero or NaN transform is exactly 128 bytes, so the
  // reader has nothing to reject it by and hands it straight back — and every
  // caller that multiplies it gets a collapsed or NaN camera with no error
  // anywhere. The value is asserted here so that a future change which starts
  // sanitising it has to come past this test rather than quietly altering what
  // a decade of callers receive.
  reusex::test_support::TempPath tmp("test_has_pose");
  seed_and_close(tmp.path, 7);

  std::array<double, 16> stored{};
  SECTION("all-zero transform") { stored = std::array<double, 16>{}; }
  SECTION("NaN transform") {
    stored = identity16();
    stored[0] = std::numeric_limits<double>::quiet_NaN();
  }
  set_raw_sensor_frame_pose(tmp.path, 7, stored);

  reusex::ProjectDB db(tmp.path);
  // Rejected as a pose...
  REQUIRE_FALSE(db.has_sensor_frame_pose(7));
  // ...but returned unchanged by the legacy accessor.
  const auto pose = db.sensor_frame_pose(7);
  for (std::size_t i = 0; i < 16; ++i) {
    if (std::isnan(stored[i]))
      REQUIRE(std::isnan(pose[i]));
    else
      REQUIRE_THAT(pose[i], WithinAbs(stored[i], 1e-12));
  }
}
