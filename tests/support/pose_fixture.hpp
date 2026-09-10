// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Sensor-frame fixtures for the degenerate-pose guards (#330, #336).
//
// `ProjectDB::save_sensor_frame()` and `update_sensor_frame_pose()` both bind
// a well-formed 128-byte pose blob, so the rows these tests need — NULL
// `transform`, an all-zero transform, a NaN transform — are unreachable
// through the public C++ interface. Importers that never carried poses produce
// exactly those rows, which is the entire reason the guards exist, so the only
// honest fixture is a raw SQL write.
//
// **The caller's `ProjectDB` must be closed before any mutator here runs.**
// The database is in WAL mode; reopening afterwards is what makes the write
// visible to a new connection.
//
// Include as "../../support/pose_fixture.hpp" from tests/unit/<module>/.

#include <reusex/core/SensorIntrinsics.hpp>

#include <opencv2/core.hpp>

#include <sqlite3.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace reusex::test_support {

// ---------------------------------------------------------------------------
// Pose values
// ---------------------------------------------------------------------------

/// Row-major 4x4 identity — a *valid* pose. A scan may legitimately put a
/// frame at the origin, so nothing here may treat this as "no pose".
inline std::array<double, 16> identity16() {
  return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
}

/// Identity rotation with translation (x, y, z), row-major.
inline std::array<double, 16> translation_pose(double x, double y, double z) {
  auto p = identity16();
  p[3] = x;
  p[7] = y;
  p[11] = z;
  return p;
}

/// The all-zero transform an importer writes when it has no pose to write.
/// Well-sized, so it survives every blob-shape check and only a det(R) test
/// rejects it.
inline std::array<double, 16> zero_pose() { return std::array<double, 16>{}; }

/// A pose whose translation is NaN — the other shape a broken import leaves
/// behind. Finite bottom row, so only an isfinite() test rejects it.
inline std::array<double, 16> nan_translation_pose() {
  auto p = identity16();
  p[3] = std::numeric_limits<double>::quiet_NaN();
  return p;
}

// ---------------------------------------------------------------------------
// Frame content
// ---------------------------------------------------------------------------

inline core::SensorIntrinsics
make_intrinsics(double fx, double fy, double cx, double cy, int w, int h,
                std::array<double, 16> local = identity16()) {
  core::SensorIntrinsics intr;
  intr.fx = fx;
  intr.fy = fy;
  intr.cx = cx;
  intr.cy = cy;
  intr.width = w;
  intr.height = h;
  intr.local_transform = local;
  return intr;
}

/// A flat BGR image. Uniform by default; pass `checker` for something ORB can
/// actually find corners in.
inline cv::Mat make_color(int w, int h, cv::Vec3b fill = {40, 90, 160}) {
  return cv::Mat(h, w, CV_8UC3, fill);
}

/// A deterministic checkerboard, for tests that need real image features.
inline cv::Mat make_checker(int w, int h, int square = 8) {
  cv::Mat img(h, w, CV_8UC3, cv::Scalar(20, 20, 20));
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x)
      if (((x / square) + (y / square)) % 2 == 0)
        img.at<cv::Vec3b>(y, x) = cv::Vec3b(235, 235, 235);
  return img;
}

/// A constant-depth plane at `metres`, in the CV_16UC1 millimetre encoding
/// `ProjectDB` stores and every back-projection expects.
inline cv::Mat make_depth(int w, int h, double metres) {
  return cv::Mat(h, w, CV_16UC1,
                 cv::Scalar(static_cast<double>(
                     static_cast<std::uint16_t>(metres * 1000.0))));
}

// ---------------------------------------------------------------------------
// Raw-SQLite pose mutation
// ---------------------------------------------------------------------------

namespace detail {

/// Run `sql` against `db_path` and require it to have changed exactly one row.
/// Throws rather than using Catch2 macros so this header stays usable from any
/// test binary and from non-test code paths.
inline void exec_one(const std::filesystem::path &db_path,
                     const std::string &sql) {
  sqlite3 *raw = nullptr;
  if (sqlite3_open(db_path.string().c_str(), &raw) != SQLITE_OK) {
    const std::string msg = raw ? sqlite3_errmsg(raw) : "unknown";
    sqlite3_close(raw);
    throw std::runtime_error("pose_fixture: cannot open " + db_path.string() +
                             ": " + msg);
  }
  char *err = nullptr;
  const int rc = sqlite3_exec(raw, sql.c_str(), nullptr, nullptr, &err);
  const std::string message = err ? err : "";
  sqlite3_free(err);
  const int changed = sqlite3_changes(raw);
  sqlite3_close(raw);
  if (rc != SQLITE_OK)
    throw std::runtime_error("pose_fixture: " + sql + " failed: " + message);
  if (changed != 1)
    throw std::runtime_error("pose_fixture: " + sql + " changed " +
                             std::to_string(changed) + " rows, expected 1");
}

} // namespace detail

/// Blank a frame's pose the way an importer without poses leaves it.
inline void clear_sensor_frame_pose(const std::filesystem::path &db_path,
                                    int node_id) {
  detail::exec_one(
      db_path, "UPDATE sensor_frames SET transform = NULL WHERE node_id = " +
                   std::to_string(node_id) + ";");
}

/// Write `pose` into the `transform` column verbatim, bypassing every check
/// the public API applies. This is how an all-zero or NaN transform gets in.
inline void set_raw_sensor_frame_pose(const std::filesystem::path &db_path,
                                      int node_id,
                                      const std::array<double, 16> &pose) {
  sqlite3 *raw = nullptr;
  if (sqlite3_open(db_path.string().c_str(), &raw) != SQLITE_OK) {
    const std::string msg = raw ? sqlite3_errmsg(raw) : "unknown";
    sqlite3_close(raw);
    throw std::runtime_error("pose_fixture: cannot open " + db_path.string() +
                             ": " + msg);
  }
  sqlite3_stmt *stmt = nullptr;
  const char *sql = "UPDATE sensor_frames SET transform = ? WHERE node_id = ?;";
  if (sqlite3_prepare_v2(raw, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    const std::string msg = sqlite3_errmsg(raw);
    sqlite3_close(raw);
    throw std::runtime_error("pose_fixture: prepare failed: " + msg);
  }
  sqlite3_bind_blob(stmt, 1, pose.data(),
                    static_cast<int>(pose.size() * sizeof(double)),
                    SQLITE_TRANSIENT);
  sqlite3_bind_int(stmt, 2, node_id);
  const int rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);
  const int changed = sqlite3_changes(raw);
  sqlite3_close(raw);
  if (rc != SQLITE_DONE)
    throw std::runtime_error("pose_fixture: raw pose write failed");
  if (changed != 1)
    throw std::runtime_error("pose_fixture: raw pose write changed " +
                             std::to_string(changed) + " rows, expected 1");
}

} // namespace reusex::test_support
