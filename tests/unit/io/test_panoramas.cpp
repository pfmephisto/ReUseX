// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/panoramas.hpp>

#include <catch2/catch_test_macros.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "../../support/temp_path.hpp"

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using reusex::ProjectDB;
using reusex::io::import_panoramas;
using reusex::io::ImportPanoramasOptions;
using reusex::test_support::TempDir;

namespace {

/// Write a minimal JPEG (cv::imencode) to path. rows×cols determines the
/// aspect ratio: use 2 cols for 1 row to produce a 2:1 equirect image.
void write_jpeg(const fs::path &path, int rows, int cols) {
  cv::Mat img(rows, cols, CV_8UC3, cv::Scalar(128, 64, 200));
  std::vector<uint8_t> buf;
  cv::imencode(".jpg", img, buf);
  std::ofstream f(path, std::ios::binary);
  f.write(reinterpret_cast<const char *>(buf.data()),
          static_cast<std::streamsize>(buf.size()));
}

} // namespace

// ─── Equirect guard ─────────────────────────────────────────────────────────

TEST_CASE("ImportPanoramas_EquirectJpeg_IsImported", "[io][panoramas]") {
  TempDir tmp("test_pano_equirect");
  const auto scan_dir = tmp.path / "scan";
  fs::create_directories(scan_dir / "360");

  // 4 cols × 2 rows: width == 2 * height → equirect.
  write_jpeg(scan_dir / "360" / "pano.jpg", 2, 4);

  ProjectDB db(tmp.path / "project.rux");
  const auto count = import_panoramas(db, scan_dir);

  CHECK(count == 1);
  CHECK(db.panoramic_image_count() == 1);
}

TEST_CASE("ImportPanoramas_NonEquirectJpeg_IsSkipped", "[io][panoramas]") {
  TempDir tmp("test_pano_nonequirect");
  const auto scan_dir = tmp.path / "scan";
  fs::create_directories(scan_dir / "360");

  // 4 cols × 3 rows: 4 ≠ 2*3 → not equirect.
  write_jpeg(scan_dir / "360" / "frame.jpg", 3, 4);

  ProjectDB db(tmp.path / "project.rux");
  const auto count = import_panoramas(db, scan_dir);

  CHECK(count == 0);
  CHECK(db.panoramic_image_count() == 0);
}

TEST_CASE("ImportPanoramas_MixedAspectRatios_OnlyEquirectImported",
          "[io][panoramas]") {
  TempDir tmp("test_pano_mixed");
  const auto scan_dir = tmp.path / "scan";
  fs::create_directories(scan_dir / "360");

  write_jpeg(scan_dir / "360" / "equirect.jpg", 2, 4); // 2:1 → keep
  write_jpeg(scan_dir / "360" / "square.jpg", 4, 4);   // 1:1 → skip
  write_jpeg(scan_dir / "360" / "frame.jpg", 3, 4);    // 4:3 → skip

  ProjectDB db(tmp.path / "project.rux");
  const auto count = import_panoramas(db, scan_dir);

  CHECK(count == 1);
  CHECK(db.panoramic_image_count() == 1);
  auto panos = db.list_panoramic_images();
  REQUIRE(panos.size() == 1);
  CHECK(panos[0].filename == "equirect.jpg");
}

// ─── Subdir discovery ───────────────────────────────────────────────────────

TEST_CASE("ImportPanoramas_KnownSubdir_ScannedFirst", "[io][panoramas]") {
  TempDir tmp("test_pano_subdir");
  const auto scan_dir = tmp.path / "scan";
  fs::create_directories(scan_dir / "panoramas");

  write_jpeg(scan_dir / "panoramas" / "pano.jpg", 2, 4);

  ProjectDB db(tmp.path / "project.rux");
  const auto count = import_panoramas(db, scan_dir);

  CHECK(count == 1);
}

TEST_CASE("ImportPanoramas_NoKnownSubdir_FallsBackToRootDir",
          "[io][panoramas]") {
  TempDir tmp("test_pano_rootfallback");
  const auto scan_dir = tmp.path / "scan";
  fs::create_directory(scan_dir);

  // Equirect in the root (no 360/, panoramas/, or pano/ subdir present).
  write_jpeg(scan_dir / "equirect.jpg", 2, 4);

  ProjectDB db(tmp.path / "project.rux");
  const auto count = import_panoramas(db, scan_dir);

  CHECK(count == 1);
}

TEST_CASE("ImportPanoramas_CustomSubdirHint_IsRespected", "[io][panoramas]") {
  TempDir tmp("test_pano_customhint");
  const auto scan_dir = tmp.path / "scan";
  fs::create_directories(scan_dir / "custom_panos");
  // Standard hints don't exist — only our custom one.
  write_jpeg(scan_dir / "custom_panos" / "pano.jpg", 2, 4);

  ImportPanoramasOptions opts;
  opts.subdir_hints = {"custom_panos"};

  ProjectDB db(tmp.path / "project.rux");
  const auto count = import_panoramas(db, scan_dir, opts);

  CHECK(count == 1);
}

// ─── Skip flag ──────────────────────────────────────────────────────────────

TEST_CASE("ImportPanoramas_SkipTrue_ReturnsZeroWithoutTouchingDB",
          "[io][panoramas]") {
  TempDir tmp("test_pano_skip");
  const auto scan_dir = tmp.path / "scan";
  fs::create_directories(scan_dir / "360");
  write_jpeg(scan_dir / "360" / "pano.jpg", 2, 4);

  ImportPanoramasOptions opts;
  opts.skip = true;

  ProjectDB db(tmp.path / "project.rux");
  const auto count = import_panoramas(db, scan_dir, opts);

  CHECK(count == 0);
  CHECK(db.panoramic_image_count() == 0);
}

// ─── Empty directory ────────────────────────────────────────────────────────

TEST_CASE("ImportPanoramas_NoJpegsInDir_ReturnsZero", "[io][panoramas]") {
  TempDir tmp("test_pano_empty");
  const auto scan_dir = tmp.path / "scan";
  fs::create_directory(scan_dir);

  ProjectDB db(tmp.path / "project.rux");
  const auto count = import_panoramas(db, scan_dir);

  CHECK(count == 0);
  CHECK(db.panoramic_image_count() == 0);
}

// ─── Idempotency ────────────────────────────────────────────────────────────

TEST_CASE("ImportPanoramas_ImportTwice_SecondCallIsIdempotent",
          "[io][panoramas]") {
  TempDir tmp("test_pano_idempotent");
  const auto scan_dir = tmp.path / "scan";
  fs::create_directories(scan_dir / "360");
  write_jpeg(scan_dir / "360" / "pano.jpg", 2, 4);

  ProjectDB db(tmp.path / "project.rux");

  const auto first = import_panoramas(db, scan_dir);
  const auto second = import_panoramas(db, scan_dir);

  CHECK(first == 1);
  CHECK(second == 0); // already imported; has_panoramic_image skips it
  CHECK(db.panoramic_image_count() == 1);
}

// ─── Unlinked import (no EXIF, no sensor frames) ───────────────────────────

TEST_CASE("ImportPanoramas_NoExifAndNoSensorFrames_ImportedUnlinked",
          "[io][panoramas]") {
  TempDir tmp("test_pano_unlinked");
  const auto scan_dir = tmp.path / "scan";
  fs::create_directories(scan_dir / "360");
  // cv::imencode produces no EXIF → read_exif_timestamp returns -1.0.
  write_jpeg(scan_dir / "360" / "pano.jpg", 2, 4);

  ProjectDB db(tmp.path / "project.rux");
  const auto count = import_panoramas(db, scan_dir);

  CHECK(count == 1);
  auto panos = db.list_panoramic_images();
  REQUIRE(panos.size() == 1);
  // No EXIF timestamp and no sensor frames → stored unlinked (node_id = -1).
  CHECK(panos[0].node_id == -1);
  CHECK(panos[0].has_pose == false);
}
