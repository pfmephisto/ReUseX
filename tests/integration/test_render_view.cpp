// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Headless rendering integration test (#294).
//
// What this asserts, and deliberately does not:
//
//   - It DOES assert that render_view() returns an image of the requested size
//     in which a substantial share of the pixels are not the background. That
//     is the regression this test exists to catch: a render pipeline that
//     "succeeds" and hands back a black frame — a missing OpenGL factory, an
//     off-screen context that never got a surface, a camera pointed at nothing.
//     Every one of those failures is invisible to an exit code and obvious in
//     one pixel statistic.
//   - It DOES assert that two identical calls produce byte-identical images
//     (STANDARDS §6), which is what makes rendered artifacts usable for
//     before/after comparison.
//   - It does NOT compare against a golden image. Point rasterization differs
//     across GPUs and driver versions, so a golden file would fail for reasons
//     unrelated to the code under test.
//
// The two rendering tests skip themselves when the machine has neither a DRM
// render node nor an X/Wayland session — a GitHub-hosted CI runner, or a nix
// build sandbox, which does not bind-mount /dev/dri.
//
// That is not a retreat from the point of #294. "Renders with no display" is
// still asserted in full: on any machine with a GPU these tests run with
// DISPLAY unset and exercise the EGL fallback, which is the case the feature
// exists for. "No rendering device at all" is simply outside what VTK can do,
// and it currently segfaults rather than reporting the failure (see the TODO
// in libs/reusex/src/visualize/render_view.cpp) — a crash that would mask
// every regression these tests exist to catch. Remove the guard once
// render_view() diagnoses that condition properly.
//
// The error-path test is deliberately left unguarded: every case it covers is
// rejected before any GL work happens, so it keeps its value on a device-less
// runner.

#include "../support/temp_path.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/segmentation/reconstruct.hpp>
#include <reusex/types/point_types.hpp>
#include <reusex/visualize/render_view.hpp>

#include <opencv2/core.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

namespace fs = std::filesystem;
namespace viz = reusex::visualize;

namespace {

/// True when VTK has some way to create a render window: a DRM render node for
/// the off-screen EGL path, or an X/Wayland session to fall back on.
///
/// Checked before calling render_view() rather than around it, because VTK does
/// not fail gracefully when neither exists — it segfaults, which no amount of
/// exception handling here can catch.
bool has_rendering_device() {
  for (const char *var : {"DISPLAY", "WAYLAND_DISPLAY"}) {
    const char *value = std::getenv(var);
    if (value != nullptr && value[0] != '\0') {
      return true;
    }
  }

  // The error_code overload yields an empty range instead of throwing when
  // /dev/dri does not exist at all, which is exactly the sandbox case.
  std::error_code ec;
  for (const auto &entry : fs::directory_iterator("/dev/dri", ec)) {
    if (entry.path().filename().string().starts_with("renderD")) {
      return true;
    }
  }
  return false;
}

/// Message shared by both rendering tests, so the reason reads the same way.
constexpr const char *kNoDeviceReason =
    "No DRM render node (/dev/dri/renderD*) and no X/Wayland display; VTK "
    "cannot create a render window.";

/// tests/integration/<this file> -> tests/fixtures/scans/office_corridor.rux
fs::path fixture_path() {
  return fs::path(__FILE__).parent_path().parent_path() / "fixtures" / "scans" /
         "office_corridor.rux";
}

/// True when `p` is an unexpanded git-lfs pointer rather than the real blob.
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

/// Fraction of pixels that differ noticeably from the render background.
///
/// Anti-aliased point edges blend toward the background, so "not background"
/// is a tolerance, not equality.
double foreground_fraction(const cv::Mat &image,
                           const std::array<double, 3> &background) {
  REQUIRE(image.type() == CV_8UC3);
  const cv::Vec3b bg(static_cast<unsigned char>(background[2] * 255.0 + 0.5),
                     static_cast<unsigned char>(background[1] * 255.0 + 0.5),
                     static_cast<unsigned char>(background[0] * 255.0 + 0.5));

  std::size_t foreground = 0;
  for (int y = 0; y < image.rows; ++y) {
    const auto *row = image.ptr<cv::Vec3b>(y);
    for (int x = 0; x < image.cols; ++x) {
      const int db = std::abs(int(row[x][0]) - int(bg[0]));
      const int dg = std::abs(int(row[x][1]) - int(bg[1]));
      const int dr = std::abs(int(row[x][2]) - int(bg[2]));
      if (db + dg + dr > 12) {
        ++foreground;
      }
    }
  }
  return static_cast<double>(foreground) /
         static_cast<double>(image.rows * image.cols);
}

/// A hollow box of points, so a render has something with extent in all axes.
reusex::CloudPtr synthetic_room(int per_side = 60) {
  auto cloud = std::make_shared<reusex::Cloud>();
  const float step = 2.0F / static_cast<float>(per_side - 1);
  for (int i = 0; i < per_side; ++i) {
    for (int j = 0; j < per_side; ++j) {
      const float u = -1.0F + step * static_cast<float>(i);
      const float v = -1.0F + step * static_cast<float>(j);
      for (const float w : {-1.0F, 1.0F}) {
        reusex::PointT p;
        p.x = u, p.y = v, p.z = w;
        p.r = 220, p.g = 180, p.b = 90;
        cloud->push_back(p);
        p.x = u, p.y = w, p.z = v;
        cloud->push_back(p);
      }
    }
  }
  cloud->width = cloud->size();
  cloud->height = 1;
  return cloud;
}

} // namespace

TEST_CASE("render_view draws a synthetic project with no display",
          "[integration][render]") {
  if (!has_rendering_device()) {
    SKIP(kNoDeviceReason);
  }

  const reusex::test_support::TempDir work("reusex_render_synthetic");
  const fs::path project = work.path / "synthetic.rux";

  reusex::ProjectDB db(project);
  REQUIRE(db.is_open());

  const auto cloud = synthetic_room();
  REQUIRE(cloud->size() > 1000);
  db.save_point_cloud("cloud", *cloud);

  viz::RenderOptions opts;
  opts.layers = {viz::Layer::cloud};
  opts.width = 480;
  opts.height = 360;

  // ── The image has the requested shape and is not blank ──────────────────
  const cv::Mat top = viz::render_view(db, opts);
  CHECK(top.cols == opts.width);
  CHECK(top.rows == opts.height);
  CHECK(top.type() == CV_8UC3);

  const double covered = foreground_fraction(top, opts.background);
  INFO("foreground fraction (top): " << covered);
  CHECK(covered > 0.01);

  // ── Repeating the call reproduces the frame exactly (STANDARDS §6) ──────
  const cv::Mat again = viz::render_view(db, opts);
  cv::Mat difference;
  cv::absdiff(top, again, difference);
  CHECK(cv::countNonZero(difference.reshape(1)) == 0);

  // ── An orbit view of the same box is also non-blank ─────────────────────
  opts.view = viz::ViewPreset::orbit;
  opts.orbit_count = 4;
  opts.orbit_index = 1;
  const cv::Mat orbit = viz::render_view(db, opts);
  INFO("foreground fraction (orbit): " << foreground_fraction(orbit,
                                                              opts.background));
  CHECK(foreground_fraction(orbit, opts.background) > 0.01);

  // ── Label colouring actually recolours the geometry ─────────────────────
  //
  // Rendering the same points through the `planes` layer must not produce the
  // same pixels as their stored RGB, or the label layer is a no-op.
  auto labels = std::make_shared<reusex::CloudL>();
  labels->resize(cloud->size());
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    labels->points[i].label = static_cast<std::uint32_t>(1 + (i % 3));
  }
  labels->width = labels->size();
  labels->height = 1;
  db.save_point_cloud("planes", *labels);

  viz::RenderOptions label_opts;
  label_opts.layers = {viz::Layer::planes};
  label_opts.width = opts.width;
  label_opts.height = opts.height;
  const cv::Mat by_plane = viz::render_view(db, label_opts);
  CHECK(foreground_fraction(by_plane, label_opts.background) > 0.01);

  cv::absdiff(top, by_plane, difference);
  CHECK(cv::countNonZero(difference.reshape(1)) > 0);
}

TEST_CASE("render_view fails loudly on missing or invalid inputs",
          "[integration][render]") {
  const reusex::test_support::TempDir work("reusex_render_errors");
  const fs::path project = work.path / "errors.rux";

  reusex::ProjectDB db(project);
  REQUIRE(db.is_open());

  // No cloud stored yet: the message must name the stage to run (STANDARDS §5).
  viz::RenderOptions opts;
  try {
    viz::render_view(db, opts);
    FAIL("rendering an empty project should have thrown");
  } catch (const std::runtime_error &e) {
    const std::string message = e.what();
    INFO("message: " << message);
    CHECK(message.find("cloud") != std::string::npos);
    CHECK(message.find("rux create clouds") != std::string::npos);
  }

  db.save_point_cloud("cloud", *synthetic_room(8));

  // A label layer whose cloud is absent names its own producing stage.
  viz::RenderOptions rooms_opts;
  rooms_opts.layers = {viz::Layer::rooms};
  try {
    viz::render_view(db, rooms_opts);
    FAIL("rendering a missing label layer should have thrown");
  } catch (const std::runtime_error &e) {
    const std::string message = e.what();
    INFO("message: " << message);
    CHECK(message.find("rux create rooms") != std::string::npos);
  }

  // Option validation happens before any GL work.
  viz::RenderOptions empty_layers;
  empty_layers.layers.clear();
  CHECK_THROWS_AS(viz::render_view(db, empty_layers), std::runtime_error);

  viz::RenderOptions bad_size;
  bad_size.width = 0;
  CHECK_THROWS_AS(viz::render_view(db, bad_size), std::runtime_error);

  viz::RenderOptions bad_orbit;
  bad_orbit.view = viz::ViewPreset::orbit;
  bad_orbit.orbit_count = 4;
  bad_orbit.orbit_index = 4; // one past the end of the ring
  CHECK_THROWS_AS(viz::render_view(db, bad_orbit), std::runtime_error);

  // Index-aligned contract: a stale label cloud is a hard error, not a
  // silently truncated render (STANDARDS §3.2).
  auto short_labels = std::make_shared<reusex::CloudL>();
  short_labels->resize(3);
  short_labels->width = 3;
  short_labels->height = 1;
  db.save_point_cloud("rooms", *short_labels);
  try {
    viz::render_view(db, rooms_opts);
    FAIL("a mismatched label cloud should have thrown");
  } catch (const std::runtime_error &e) {
    const std::string message = e.what();
    INFO("message: " << message);
    CHECK(message.find("out of sync") != std::string::npos);
  }
}

TEST_CASE("render_view renders the real-scan fixture headlessly",
          "[integration][fixture][render]") {
  if (!has_rendering_device()) {
    SKIP(kNoDeviceReason);
  }

  const auto fixture = fixture_path();
  if (!fs::exists(fixture)) {
    SKIP("Fixture missing: " << fixture
                             << " -- run `git lfs pull` to fetch it.");
  }
  if (is_lfs_pointer(fixture)) {
    SKIP("Fixture is an unexpanded git-lfs pointer: "
         << fixture << " -- run `git lfs pull` to fetch it.");
  }

  const reusex::test_support::TempDir work("reusex_render_fixture");
  const fs::path project = work.path / "office_corridor.rux";
  fs::copy_file(fixture, project);

  reusex::ProjectDB db(project);
  REQUIRE(db.is_open());

  // Same pinned parameters as tests/integration/test_real_scan_fixture.cpp, so
  // both tests describe the same reconstruction.
  reusex::geometry::ReconstructionParams params;
  params.resolution = 0.02F;
  params.min_distance = 0.0F;
  params.max_distance = 4.0F;
  params.sampling_factor = 1;
  params.confidence_threshold = 2;
  reusex::geometry::reconstruct_point_clouds(db, params);

  const auto cloud = db.point_cloud_xyzrgb("cloud");
  REQUIRE(cloud != nullptr);
  REQUIRE_FALSE(cloud->empty());

  viz::RenderOptions opts;
  opts.layers = {viz::Layer::cloud};
  opts.width = 640;
  opts.height = 480;

  const cv::Mat plan = viz::render_view(db, opts);
  REQUIRE(plan.cols == 640);
  REQUIRE(plan.rows == 480);

  // The corridor fills a good share of an orthographic plan view. 1 % is a
  // floor, not a target: the reference run covers far more, and anything at or
  // below this threshold means the scene did not reach the framebuffer.
  const double covered = foreground_fraction(plan, opts.background);
  INFO("foreground fraction: " << covered);
  CHECK(covered > 0.01);

  opts.view = viz::ViewPreset::orbit;
  opts.orbit_count = 8;
  opts.orbit_index = 1;
  const cv::Mat orbit = viz::render_view(db, opts);
  const double orbit_covered = foreground_fraction(orbit, opts.background);
  INFO("orbit foreground fraction: " << orbit_covered);
  CHECK(orbit_covered > 0.01);
}
