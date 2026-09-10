// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// GPU tests for `run_gsplat_stage` — the seam between the training loop and
// the project (#322).
//
// `test_train.cpp` covers the optimizer; what is checked here is the part a
// user notices: after a run, is the trained model actually IN the `.rux`, is
// it the same model that `--out` wrote, and does a run refuse before spending
// GPU minutes when it has nowhere to put the result. Those became worth
// testing when the splat stopped being a file the stage dropped on disk and
// became project state.
//
// Tagged [gpu] with a `has_cuda_device()` guard like the rest of this
// directory: the stage runs the real rasterizer, so a CUDA-less machine
// reports these as skipped rather than failed.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "../../../support/temp_path.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>
#include <reusex/gsplat/train.hpp>

#include <opencv2/core.hpp>

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

using namespace reusex;
using Catch::Matchers::ContainsSubstring;
using reusex::test_support::TempDir;

namespace {

std::array<double, 16> pose_at(double x, double y, double z) {
  std::array<double, 16> p{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  p[3] = x;
  p[7] = y;
  p[11] = z;
  return p;
}

core::SensorIntrinsics tiny_intrinsics() {
  core::SensorIntrinsics intr;
  intr.fx = intr.fy = 24.0;
  intr.cx = 16.0;
  intr.cy = 12.0;
  intr.width = 32;
  intr.height = 24;
  intr.local_transform = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  return intr;
}

/// The smallest project the stage's input contract accepts: a seed cloud and
/// a handful of posed colour frames looking at it from different places.
void seed_project(ProjectDB &db) {
  Cloud cloud;
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      for (int k = 0; k < 6; ++k) {
        PointT p;
        p.x = -0.2F + 0.08F * static_cast<float>(i);
        p.y = -0.2F + 0.08F * static_cast<float>(j);
        p.z = -0.2F + 0.08F * static_cast<float>(k);
        p.r = 220;
        p.g = 120;
        p.b = 40;
        cloud.push_back(p);
      }
  cloud.width = cloud.size();
  cloud.height = 1;
  db.save_point_cloud("cloud", cloud, "test");

  const auto intr = tiny_intrinsics();
  const cv::Mat image(intr.height, intr.width, CV_8UC3, cv::Vec3b{40, 90, 160});
  for (int i = 0; i < 4; ++i)
    db.save_sensor_frame(i, image, cv::Mat(), cv::Mat(),
                         pose_at(0.6 * i - 0.9, -1.5, 0.0), intr);
}

gsplat::GsplatStageOptions tiny_stage_options() {
  gsplat::GsplatStageOptions opt;
  opt.init.max_points = 200;
  opt.train.iterations = 2;
  // Every view trains; a held-out split of four views leaves too few to be
  // meaningful and is not what these cases are about.
  opt.train.holdout_every = 0;
  return opt;
}

std::vector<std::uint8_t> read_bytes(const std::filesystem::path &path) {
  std::ifstream in(path, std::ios::binary);
  REQUIRE(in.good());
  return std::vector<std::uint8_t>((std::istreambuf_iterator<char>(in)),
                                   std::istreambuf_iterator<char>());
}

} // namespace

TEST_CASE("GsplatStage_CompletedRun_StoresTheSplatInTheProject",
          "[gsplat][gpu]") {
  if (!gsplat::has_cuda_device())
    SKIP("no CUDA device available");

  TempDir dir("test_gsplat_stage");
  ProjectDB db(dir.path / "project.rux", /*readOnly=*/false);
  seed_project(db);

  auto opt = tiny_stage_options();
  const auto result = gsplat::run_gsplat_stage(db, opt);
  REQUIRE(result.gaussians.size() > 0);

  // The stage's product is a project row, not a file. Nothing else in this
  // test writes one, so this is the whole output of the run.
  REQUIRE(db.has_gaussian_splat("splat"));
  const auto metadata = db.gaussian_splat_metadata("splat");
  CHECK(metadata.format == "ply");
  CHECK(metadata.gaussian_count == result.gaussians.size());
  CHECK(metadata.sh_degree == result.gaussians.sh_degree);
  CHECK(metadata.stage == "gsplat");
  // The parameters are what `rux log` shows for the run, so they have to name
  // the settings that shaped it rather than being an empty object.
  CHECK_THAT(metadata.parameters, ContainsSubstring("\"iterations\":2"));
  CHECK_THAT(metadata.parameters, ContainsSubstring("\"name\":\"splat\""));
}

TEST_CASE("GsplatStage_CustomName_StoresUnderThatName", "[gsplat][gpu]") {
  if (!gsplat::has_cuda_device())
    SKIP("no CUDA device available");

  TempDir dir("test_gsplat_stage");
  ProjectDB db(dir.path / "project.rux", /*readOnly=*/false);
  seed_project(db);

  auto opt = tiny_stage_options();
  opt.splat_name = "detailed";
  gsplat::run_gsplat_stage(db, opt);

  // `--name` overrides the contract's declared `splat`, exactly as
  // `--seed-cloud` overrides `cloud` (docs/CONTRACTS.md).
  CHECK(db.has_gaussian_splat("detailed"));
  CHECK_FALSE(db.has_gaussian_splat("splat"));
}

TEST_CASE("GsplatStage_OutPlyGiven_WritesTheSameBytesItStored",
          "[gsplat][gpu]") {
  if (!gsplat::has_cuda_device())
    SKIP("no CUDA device available");

  TempDir dir("test_gsplat_stage");
  ProjectDB db(dir.path / "project.rux", /*readOnly=*/false);
  seed_project(db);

  auto opt = tiny_stage_options();
  opt.out_ply = dir.path / "exported.ply";
  gsplat::run_gsplat_stage(db, opt);

  // `--out` is an export of the stored model, not a second serialization of
  // it. If these could differ, a project's splat and the .ply a user handed to
  // someone else would be different models under one name.
  REQUIRE(std::filesystem::exists(opt.out_ply));
  CHECK(read_bytes(opt.out_ply) == db.gaussian_splat_blob("splat"));
}

TEST_CASE("GsplatStage_EmptyName_RefusesBeforeTraining", "[gsplat][gpu]") {
  if (!gsplat::has_cuda_device())
    SKIP("no CUDA device available");

  TempDir dir("test_gsplat_stage");
  ProjectDB db(dir.path / "project.rux", /*readOnly=*/false);
  seed_project(db);

  auto opt = tiny_stage_options();
  opt.splat_name.clear();

  // A blank name is the one way this stage could still run for minutes and
  // finish with nothing to show for itself, so it is refused up front
  // (STANDARDS §5) rather than after the GPU time is spent.
  REQUIRE_THROWS_WITH(gsplat::run_gsplat_stage(db, opt),
                      ContainsSubstring("--name"));
  CHECK(db.list_gaussian_splats().empty());
}

TEST_CASE("GsplatStage_RerunWithSameName_ReplacesThePreviousSplat",
          "[gsplat][gpu]") {
  if (!gsplat::has_cuda_device())
    SKIP("no CUDA device available");

  TempDir dir("test_gsplat_stage");
  ProjectDB db(dir.path / "project.rux", /*readOnly=*/false);
  seed_project(db);

  auto opt = tiny_stage_options();
  opt.init.max_points = 100;
  gsplat::run_gsplat_stage(db, opt);
  const auto first = db.gaussian_splat_metadata("splat");

  // Retraining is the normal way a user iterates, and it must leave one splat
  // under that name rather than two rows or a stale blob.
  opt.init.max_points = 200;
  gsplat::run_gsplat_stage(db, opt);
  const auto second = db.gaussian_splat_metadata("splat");

  CHECK(db.list_gaussian_splats().size() == 1);
  CHECK(second.gaussian_count > first.gaussian_count);
  CHECK(db.gaussian_splat_blob("splat").size() == second.byte_size);
}
