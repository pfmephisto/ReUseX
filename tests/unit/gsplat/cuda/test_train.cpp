// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// GPU tests for the training loop. Every TEST_CASE here but the capability
// probe is tagged [gpu]: gsplat's rasterizer is CUDA-only (there is no CPU
// kernel), so none of them can run without a device.
//
// Each one opens with a `has_cuda_device()` guard and Catch2's SKIP(), so a
// run on a CUDA-less machine reports them as *skipped* — which is what they
// are — instead of failing and inviting the reader to exclude them by hand.
// That also makes this file safe to compile in a build where the module exists
// but the runner has no GPU (#332).
//
// The scene is synthetic and deliberately tiny — a handful of Gaussians seen
// from a few cameras — so the whole file runs in seconds and asserts on
// *behaviour* (does the loss fall, does the geometry survive a round trip)
// rather than on reconstruction quality, which needs a real capture.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "../../../support/temp_path.hpp"

#include <reusex/gsplat/GaussianCloud.hpp>
#include <reusex/gsplat/TrainingViews.hpp>
#include <reusex/gsplat/train.hpp>

#include <opencv2/core.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <vector>

using namespace reusex;
using Catch::Approx;
using reusex::test_support::TempDir;

namespace {

/// A cube of coloured points around the origin, dense enough that the
/// rasterizer has something to composite from every viewpoint.
CloudPtr make_blob(int n_per_axis = 8, float extent = 0.4f) {
  CloudPtr cloud(new Cloud);
  const float step = 2.0f * extent / std::max(n_per_axis - 1, 1);
  for (int i = 0; i < n_per_axis; ++i)
    for (int j = 0; j < n_per_axis; ++j)
      for (int k = 0; k < n_per_axis; ++k) {
        PointT p;
        p.x = -extent + i * step;
        p.y = -extent + j * step;
        p.z = -extent + k * step;
        // A position-dependent colour, so a wrong channel order or a wrong
        // pose would show up as a loss that refuses to fall.
        p.r = static_cast<uint8_t>(255 * i / std::max(n_per_axis - 1, 1));
        p.g = static_cast<uint8_t>(255 * j / std::max(n_per_axis - 1, 1));
        p.b = static_cast<uint8_t>(255 * k / std::max(n_per_axis - 1, 1));
        cloud->push_back(p);
      }
  return cloud;
}

/// A camera on a ring around the origin, looking inward.
gsplat::TrainingView make_view(int id, double angle_rad, int w = 96, int h = 72,
                               double radius = 2.0) {
  gsplat::TrainingView v;
  v.id = id;
  v.name = fmt::format("synthetic_{:03d}", id);

  v.K = Eigen::Matrix3d::Identity();
  v.K(0, 0) = w * 0.8;
  v.K(1, 1) = w * 0.8;
  v.K(0, 2) = w / 2.0;
  v.K(1, 2) = h / 2.0;

  // Camera centre on the ring; +Z of the optical frame must point at origin.
  const Eigen::Vector3d c(radius * std::cos(angle_rad), 0.0,
                          radius * std::sin(angle_rad));
  const Eigen::Vector3d fwd = (-c).normalized();
  const Eigen::Vector3d up(0, -1, 0); // y-down optical convention
  Eigen::Vector3d right = up.cross(fwd).normalized();
  Eigen::Vector3d down = fwd.cross(right).normalized();

  Eigen::Matrix3d R_wc;
  R_wc.col(0) = right;
  R_wc.col(1) = down;
  R_wc.col(2) = fwd;

  Eigen::Matrix4d T_wc = Eigen::Matrix4d::Identity();
  T_wc.block<3, 3>(0, 0) = R_wc;
  T_wc.block<3, 1>(0, 3) = c;

  v.T_cw = Eigen::Matrix4d::Identity();
  v.T_cw.block<3, 3>(0, 0) = R_wc.transpose();
  v.T_cw.block<3, 1>(0, 3) = -R_wc.transpose() * c;

  v.image = cv::Mat(h, w, CV_8UC3, cv::Scalar(40, 90, 160));
  return v;
}

/// `make_blob` plus a handful of far-flung isolated points.
///
/// The isolated points have no near neighbour, so their seeded scale saturates
/// at GaussianInitOptions::max_scale (0.5 m) while the cube's stays at its
/// 0.16 m grid spacing. That 3x separation is what lets a prune test fire on a
/// *known* subset — keying the test on opacity instead would depend on how far
/// the logits happen to travel in a few dozen iterations, which is why the
/// original 0.99-threshold version pruned nothing at all.
CloudPtr make_blob_with_outliers(int n_per_axis, int n_outliers,
                                 float outlier_radius = 3.0f) {
  CloudPtr cloud = make_blob(n_per_axis);
  for (int i = 0; i < n_outliers; ++i) {
    PointT p;
    // Corners of a cube at +/- outlier_radius: mutually distant, and distant
    // from the blob, so every one of them saturates.
    p.x = (i & 1) ? outlier_radius : -outlier_radius;
    p.y = (i & 2) ? outlier_radius : -outlier_radius;
    p.z = (i & 4) ? outlier_radius : -outlier_radius;
    p.r = p.g = p.b = 255;
    cloud->push_back(p);
  }
  return cloud;
}

std::vector<gsplat::TrainingView> make_ring(int n) {
  std::vector<gsplat::TrainingView> views;
  for (int i = 0; i < n; ++i)
    views.push_back(make_view(i, 2.0 * M_PI * i / n));
  return views;
}

/// Every file in @p dir with extension @p ext, sorted by name — which for
/// `checkpoint_iter%06d.ply` is the same as sorted by iteration.
///
/// Uses the error_code overloads throughout: the cancellation test calls this
/// from a watcher thread while the trainer is renaming files into the same
/// directory, and a transient ENOENT there must not blow up the test.
std::vector<std::filesystem::path>
files_with_extension(const std::filesystem::path &dir, const char *ext) {
  std::vector<std::filesystem::path> out;
  std::error_code ec;
  for (std::filesystem::directory_iterator it(dir, ec), end; it != end && !ec;
       it.increment(ec))
    if (it->path().extension() == ext)
      out.push_back(it->path());
  std::sort(out.begin(), out.end());
  return out;
}

/// Restores a directory's permissions on scope exit, so a failed REQUIRE
/// cannot leave a write-protected temp directory behind that TempDir's
/// destructor is then unable to remove.
struct PermissionsGuard {
  std::filesystem::path dir;
  std::filesystem::perms saved;

  PermissionsGuard(std::filesystem::path d, std::filesystem::perms s)
      : dir(std::move(d)), saved(s) {}
  PermissionsGuard(const PermissionsGuard &) = delete;
  PermissionsGuard &operator=(const PermissionsGuard &) = delete;
  ~PermissionsGuard() {
    std::error_code ec;
    std::filesystem::permissions(dir, saved,
                                 std::filesystem::perm_options::replace, ec);
  }
};

/// True when a new file really cannot be created in @p dir. Running as root
/// defeats the permission bits entirely, so the render-failure test has to ask
/// rather than assume.
bool directory_is_unwritable(const std::filesystem::path &dir) {
  const auto probe = dir / "writability_probe";
  std::ofstream f(probe);
  const bool opened = f.is_open();
  f.close();
  std::error_code ec;
  std::filesystem::remove(probe, ec);
  return !opened;
}

} // namespace

TEST_CASE("IsAvailable_BuiltWithGsplatModule_ReturnsTrue", "[gsplat]") {
  // Compiled at all means the CUDA backend was found at configure time; this
  // is the capability probe the CLI reports on.
  REQUIRE(gsplat::is_available());
}

TEST_CASE("RenderView_SeededGaussians_ProducesNonEmptyImage", "[gsplat][gpu]") {
  if (!gsplat::has_cuda_device())
    SKIP("no CUDA device available");

  auto g = gsplat::init_from_point_cloud(make_blob());
  auto views = make_ring(4);

  cv::Mat img = gsplat::render_view(g, views.front());

  REQUIRE(img.cols == views.front().width());
  REQUIRE(img.rows == views.front().height());
  REQUIRE(img.type() == CV_8UC3);
  // The blob sits in front of the camera, so something must have been
  // composited — an all-black frame means the pose or projection is wrong.
  REQUIRE(cv::countNonZero(cv::Mat(img.reshape(1))) > 0);
}

TEST_CASE("TrainGaussians_ShortRun_ReducesLoss", "[gsplat][gpu]") {
  if (!gsplat::has_cuda_device())
    SKIP("no CUDA device available");

  auto g = gsplat::init_from_point_cloud(make_blob());
  auto views = make_ring(6);

  gsplat::TrainOptions opt;
  opt.iterations = 60;
  opt.log_interval = 10;
  opt.prune_enabled = false; // isolate the optimizer from density control
  opt.lambda_dssim = 0.0f;   // pure L1, so the assertion has one cause

  auto result = gsplat::train_gaussians(g, views, opt);

  REQUIRE(result.history.size() >= 2);
  const double first = result.history.front().loss;
  const double last = result.history.back().loss;
  INFO("loss " << first << " -> " << last);
  REQUIRE(last < first);
  REQUIRE(std::isfinite(last));
  REQUIRE(result.final_count == g.size());
  REQUIRE(result.seconds > 0.0);
}

TEST_CASE("TrainGaussians_FixedSeed_ReproducesLossWithinTolerance",
          "[gsplat][gpu]") {
  if (!gsplat::has_cuda_device())
    SKIP("no CUDA device available");

  auto g = gsplat::init_from_point_cloud(make_blob(6));
  auto views = make_ring(4);

  gsplat::TrainOptions opt;
  opt.iterations = 20;
  opt.log_interval = 10;
  opt.prune_enabled = false;
  opt.seed = 7;

  auto a = gsplat::train_gaussians(g, views, opt);
  auto b = gsplat::train_gaussians(g, views, opt);

  REQUIRE(a.history.size() == b.history.size());

  // NOT bit-exact, and deliberately not asserted as such. The seeded RNG fixes
  // the *view order*, which is the part we control — but gsplat's backward
  // kernels accumulate per-Gaussian gradients with atomicAdd, and the order in
  // which blocks land is a property of the GPU scheduler, not of our seed.
  // Measured run-to-run spread on an RTX 6000 Ada is ~2.5e-6 relative after 20
  // iterations, so a 1e-6 epsilon fails about half the time.
  //
  // 1e-3 is still a meaningful assertion: a genuinely different view order
  // moves these losses by whole percent, not by parts per million.
  for (std::size_t i = 0; i < a.history.size(); ++i)
    REQUIRE(a.history[i].loss == Approx(b.history[i].loss).epsilon(1e-3));
}

TEST_CASE("TrainGaussians_PostTraining_GaussiansValidateAndStayFinite",
          "[gsplat][gpu]") {
  if (!gsplat::has_cuda_device())
    SKIP("no CUDA device available");

  auto seed = gsplat::init_from_point_cloud(make_blob(5));
  auto views = make_ring(3);

  gsplat::TrainOptions opt;
  opt.iterations = 5;
  opt.log_interval = 5;
  opt.prune_enabled = false;

  auto result = gsplat::train_gaussians(seed, views, opt);

  REQUIRE_NOTHROW(result.gaussians.validate());
  REQUIRE(result.gaussians.size() == seed.size());
  REQUIRE(result.gaussians.sh_degree == seed.sh_degree);
  for (const auto &m : result.gaussians.means) {
    REQUIRE(std::isfinite(m[0]));
    REQUIRE(std::isfinite(m[1]));
    REQUIRE(std::isfinite(m[2]));
  }
}

TEST_CASE("TrainGaussians_PruningOversizedGaussians_RemovesOutliers",
          "[gsplat][gpu]") {
  if (!gsplat::has_cuda_device())
    SKIP("no CUDA device available");

  // 216 blob Gaussians at ~0.16 m scale + 8 isolated ones saturated at 0.5 m.
  constexpr std::size_t kBlob = 6 * 6 * 6;
  constexpr std::size_t kOutliers = 8;
  auto seed = gsplat::init_from_point_cloud(make_blob_with_outliers(6, 8));
  auto views = make_ring(4);
  REQUIRE(seed.size() == kBlob + kOutliers);

  gsplat::TrainOptions opt;
  opt.iterations = 30;
  opt.log_interval = 10;
  opt.prune_enabled = true;
  opt.prune_start = 20;
  opt.prune_interval = 20; // fires exactly once, at iteration 20
  // Between the two populations, and far enough from both that 30 iterations
  // of lr_scales=5e-3 cannot move either across it.
  opt.prune_max_scale = 0.3f;
  // Effectively off, so this test has exactly one cause.
  opt.prune_opacity = 1e-6f;

  auto result = gsplat::train_gaussians(seed, views, opt);

  // The point of the test: the prune path executed, and took precisely the
  // Gaussians it was aimed at. `remap_parameters` — including the Adam moment
  // carry — runs only on this branch.
  INFO("seed " << seed.size() << " -> final " << result.final_count);
  REQUIRE(result.final_count < seed.size());
  REQUIRE(result.final_count == kBlob);
  REQUIRE_NOTHROW(result.gaussians.validate());

  // Every surviving Gaussian is below the threshold, and none is a leftover
  // row from the old tensors: a remap that dropped or misaligned a parameter
  // shows up here as a non-finite or oversized scale.
  for (const auto &s : result.gaussians.scales) {
    REQUIRE(std::isfinite(s[0]));
    REQUIRE(std::exp(std::max({s[0], s[1], s[2]})) < opt.prune_max_scale);
  }
  for (const auto &m : result.gaussians.means) {
    REQUIRE(std::isfinite(m[0]));
    REQUIRE(std::isfinite(m[1]));
    REQUIRE(std::isfinite(m[2]));
  }

  // Training continued past the prune on the remapped tensors rather than
  // stopping or diverging there.
  REQUIRE(result.history.back().iteration == opt.iterations);
  REQUIRE(std::isfinite(result.history.back().loss));
}

TEST_CASE("TrainGaussians_PruneThresholdAboveInitialOpacity_SkipsPruning",
          "[gsplat][gpu]") {
  if (!gsplat::has_cuda_device())
    SKIP("no CUDA device available");

  auto seed = gsplat::init_from_point_cloud(make_blob(6));
  auto views = make_ring(4);

  gsplat::TrainOptions opt;
  opt.iterations = 40;
  opt.log_interval = 20;
  opt.prune_enabled = true;
  opt.prune_start = 20;
  opt.prune_interval = 20;
  // A threshold above the initial alpha (0.1) would take everything; the
  // trainer must skip the pass rather than train on an empty model.
  opt.prune_opacity = 0.99f;

  auto result = gsplat::train_gaussians(seed, views, opt);

  REQUIRE(result.final_count == seed.size());
  REQUIRE(std::isfinite(result.history.back().loss));
}

TEST_CASE("TrainGaussians_HoldoutViews_ExcludedFromTrainingAndReported",
          "[gsplat][gpu]") {
  if (!gsplat::has_cuda_device())
    SKIP("no CUDA device available");

  auto seed = gsplat::init_from_point_cloud(make_blob(5));
  auto views = make_ring(8);

  gsplat::TrainOptions opt;
  opt.iterations = 60; // enough draws that all 6 training views get visited
  opt.log_interval = 10;
  opt.eval_interval = 10;
  opt.prune_enabled = false;
  opt.holdout_every = 4; // views 0 and 4 held out, 6 left to train on

  auto result = gsplat::train_gaussians(seed, views, opt);

  REQUIRE(!result.evals.empty());
  const auto &last = result.evals.back();
  REQUIRE(last.holdout_views == 2);
  REQUIRE(last.train_views == 6);
  REQUIRE(std::isfinite(last.holdout_psnr));
  REQUIRE(std::isfinite(last.holdout_ssim));
  REQUIRE(result.final_holdout_psnr == Approx(last.holdout_psnr));

  // --- the claim itself ----------------------------------------------------
  // "Held out" means no gradient ever flowed from these views, which is a
  // property of the per-iteration draw and not of the split. Assert it on the
  // views the loop actually touched: counts alone are satisfied by a loop that
  // draws from the full view list.
  REQUIRE(result.holdout_view_indices == std::vector<std::size_t>{0, 4});
  REQUIRE(result.trained_views == std::vector<std::size_t>{1, 2, 3, 5, 6, 7});
  for (const std::size_t h : result.holdout_view_indices)
    REQUIRE(std::find(result.trained_views.begin(), result.trained_views.end(),
                      h) == result.trained_views.end());
  // Together the two sets account for every view exactly once.
  REQUIRE(result.trained_views.size() + result.holdout_view_indices.size() ==
          views.size());

  // Every view in this synthetic scene shows the same blob against the same
  // flat target, so the two numbers should land close together. The assertion
  // that matters is that a held-out number exists at all and is a real
  // measurement, not that it beats or trails the training one.
  REQUIRE(last.holdout_psnr > 0.0);
}

TEST_CASE("TrainGaussians_HoldoutSplit_IsDeterministicAcrossRuns",
          "[gsplat][gpu]") {
  if (!gsplat::has_cuda_device())
    SKIP("no CUDA device available");

  auto seed = gsplat::init_from_point_cloud(make_blob(5));
  auto views = make_ring(8);

  gsplat::TrainOptions opt;
  opt.iterations = 20;
  opt.log_interval = 20;
  opt.eval_interval = 20;
  opt.prune_enabled = false;
  opt.holdout_every = 4;
  opt.seed = 11;

  auto a = gsplat::train_gaussians(seed, views, opt);
  auto b = gsplat::train_gaussians(seed, views, opt);

  REQUIRE(a.evals.size() == b.evals.size());
  REQUIRE(!a.evals.empty());
  // The split is a pure function of the view count, so it cannot drift even
  // though the trained values themselves are only reproducible to ~1e-3 (see
  // the reproducibility test above for why).
  REQUIRE(a.evals.back().holdout_views == b.evals.back().holdout_views);
  REQUIRE(a.evals.back().train_views == b.evals.back().train_views);
  REQUIRE(a.evals.back().holdout_psnr ==
          Approx(b.evals.back().holdout_psnr).epsilon(1e-3));
}

TEST_CASE("TrainGaussians_HoldoutEveryOne_FallsBackToTrainingOnAllViews",
          "[gsplat][gpu]") {
  if (!gsplat::has_cuda_device())
    SKIP("no CUDA device available");

  auto seed = gsplat::init_from_point_cloud(make_blob(4));
  auto views = make_ring(4);

  gsplat::TrainOptions opt;
  opt.iterations = 10;
  opt.log_interval = 10;
  opt.eval_interval = 10;
  opt.prune_enabled = false;
  opt.holdout_every = 1; // would hold out everything

  auto result = gsplat::train_gaussians(seed, views, opt);

  // Falls back to training on all views and reports no held-out metric,
  // rather than training on nothing.
  REQUIRE(!result.evals.empty());
  REQUIRE(result.evals.back().holdout_views == 0);
  REQUIRE(result.evals.back().train_views == 4);
  REQUIRE(result.final_holdout_psnr == Approx(0.0));
  // Nothing was withheld, so the fallback must train on the whole set rather
  // than on whatever the empty split left behind.
  REQUIRE(result.holdout_view_indices.empty());
  REQUIRE(result.trained_views == std::vector<std::size_t>{0, 1, 2, 3});
}

TEST_CASE("TrainGaussians_McmcEnabled_GrowsModelTowardCapBudget",
          "[gsplat][gpu]") {
  if (!gsplat::has_cuda_device())
    SKIP("no CUDA device available");

  auto seed = gsplat::init_from_point_cloud(make_blob(6));
  auto views = make_ring(4);

  gsplat::TrainOptions opt;
  opt.iterations = 60;
  opt.log_interval = 20;
  opt.eval_interval = 60;
  opt.holdout_every = 0;
  opt.mcmc.enabled = true;
  opt.mcmc.cap_factor = 1.5;
  opt.mcmc.refine_every = 10;
  opt.mcmc.refine_start = 10;

  auto result = gsplat::train_gaussians(seed, views, opt);

  const auto cap = static_cast<std::size_t>(seed.size() * 1.5);
  INFO("seed " << seed.size() << " -> final " << result.final_count << " (cap "
               << cap << "), added " << result.added);
  REQUIRE(result.added > 0);
  REQUIRE(result.final_count > seed.size());
  REQUIRE(result.final_count <= cap);

  // Growth must not corrupt the model: relocation writes Eq. 9's corrected
  // opacity and scale, and a sign or activation-space error there produces
  // NaNs within a few passes.
  REQUIRE_NOTHROW(result.gaussians.validate());
  for (const auto &m : result.gaussians.means) {
    REQUIRE(std::isfinite(m[0]));
    REQUIRE(std::isfinite(m[1]));
    REQUIRE(std::isfinite(m[2]));
  }
  for (const auto &s : result.gaussians.scales)
    REQUIRE(std::isfinite(s[0]));
  for (const float o : result.gaussians.opacities)
    REQUIRE(std::isfinite(o));
}

TEST_CASE("TrainGaussians_McmcAbsoluteCap_RespectsBudgetOverFactor",
          "[gsplat][gpu]") {
  if (!gsplat::has_cuda_device())
    SKIP("no CUDA device available");

  auto seed = gsplat::init_from_point_cloud(make_blob(5));
  auto views = make_ring(4);

  const auto cap = static_cast<std::int64_t>(seed.size()) + 10;

  gsplat::TrainOptions opt;
  opt.iterations = 80;
  opt.log_interval = 40;
  opt.eval_interval = 80;
  opt.holdout_every = 0;
  opt.mcmc.enabled = true;
  opt.mcmc.cap_absolute = cap; // takes precedence over cap_factor
  opt.mcmc.cap_factor = 100.0;
  opt.mcmc.refine_every = 10;
  opt.mcmc.refine_start = 10;

  auto result = gsplat::train_gaussians(seed, views, opt);

  REQUIRE(result.final_count <= static_cast<std::size_t>(cap));
}

TEST_CASE("TrainGaussians_McmcNoiseInjection_KeepsModelFinite",
          "[gsplat][gpu]") {
  if (!gsplat::has_cuda_device())
    SKIP("no CUDA device available");

  auto seed = gsplat::init_from_point_cloud(make_blob(5));
  auto views = make_ring(4);

  gsplat::TrainOptions opt;
  opt.iterations = 30;
  opt.log_interval = 30;
  opt.eval_interval = 30;
  opt.holdout_every = 0;
  opt.mcmc.enabled = true;
  // Isolate the Langevin term: no refine pass inside the run.
  opt.mcmc.refine_start = 1000;
  opt.mcmc.noise_lr = 5e5f;

  auto result = gsplat::train_gaussians(seed, views, opt);

  REQUIRE(result.final_count == seed.size());
  REQUIRE(std::isfinite(result.history.back().loss));
  for (const auto &m : result.gaussians.means) {
    REQUIRE(std::isfinite(m[0]));
    REQUIRE(std::isfinite(m[1]));
    REQUIRE(std::isfinite(m[2]));
  }
}

TEST_CASE("TrainGaussians_EmptyGaussiansOrNoViews_Throws", "[gsplat][gpu]") {
  if (!gsplat::has_cuda_device())
    SKIP("no CUDA device available");

  auto g = gsplat::init_from_point_cloud(make_blob(4));
  auto views = make_ring(2);

  REQUIRE_THROWS_WITH(gsplat::train_gaussians(gsplat::GaussianCloud{}, views),
                      Catch::Matchers::ContainsSubstring("zero Gaussians"));
  REQUIRE_THROWS_WITH(gsplat::train_gaussians(g, {}),
                      Catch::Matchers::ContainsSubstring("training views"));
}

// --- cancellation (#329) ----------------------------------------------------

TEST_CASE("TrainGaussians_CancelTokenSetBeforeCall_ReturnsUsableSeedModel",
          "[gsplat][gpu]") {
  if (!gsplat::has_cuda_device())
    SKIP("no CUDA device available");

  // Catches: a cancel token that is only consulted on some later schedule (or
  // not at all), and a cancelled run that returns an empty/garbage model
  // instead of the Gaussians it holds. Salvaging the model is the entire
  // reason a cooperative cancel exists rather than a SIGKILL.
  auto seed = gsplat::init_from_point_cloud(make_blob(5));
  auto views = make_ring(4);

  std::atomic_bool cancel{true}; // already requested before training starts

  gsplat::TrainOptions opt;
  opt.iterations = 500; // far more than a prompt cancel can get through
  opt.log_interval = 10;
  opt.prune_enabled = false;
  opt.cancel_token = &cancel;

  auto result = gsplat::train_gaussians(seed, views, opt);

  REQUIRE(result.cancelled);
  REQUIRE(result.iterations_run < opt.iterations / 10);

  // The model still comes back whole: same Gaussian count as the seed, valid,
  // finite, non-empty.
  REQUIRE_NOTHROW(result.gaussians.validate());
  REQUIRE_FALSE(result.gaussians.empty());
  REQUIRE(result.gaussians.size() == seed.size());
  REQUIRE(result.final_count == seed.size());
  for (const auto &m : result.gaussians.means) {
    REQUIRE(std::isfinite(m[0]));
    REQUIRE(std::isfinite(m[1]));
    REQUIRE(std::isfinite(m[2]));
  }
  for (const float o : result.gaussians.opacities)
    REQUIRE(std::isfinite(o));
}

TEST_CASE(
    "TrainGaussians_CancelTokenSetMidRun_StopsEarlyAndReturnsPartialModel",
    "[gsplat][gpu]") {
  if (!gsplat::has_cuda_device())
    SKIP("no CUDA device available");

  // Catches: a loop that ignores the token once it is past its first
  // iteration, and a cancelled run that reports the *requested* iteration
  // count or an empty history/eval list.
  //
  // The flip is driven by an observable side effect rather than by a sleep:
  // the watcher waits until the first checkpoint `.ply` has landed, which can
  // only happen after iteration `checkpoint_every` completed. That makes the
  // lower bound on `iterations_run` a fact rather than a race, and leaves the
  // remaining ~995 iterations of GPU work as the margin on the upper side.
  TempDir dir("reusex_gsplat_cancel");

  auto seed = gsplat::init_from_point_cloud(make_blob(5));
  auto views = make_ring(4);

  std::atomic_bool cancel{false};
  std::atomic_bool watcher_timed_out{false};

  gsplat::TrainOptions opt;
  opt.iterations = 1000; // never reached; the cancel lands around iteration 5
  opt.log_interval = 5;
  opt.eval_interval = 1000; // only the iteration-0 baseline fires on schedule
  opt.holdout_every = 0;
  opt.prune_enabled = false;
  opt.checkpoint_every = 5;
  opt.checkpoint_keep = 3;
  opt.checkpoint_dir = dir.path;
  opt.cancel_token = &cancel;

  std::thread watcher([&] {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(60);
    while (std::chrono::steady_clock::now() < deadline) {
      if (!files_with_extension(dir.path, ".ply").empty()) {
        cancel.store(true, std::memory_order_relaxed);
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    watcher_timed_out.store(true);
    cancel.store(true, std::memory_order_relaxed);
  });

  auto result = gsplat::train_gaussians(seed, views, opt);
  watcher.join();

  REQUIRE_FALSE(watcher_timed_out.load());
  REQUIRE(result.cancelled);
  INFO("stopped after " << result.iterations_run << " of " << opt.iterations);
  // Bounded on both sides: the flip cannot precede the checkpoint at iteration
  // 5, and the loop cannot run past what was asked for.
  REQUIRE(result.iterations_run >= opt.checkpoint_every);
  REQUIRE(result.iterations_run <= opt.iterations);

  // A cancelled run is still a complete run report, and every number in it
  // describes iterations that actually executed.
  REQUIRE_FALSE(result.history.empty());
  REQUIRE(result.history.back().iteration <= result.iterations_run);
  REQUIRE(std::isfinite(result.history.back().loss));
  REQUIRE_FALSE(result.evals.empty());
  REQUIRE(result.evals.back().iteration <= result.iterations_run);
  REQUIRE(result.seconds > 0.0);

  REQUIRE_NOTHROW(result.gaussians.validate());
  REQUIRE(result.gaussians.size() == seed.size());
  for (const auto &m : result.gaussians.means) {
    REQUIRE(std::isfinite(m[0]));
    REQUIRE(std::isfinite(m[1]));
    REQUIRE(std::isfinite(m[2]));
  }
}

TEST_CASE("TrainGaussians_CancelTokenNeverSet_RunsToCompletion",
          "[gsplat][gpu]") {
  if (!gsplat::has_cuda_device())
    SKIP("no CUDA device available");

  // The counterpart to the two above: catches a `cancelled` flag stuck on, a
  // token pointer that is misread as "cancel requested" merely by being
  // non-null, and an `iterations_run` that is never filled in.
  auto seed = gsplat::init_from_point_cloud(make_blob(4));
  auto views = make_ring(3);

  std::atomic_bool cancel{false};

  gsplat::TrainOptions opt;
  opt.iterations = 10;
  opt.log_interval = 5;
  opt.prune_enabled = false;
  opt.cancel_token = &cancel;

  auto result = gsplat::train_gaussians(seed, views, opt);

  REQUIRE_FALSE(result.cancelled);
  REQUIRE(result.iterations_run == opt.iterations);
  REQUIRE(result.history.back().iteration == opt.iterations);
}

// --- checkpointing (#329) ---------------------------------------------------

TEST_CASE("TrainGaussians_CheckpointEverySet_WritesLoadablePlysOnSchedule",
          "[gsplat][gpu]") {
  if (!gsplat::has_cuda_device())
    SKIP("no CUDA device available");

  // Catches: an off-by-one in the schedule, a `TrainResult::checkpoints` list
  // that disagrees with what is on disk, a duplicate checkpoint on the final
  // iteration, and a `.ply` written from stale or half-uploaded tensors.
  TempDir dir("reusex_gsplat_ckpt");

  auto seed = gsplat::init_from_point_cloud(make_blob(5));
  auto views = make_ring(4);

  gsplat::TrainOptions opt;
  opt.iterations = 20;
  opt.log_interval = 10;
  opt.eval_interval = 20;
  opt.holdout_every = 0;
  opt.prune_enabled = false;
  opt.checkpoint_every = 5;
  opt.checkpoint_keep = 0; // keep every checkpoint
  opt.checkpoint_dir = dir.path;

  auto result = gsplat::train_gaussians(seed, views, opt);

  REQUIRE(result.iterations_run == opt.iterations);

  // 5, 10, 15 — and deliberately NOT 20: the final model is what the caller
  // gets back, so a checkpoint there would be a byte-identical duplicate.
  const std::vector<std::filesystem::path> expected{
      dir.path / "checkpoint_iter000005.ply",
      dir.path / "checkpoint_iter000010.ply",
      dir.path / "checkpoint_iter000015.ply"};
  REQUIRE(result.checkpoints == expected);
  REQUIRE(files_with_extension(dir.path, ".ply") == expected);
  REQUIRE_FALSE(
      std::filesystem::exists(dir.path / "checkpoint_iter000020.ply"));

  // Every listed path is a file a caller can actually open, and it holds the
  // model rather than an empty shell.
  for (const auto &p : result.checkpoints) {
    REQUIRE(std::filesystem::exists(p));
    REQUIRE(std::filesystem::file_size(p) > 0);
  }
  const auto reloaded = gsplat::load_gaussian_ply(result.checkpoints.front());
  REQUIRE_NOTHROW(reloaded.validate());
  REQUIRE(reloaded.size() == seed.size());
  REQUIRE(reloaded.sh_degree == seed.sh_degree);
  for (const auto &m : reloaded.means) {
    REQUIRE(std::isfinite(m[0]));
    REQUIRE(std::isfinite(m[1]));
    REQUIRE(std::isfinite(m[2]));
  }
}

TEST_CASE("TrainGaussians_CheckpointKeepLimit_RetainsOnlyTheNewestN",
          "[gsplat][gpu]") {
  if (!gsplat::has_cuda_device())
    SKIP("no CUDA device available");

  // Catches: retention that deletes the newest instead of the oldest, that
  // trims the in-memory list without unlinking the file (or the reverse), and
  // a `.tmp` staging file left behind by the atomic write.
  TempDir dir("reusex_gsplat_keep");

  auto seed = gsplat::init_from_point_cloud(make_blob(4));
  auto views = make_ring(3);

  gsplat::TrainOptions opt;
  opt.iterations = 25;
  opt.log_interval = 25;
  opt.eval_interval = 25;
  opt.holdout_every = 0;
  opt.prune_enabled = false;
  opt.checkpoint_every = 5; // writes at 5, 10, 15, 20 (not 25)
  opt.checkpoint_keep = 2;
  opt.checkpoint_dir = dir.path;

  auto result = gsplat::train_gaussians(seed, views, opt);

  const std::vector<std::filesystem::path> survivors{
      dir.path / "checkpoint_iter000015.ply",
      dir.path / "checkpoint_iter000020.ply"};
  REQUIRE(result.checkpoints == survivors);
  REQUIRE(files_with_extension(dir.path, ".ply") == survivors);

  // The superseded ones are gone from disk, not merely dropped from the list.
  REQUIRE_FALSE(
      std::filesystem::exists(dir.path / "checkpoint_iter000005.ply"));
  REQUIRE_FALSE(
      std::filesystem::exists(dir.path / "checkpoint_iter000010.ply"));

  // The write goes `.tmp` -> rename; nothing may be left staged.
  REQUIRE(files_with_extension(dir.path, ".tmp").empty());

  REQUIRE(gsplat::load_gaussian_ply(survivors.back()).size() == seed.size());
}

TEST_CASE("TrainGaussians_CheckpointEveryZero_WritesNoCheckpointFiles",
          "[gsplat][gpu]") {
  if (!gsplat::has_cuda_device())
    SKIP("no CUDA device available");

  // Catches checkpointing that switches itself on whenever a directory is
  // present — a long run silently spraying `.ply` files nobody asked for.
  TempDir dir("reusex_gsplat_nockpt");

  auto seed = gsplat::init_from_point_cloud(make_blob(4));
  auto views = make_ring(3);

  gsplat::TrainOptions opt;
  opt.iterations = 12;
  opt.log_interval = 12;
  opt.eval_interval = 12;
  opt.holdout_every = 0;
  opt.prune_enabled = false;
  opt.checkpoint_dir = dir.path; // set, but the schedule is off by default
  REQUIRE(opt.checkpoint_every == 0);

  auto result = gsplat::train_gaussians(seed, views, opt);

  REQUIRE(result.iterations_run == opt.iterations);
  REQUIRE(result.checkpoints.empty());
  REQUIRE(files_with_extension(dir.path, ".ply").empty());
  REQUIRE(files_with_extension(dir.path, ".tmp").empty());
}

// --- non-fatal render failures / fatal bad destinations (#329) --------------

TEST_CASE("TrainGaussians_CheckpointRenderWriteFails_ContinuesAndOmitsPath",
          "[gsplat][gpu]") {
  if (!gsplat::has_cuda_device())
    SKIP("no CUDA device available");

  // Catches a regression to the old behaviour, where a single failed
  // cv::imwrite at iteration 500 of 30 000 threw away the whole run. The
  // render is a diagnostic; only the model is the product.
  TempDir dir("reusex_gsplat_render");

  const auto saved = std::filesystem::status(dir.path).permissions();
  std::filesystem::permissions(dir.path,
                               std::filesystem::perms::owner_read |
                                   std::filesystem::perms::owner_exec,
                               std::filesystem::perm_options::replace);
  PermissionsGuard restore(dir.path, saved);

  if (!directory_is_unwritable(dir.path))
    SKIP("cannot make a directory unwritable here (running as root?)");

  auto seed = gsplat::init_from_point_cloud(make_blob(4));
  auto views = make_ring(3);

  gsplat::TrainOptions opt;
  opt.iterations = 10;
  opt.log_interval = 10;
  opt.eval_interval = 10;
  opt.holdout_every = 0;
  opt.prune_enabled = false;
  opt.render_dir = dir.path;   // exists, so startup validation passes...
  opt.render_iterations = {5}; // ...but the write at iteration 5 cannot land
  opt.render_view_index = 0;

  gsplat::TrainResult result;
  REQUIRE_NOTHROW(result = gsplat::train_gaussians(seed, views, opt));

  // The run reached the end despite the failed write.
  REQUIRE(result.iterations_run == opt.iterations);
  REQUIRE_FALSE(result.cancelled);
  REQUIRE(result.history.back().iteration == opt.iterations);
  REQUIRE_NOTHROW(result.gaussians.validate());

  // `renders` is what is on disk, not what was attempted.
  REQUIRE(result.renders.empty());
  REQUIRE(files_with_extension(dir.path, ".png").empty());
}

TEST_CASE("TrainGaussians_UncreatableOutputDirectory_ThrowsBeforeTraining",
          "[gsplat][gpu]") {
  if (!gsplat::has_cuda_device())
    SKIP("no CUDA device available");

  // The one destination failure that must stay fatal: a directory the caller
  // named that can never exist. Catches a blanket downgrade of write failures
  // that would let an hours-long run proceed to a destination it can never
  // use. The parent here is a regular file, so create_directories cannot win.
  TempDir dir("reusex_gsplat_baddir");
  const auto blocker = dir.path / "not_a_directory";
  {
    std::ofstream f(blocker);
    f << "regular file";
  }
  REQUIRE(std::filesystem::is_regular_file(blocker));
  const auto impossible = blocker / "sub";

  auto seed = gsplat::init_from_point_cloud(make_blob(4));
  auto views = make_ring(3);

  gsplat::TrainOptions base;
  base.iterations = 5;
  base.log_interval = 5;
  base.eval_interval = 5;
  base.holdout_every = 0;
  base.prune_enabled = false;

  auto render = base;
  render.render_dir = impossible;
  render.render_iterations = {2};
  REQUIRE_THROWS_AS(gsplat::train_gaussians(seed, views, render),
                    std::runtime_error);

  auto checkpoint = base;
  checkpoint.checkpoint_every = 2;
  checkpoint.checkpoint_dir = impossible;
  REQUIRE_THROWS_AS(gsplat::train_gaussians(seed, views, checkpoint),
                    std::runtime_error);

  // Checkpointing on with nowhere to put the files is the same class of
  // mistake and must also refuse rather than train and discard.
  auto nowhere = base;
  nowhere.checkpoint_every = 2;
  nowhere.checkpoint_dir.clear();
  REQUIRE_THROWS_AS(gsplat::train_gaussians(seed, views, nowhere),
                    std::runtime_error);
}
