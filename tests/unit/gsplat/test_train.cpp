// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// GPU tests for the training loop. Every TEST_CASE here is tagged [gpu]:
// gsplat's rasterizer is CUDA-only (there is no CPU kernel), so on a machine
// without a device these must be excluded with `ctest -E` / `--exclude-tags`
// rather than silently passing.
//
// The scene is synthetic and deliberately tiny — a handful of Gaussians seen
// from a few cameras — so the whole file runs in seconds and asserts on
// *behaviour* (does the loss fall, does the geometry survive a round trip)
// rather than on reconstruction quality, which needs a real capture.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <reusex/gsplat/GaussianCloud.hpp>
#include <reusex/gsplat/TrainingViews.hpp>
#include <reusex/gsplat/train.hpp>

#include <opencv2/core.hpp>

#include <fmt/format.h>

#include <cmath>
#include <cstdint>

using namespace reusex;
using Catch::Approx;

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

std::vector<gsplat::TrainingView> make_ring(int n) {
  std::vector<gsplat::TrainingView> views;
  for (int i = 0; i < n; ++i)
    views.push_back(make_view(i, 2.0 * M_PI * i / n));
  return views;
}

} // namespace

TEST_CASE("the trainer reports itself available in this build", "[gsplat]") {
  // Compiled at all means the CUDA backend was found at configure time; this
  // is the capability probe the CLI reports on.
  REQUIRE(gsplat::is_available());
}

TEST_CASE("rendering the seed produces a non-empty image", "[gsplat][gpu]") {
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

TEST_CASE("one training run reduces the loss", "[gsplat][gpu]") {
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

TEST_CASE("training reproduces to float tolerance for a fixed seed",
          "[gsplat][gpu]") {
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

TEST_CASE("trained Gaussians survive the tensor round trip", "[gsplat][gpu]") {
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

TEST_CASE("pruning drops collapsed Gaussians without emptying the model",
          "[gsplat][gpu]") {
  auto seed = gsplat::init_from_point_cloud(make_blob(6));
  auto views = make_ring(4);

  gsplat::TrainOptions opt;
  opt.iterations = 40;
  opt.log_interval = 20;
  opt.prune_enabled = true;
  opt.prune_start = 20;
  opt.prune_interval = 20;
  // A threshold above the initial alpha (0.1) would take everything; the
  // trainer must refuse rather than train on an empty model.
  opt.prune_opacity = 0.99f;

  auto result = gsplat::train_gaussians(seed, views, opt);

  REQUIRE(result.final_count > 0);
}

TEST_CASE("held-out views are excluded from training and reported",
          "[gsplat][gpu]") {
  auto seed = gsplat::init_from_point_cloud(make_blob(5));
  auto views = make_ring(8);

  gsplat::TrainOptions opt;
  opt.iterations = 20;
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

  // Every view in this synthetic scene shows the same blob against the same
  // flat target, so the two numbers should land close together. The assertion
  // that matters is that a held-out number exists at all and is a real
  // measurement, not that it beats or trails the training one.
  REQUIRE(last.holdout_psnr > 0.0);
}

TEST_CASE("the held-out split is identical across runs", "[gsplat][gpu]") {
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

TEST_CASE("a split that would leave no training views is refused",
          "[gsplat][gpu]") {
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
}

TEST_CASE("MCMC grows the model toward its budget", "[gsplat][gpu]") {
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

TEST_CASE("MCMC respects an absolute budget", "[gsplat][gpu]") {
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

TEST_CASE("MCMC noise injection leaves the model finite", "[gsplat][gpu]") {
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

TEST_CASE("the trainer rejects unusable input", "[gsplat][gpu]") {
  auto g = gsplat::init_from_point_cloud(make_blob(4));
  auto views = make_ring(2);

  REQUIRE_THROWS_WITH(gsplat::train_gaussians(gsplat::GaussianCloud{}, views),
                      Catch::Matchers::ContainsSubstring("zero Gaussians"));
  REQUIRE_THROWS_WITH(gsplat::train_gaussians(g, {}),
                      Catch::Matchers::ContainsSubstring("training views"));
}
