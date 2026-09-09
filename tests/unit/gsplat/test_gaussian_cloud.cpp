// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// CPU-only tests for the Gaussian seeding + .ply round-trip. Nothing here
// touches CUDA, so they run everywhere the heavy test binary builds.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <reusex/gsplat/GaussianCloud.hpp>

#include "../../support/temp_path.hpp"

#include <cmath>
#include <limits>
#include <vector>

using namespace reusex;
using Catch::Approx;

namespace {

/// A small axis-aligned grid with a known spacing, so the seeded scale can be
/// checked against an exact expectation rather than "looks plausible".
CloudPtr make_grid(int n_per_axis, float spacing) {
  CloudPtr cloud(new Cloud);
  for (int i = 0; i < n_per_axis; ++i)
    for (int j = 0; j < n_per_axis; ++j)
      for (int k = 0; k < n_per_axis; ++k) {
        PointT p;
        p.x = i * spacing;
        p.y = j * spacing;
        p.z = k * spacing;
        p.r = 255;
        p.g = 128;
        p.b = 0;
        cloud->push_back(p);
      }
  return cloud;
}

} // namespace

TEST_CASE("SH DC round-trips to RGB", "[gsplat]") {
  for (float c : {0.0f, 0.25f, 0.5f, 1.0f}) {
    const float f = gsplat::rgb_to_sh_dc(c);
    REQUIRE(gsplat::sh_dc_to_rgb(f) == Approx(c).margin(1e-6));
  }
  // Mid-grey is the fixed point of the transform: it carries no DC signal.
  REQUIRE(gsplat::rgb_to_sh_dc(0.5f) == Approx(0.0).margin(1e-9));
}

TEST_CASE("inverse_sigmoid inverts sigmoid and stays finite at the ends",
          "[gsplat]") {
  auto sigmoid = [](float x) { return 1.0f / (1.0f + std::exp(-x)); };
  REQUIRE(sigmoid(gsplat::inverse_sigmoid(0.1f)) == Approx(0.1f).margin(1e-5));
  REQUIRE(sigmoid(gsplat::inverse_sigmoid(0.9f)) == Approx(0.9f).margin(1e-5));
  // Clamped rather than infinite — an initial opacity of 0 or 1 must not
  // produce a non-finite parameter for Adam to step.
  REQUIRE(std::isfinite(gsplat::inverse_sigmoid(0.0f)));
  REQUIRE(std::isfinite(gsplat::inverse_sigmoid(1.0f)));
}

TEST_CASE("init_from_point_cloud seeds one Gaussian per point", "[gsplat]") {
  const float spacing = 0.05f;
  auto cloud = make_grid(5, spacing); // 125 points

  auto g = gsplat::init_from_point_cloud(cloud);

  REQUIRE(g.size() == cloud->size());
  REQUIRE(g.scales.size() == g.size());
  REQUIRE(g.quats.size() == g.size());
  REQUIRE(g.opacities.size() == g.size());
  REQUIRE(g.sh_dc.size() == g.size());
  REQUIRE_NOTHROW(g.validate());

  SECTION("means are the point positions") {
    REQUIRE(g.means.front()[0] == Approx(0.0f));
    REQUIRE(g.means.back()[0] == Approx(4 * spacing));
  }

  SECTION("colour becomes the DC coefficient, not raw RGB") {
    // The grid is (255,128,0); the DC term must be the SH encoding of that.
    REQUIRE(g.sh_dc.front()[0] == Approx(gsplat::rgb_to_sh_dc(1.0f)));
    REQUIRE(g.sh_dc.front()[2] == Approx(gsplat::rgb_to_sh_dc(0.0f)));
  }

  SECTION("rotation starts at identity") {
    REQUIRE(g.quats.front()[0] == Approx(1.0f));
    REQUIRE(g.quats.front()[1] == Approx(0.0f));
  }

  SECTION("scale tracks the neighbour spacing") {
    // Every scale is a log-scale within the configured clamp, and an interior
    // point of a regular grid sits exactly `spacing` from its 3 axis
    // neighbours.
    for (const auto &s : g.scales) {
      const float metres = std::exp(s[0]);
      REQUIRE(metres >= gsplat::GaussianInitOptions{}.min_scale);
      REQUIRE(metres <= gsplat::GaussianInitOptions{}.max_scale);
      REQUIRE(s[0] == Approx(s[1])); // isotropic at init
    }
    const float centre_scale = std::exp(g.scales[62][0]); // interior point
    REQUIRE(centre_scale == Approx(spacing).margin(1e-4));
  }

  SECTION("opacity is the logit of the requested alpha") {
    const float expected =
        gsplat::inverse_sigmoid(gsplat::GaussianInitOptions{}.initial_opacity);
    REQUIRE(g.opacities.front() == Approx(expected));
  }
}

TEST_CASE("init_from_point_cloud honours max_points with a uniform stride",
          "[gsplat]") {
  auto cloud = make_grid(10, 0.05f); // 1000 points
  gsplat::GaussianInitOptions opt;
  opt.max_points = 100;

  auto g = gsplat::init_from_point_cloud(cloud, opt);

  // Stride is ceil(1000/100) = 10, so exactly 100 seeds survive.
  REQUIRE(g.size() == 100);
  REQUIRE(g.means.front()[0] == Approx((*cloud)[0].x));
  REQUIRE(g.means[1][0] == Approx((*cloud)[10].x));
}

TEST_CASE("init_from_point_cloud is deterministic", "[gsplat]") {
  auto cloud = make_grid(6, 0.03f);
  gsplat::GaussianInitOptions opt;
  opt.max_points = 50;

  auto a = gsplat::init_from_point_cloud(cloud, opt);
  auto b = gsplat::init_from_point_cloud(cloud, opt);

  REQUIRE(a.size() == b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    REQUIRE(a.means[i][0] == b.means[i][0]);
    REQUIRE(a.scales[i][0] == b.scales[i][0]);
    REQUIRE(a.opacities[i] == b.opacities[i]);
  }
}

TEST_CASE("init_from_point_cloud rejects unusable input", "[gsplat]") {
  REQUIRE_THROWS_WITH(gsplat::init_from_point_cloud(CloudPtr{}),
                      Catch::Matchers::ContainsSubstring("null"));

  CloudPtr empty(new Cloud);
  REQUIRE_THROWS_WITH(gsplat::init_from_point_cloud(empty),
                      Catch::Matchers::ContainsSubstring("empty"));

  auto cloud = make_grid(3, 0.1f);
  gsplat::GaussianInitOptions bad;
  bad.sh_degree = 9;
  REQUIRE_THROWS_WITH(gsplat::init_from_point_cloud(cloud, bad),
                      Catch::Matchers::ContainsSubstring("sh_degree"));
}

TEST_CASE("duplicate points are clamped, not turned into zero-size Gaussians",
          "[gsplat]") {
  CloudPtr cloud(new Cloud);
  for (int i = 0; i < 10; ++i) {
    PointT p;
    p.x = p.y = p.z = 0.0f;
    p.r = p.g = p.b = 255;
    cloud->push_back(p);
  }

  auto g = gsplat::init_from_point_cloud(cloud);

  REQUIRE(g.size() == 10);
  for (const auto &s : g.scales) {
    REQUIRE(std::isfinite(s[0]));
    REQUIRE(std::exp(s[0]) ==
            Approx(gsplat::GaussianInitOptions{}.min_scale).margin(1e-9));
  }
}

TEST_CASE("gaussian .ply survives a write/read round-trip", "[gsplat]") {
  auto cloud = make_grid(4, 0.07f);
  auto g = gsplat::init_from_point_cloud(cloud);

  reusex::test_support::TempPath tmp("test_gaussian_cloud", ".ply");
  gsplat::save_gaussian_ply(g, tmp.path);
  REQUIRE(std::filesystem::exists(tmp.path));

  auto back = gsplat::load_gaussian_ply(tmp.path);

  REQUIRE(back.size() == g.size());
  REQUIRE(back.sh_degree == g.sh_degree);
  for (std::size_t i = 0; i < g.size(); ++i) {
    REQUIRE(back.means[i][0] == Approx(g.means[i][0]));
    REQUIRE(back.means[i][2] == Approx(g.means[i][2]));
    REQUIRE(back.scales[i][1] == Approx(g.scales[i][1]));
    REQUIRE(back.quats[i][0] == Approx(g.quats[i][0]));
    REQUIRE(back.opacities[i] == Approx(g.opacities[i]));
    REQUIRE(back.sh_dc[i][1] == Approx(g.sh_dc[i][1]));
  }
}

TEST_CASE("validate() names the array that disagrees", "[gsplat]") {
  gsplat::GaussianCloud g;
  g.means.resize(3);
  g.scales.resize(3);
  g.quats.resize(3);
  g.opacities.resize(2); // deliberately short
  g.sh_dc.resize(3);
  REQUIRE_THROWS_WITH(g.validate(),
                      Catch::Matchers::ContainsSubstring("opacities"));
}

TEST_CASE("validate() rejects ragged sh_rest rows", "[gsplat]") {
  // A short row is the dangerous case: save_gaussian_ply sizes the header from
  // row 0 and then writes every row at that stride, so one wrong width shifts
  // the byte offset of every vertex after it. The file stays syntactically
  // valid, which is exactly why validate() has to catch it.
  auto make = [](int degree, std::size_t n) {
    gsplat::GaussianCloud g;
    g.sh_degree = degree;
    g.means.resize(n);
    g.scales.resize(n);
    g.quats.resize(n);
    g.opacities.resize(n);
    g.sh_dc.resize(n);
    const auto bands = static_cast<std::size_t>((degree + 1) * (degree + 1));
    g.sh_rest.assign(n, std::vector<float>((bands - 1) * 3, 0.0f));
    return g;
  };

  SECTION("a consistent degree-2 cloud passes") {
    auto g = make(2, 4);
    REQUIRE(g.sh_rest.front().size() == 24); // (9 - 1) * 3
    REQUIRE_NOTHROW(g.validate());
  }

  SECTION("one short row is caught and located") {
    auto g = make(2, 4);
    g.sh_rest[2].pop_back();
    REQUIRE_THROWS_WITH(g.validate(),
                        Catch::Matchers::ContainsSubstring("row 2"));
  }

  SECTION("a width that contradicts sh_degree is caught") {
    auto g = make(2, 4);
    g.sh_degree = 1; // rows are still 24 wide, degree 1 wants 9
    REQUIRE_THROWS_WITH(g.validate(),
                        Catch::Matchers::ContainsSubstring("sh_rest"));
  }
}

TEST_CASE("init_from_point_cloud rejects non-finite coordinates", "[gsplat]") {
  // PCL's FLANN tree does not report NaN as an error — it returns some
  // neighbour with a NaN squared distance, which the seeding code would
  // otherwise misread as a duplicate point and clamp, carrying the NaN into
  // the mean and from there into the first loss.
  auto cloud = make_grid(4, 0.05f);
  (*cloud)[7].y = std::numeric_limits<float>::quiet_NaN();

  REQUIRE_THROWS_WITH(gsplat::init_from_point_cloud(cloud),
                      Catch::Matchers::ContainsSubstring("non-finite"));

  auto inf_cloud = make_grid(4, 0.05f);
  (*inf_cloud)[3].z = std::numeric_limits<float>::infinity();
  REQUIRE_THROWS_WITH(gsplat::init_from_point_cloud(inf_cloud),
                      Catch::Matchers::ContainsSubstring("non-finite"));

  // A NaN the stride never selects is not the model's problem: seeding must
  // not fail on a point it does not use.
  auto strided = make_grid(4, 0.05f);
  (*strided)[1].x = std::numeric_limits<float>::quiet_NaN();
  gsplat::GaussianInitOptions opt;
  opt.max_points = 8; // stride 8, so indices 0, 8, 16, ... are picked
  REQUIRE_NOTHROW(gsplat::init_from_point_cloud(strided, opt));
}
