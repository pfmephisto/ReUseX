// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// geometry::apply_optimize_parameters: the one reader of the optimize stage's
// JSON parameters, shared by `rux optimize`, the Qt client and ruxd's web GUI.

#include <catch2/catch_test_macros.hpp>

#include <reusex/slam/optimize_parameters.hpp>

#include <stdexcept>

using reusex::geometry::apply_optimize_parameters;
using reusex::geometry::PlaneGraphOptions;

TEST_CASE("OptimizeParameters_Empty_KeepsEverything",
          "[slam][optimize_parameters]") {
  PlaneGraphOptions options;
  options.min_landmark_observations = 9;
  bool dry_run = true;
  apply_optimize_parameters(options, dry_run, "");
  CHECK(options.min_landmark_observations == 9);
  CHECK(options.assoc_rounds == PlaneGraphOptions{}.assoc_rounds);
  CHECK(options.use_gnc);
  CHECK(dry_run);

  apply_optimize_parameters(options, dry_run, "{}");
  CHECK(options.min_landmark_observations == 9);
  CHECK(dry_run);
}

TEST_CASE("OptimizeParameters_Keys_Applied", "[slam][optimize_parameters]") {
  PlaneGraphOptions options;
  bool dry_run = false;
  apply_optimize_parameters(
      options, dry_run,
      R"({"min_observations":7,"assoc_rounds":3,"no_gnc":true,"dry_run":true,
          "unknown_key":"ignored"})");
  CHECK(options.min_landmark_observations == 7);
  CHECK(options.assoc_rounds == 3);
  CHECK_FALSE(options.use_gnc);
  CHECK(dry_run);
}

TEST_CASE("OptimizeParameters_NullAndFalse_LeaveTheBase",
          "[slam][optimize_parameters]") {
  PlaneGraphOptions options;
  options.use_gnc = false; // e.g. a CLI --no-gnc already applied
  options.assoc_rounds = 5;
  bool dry_run = true;
  apply_optimize_parameters(options, dry_run,
                            R"({"assoc_rounds":null,"no_gnc":false})");
  CHECK(options.assoc_rounds == 5);
  CHECK_FALSE(options.use_gnc); // "no_gnc": false never turns GNC back on
  CHECK(dry_run);
}

TEST_CASE("OptimizeParameters_BadInput_InvalidArgument",
          "[slam][optimize_parameters]") {
  PlaneGraphOptions options;
  bool dry_run = false;
  CHECK_THROWS_AS(apply_optimize_parameters(options, dry_run, "[1,2]"),
                  std::invalid_argument);
  CHECK_THROWS_AS(apply_optimize_parameters(options, dry_run, "{not json"),
                  std::invalid_argument);
  CHECK_THROWS_AS(apply_optimize_parameters(options, dry_run,
                                            R"({"min_observations":"7"})"),
                  std::invalid_argument);
}
