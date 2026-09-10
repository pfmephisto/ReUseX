// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Unit tests for the `--filter` pre-flight checks shared by `rux create mesh`,
// `rux export ply/e57` and `rux view` (#249).
//
// `rux::filters` is the app-layer diagnostic wrapper around the library's
// filter evaluator: it is what turns a typo into "Label cloud 'plnes' not
// found. Did you mean 'planes'?" instead of a bare parse failure. That
// messaging is the whole value of the layer, so the tests assert on the hints
// and not just on the boolean.
//
// Linked from `rux_core_lib`, so these run in the light test binary.

#include <catch2/catch_test_macros.hpp>

#include <filter_utils.hpp>

#include "../../support/temp_path.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/types/point_types.hpp>

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

using reusex::test_support::TempPath;

namespace {

/// Store a label cloud of `size` points under `name`, labels 0..n-1 modulo 4.
void seed_label_cloud(reusex::ProjectDB &db, std::string_view name,
                      std::size_t size) {
  reusex::CloudL cloud;
  cloud.resize(size);
  for (std::size_t i = 0; i < size; ++i) {
    cloud[i].label = static_cast<std::uint32_t>(i % 4);
  }
  db.save_point_cloud(name, cloud);
}

} // namespace

// ===========================================================================
// validate_expression_syntax -- no database required
// ===========================================================================

TEST_CASE("ValidateExpressionSyntax_WellFormedExpression_ReturnsValid",
          "[rux][filters]") {
  const auto result = rux::filters::validate_expression_syntax("planes in [1]");

  CHECK(result.valid);
  CHECK(static_cast<bool>(result)); // operator bool mirrors .valid
  CHECK(result.error_message.empty());
  CHECK(result.resolution_hint.empty());
}

TEST_CASE("ValidateExpressionSyntax_EmptyExpression_ReturnsInvalid",
          "[rux][filters]") {
  const auto result = rux::filters::validate_expression_syntax("");

  CHECK_FALSE(result.valid);
  CHECK(result.error_message == "Empty filter expression");
  CHECK_FALSE(result.resolution_hint.empty());
}

TEST_CASE("ValidateExpressionSyntax_MismatchedParentheses_ReturnsInvalid",
          "[rux][filters]") {
  const auto missing_close =
      rux::filters::validate_expression_syntax("(planes in [1]");
  CHECK_FALSE(missing_close.valid);
  CHECK(missing_close.error_message == "Mismatched parentheses");

  const auto missing_open =
      rux::filters::validate_expression_syntax("planes in [1])");
  CHECK_FALSE(missing_open.valid);
  CHECK(missing_open.error_message == "Mismatched parentheses");

  // Balanced, just nested.
  CHECK(rux::filters::validate_expression_syntax("((planes in [1]))").valid);
}

TEST_CASE("ValidateExpressionSyntax_MismatchedBrackets_ReturnsInvalid",
          "[rux][filters]") {
  const auto result =
      rux::filters::validate_expression_syntax("planes in [1, 2");

  CHECK_FALSE(result.valid);
  CHECK(result.error_message == "Mismatched brackets");
  CHECK(result.resolution_hint.find("'['") != std::string::npos);
}

TEST_CASE("ValidateExpressionSyntax_InOperatorWithoutBrackets_ReturnsInvalid",
          "[rux][filters]") {
  const auto result = rux::filters::validate_expression_syntax("planes in 1");

  CHECK_FALSE(result.valid);
  CHECK(result.error_message.find("'in' operator") != std::string::npos);
  CHECK(result.resolution_hint.find("cloud in [value1") != std::string::npos);
}

TEST_CASE("ValidateExpressionSyntax_ComparisonOperators_ReturnsValid",
          "[rux][filters]") {
  // No 'in', so the bracket rule does not apply.
  for (const auto *expr : {"planes == 3", "rooms != 0", "planes >= 2",
                           "planes < 10", "rooms > 1 and planes <= 5"}) {
    INFO("expression: " << expr);
    CHECK(rux::filters::validate_expression_syntax(expr).valid);
  }
}

// ===========================================================================
// validate_clouds_exist -- against a real ProjectDB
// ===========================================================================

TEST_CASE("ValidateCloudsExist_AllReferencedCloudsPresent_ReturnsValid",
          "[rux][filters]") {
  TempPath project("test_filter_utils");
  reusex::ProjectDB db(project.path);
  seed_label_cloud(db, "planes", 16);
  seed_label_cloud(db, "rooms", 16);

  const auto result =
      rux::filters::validate_clouds_exist("planes in [1] and rooms in [2]", db);

  CHECK(result.valid);
  CHECK(result.error_message.empty());
}

TEST_CASE("ValidateCloudsExist_MissingCloud_ReturnsInvalidNamingTheCloud",
          "[rux][filters]") {
  TempPath project("test_filter_utils");
  reusex::ProjectDB db(project.path);
  seed_label_cloud(db, "planes", 16);

  const auto result = rux::filters::validate_clouds_exist("rooms in [2]", db);

  CHECK_FALSE(result.valid);
  CHECK(result.error_message.find("rooms") != std::string::npos);
  // The hint lists what is actually available (STANDARDS §5: fail with the
  // numbers/names, not just "invalid").
  CHECK(result.resolution_hint.find("planes") != std::string::npos);
}

TEST_CASE("ValidateCloudsExist_TypoWithSubstringOverlap_SuggestsNearestCloud",
          "[rux][filters]") {
  TempPath project("test_filter_utils");
  reusex::ProjectDB db(project.path);
  seed_label_cloud(db, "planes", 16);

  // "plane" is a substring of "planes", which is what the fuzzy match keys on.
  const auto result = rux::filters::validate_clouds_exist("plane == 1", db);

  CHECK_FALSE(result.valid);
  CHECK(result.resolution_hint.find("Did you mean 'planes'?") !=
        std::string::npos);
}

TEST_CASE("ValidateCloudsExist_ExpressionNamesNoCloud_ReturnsValid",
          "[rux][filters]") {
  TempPath project("test_filter_utils");
  reusex::ProjectDB db(project.path);

  // No identifier is followed by an operator, so there is nothing to resolve
  // and nothing to complain about at this stage.
  CHECK(rux::filters::validate_clouds_exist("", db).valid);
  CHECK(rux::filters::validate_clouds_exist("planes", db).valid);
}

// ===========================================================================
// validate_cloud_sizes
// ===========================================================================

TEST_CASE("ValidateCloudSizes_SingleCloudReferenced_ReturnsValid",
          "[rux][filters]") {
  TempPath project("test_filter_utils");
  reusex::ProjectDB db(project.path);
  seed_label_cloud(db, "planes", 16);

  // One cloud cannot disagree with itself, so the check short-circuits without
  // ever touching the database.
  CHECK(rux::filters::validate_cloud_sizes("planes in [1]", db).valid);
}

TEST_CASE("ValidateCloudSizes_MatchingSizes_ReturnsValid", "[rux][filters]") {
  TempPath project("test_filter_utils");
  reusex::ProjectDB db(project.path);
  seed_label_cloud(db, "planes", 32);
  seed_label_cloud(db, "rooms", 32);

  CHECK(rux::filters::validate_cloud_sizes("planes in [1] and rooms in [2]", db)
            .valid);
}

TEST_CASE("ValidateCloudSizes_DivergentSizes_ReturnsInvalidWithBothCounts",
          "[rux][filters]") {
  TempPath project("test_filter_utils");
  reusex::ProjectDB db(project.path);
  seed_label_cloud(db, "planes", 32);
  seed_label_cloud(db, "rooms", 8);

  const auto result =
      rux::filters::validate_cloud_sizes("planes in [1] and rooms in [2]", db);

  CHECK_FALSE(result.valid);
  // Index-aligned clouds are a hard contract (STANDARDS §3.2); the message has
  // to report both counts so the user can tell which stage is stale.
  CHECK(result.error_message.find("32") != std::string::npos);
  CHECK(result.error_message.find("8") != std::string::npos);
  CHECK(result.resolution_hint.find("re-run segmentation") !=
        std::string::npos);
}
