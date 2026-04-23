// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>
#include <core/ProjectDB.hpp>
#include <core/filter_expression.hpp>

TEST_CASE("Filter expression parsing and evaluation", "[filter][parser]") {
  // Create in-memory database
  reusex::ProjectDB db(":memory:");

  // Create test label cloud
  auto planes = std::make_shared<reusex::CloudL>();
  planes->resize(100);
  planes->width = 100;
  planes->height = 1;

  // Assign labels: 10 points per label (1-10)
  for (size_t i = 0; i < 100; ++i) {
    planes->points[i].label = static_cast<int32_t>((i / 10) + 1);
  }
  db.save_point_cloud("planes", *planes);

  SECTION("Parse simple equality") {
    auto expr = reusex::core::parse_filter_expression("planes == 5", db);
    REQUIRE(expr != nullptr);
    REQUIRE(expr->clouds.size() == 1);
    REQUIRE(expr->clouds[0].name() == "planes");

    // Evaluate filter
    auto indices = reusex::core::evaluate_filter(*expr, planes->size());
    REQUIRE(indices->size() == 10); // Points 40-49 have label 5
    REQUIRE((*indices)[0] == 40);
    REQUIRE((*indices)[9] == 49);
  }

  SECTION("Parse in-set expression") {
    auto expr = reusex::core::parse_filter_expression("planes in [1, 2, 5]", db);
    REQUIRE(expr != nullptr);

    auto indices = reusex::core::evaluate_filter(*expr, planes->size());
    REQUIRE(indices->size() == 30); // 10 points each for labels 1, 2, 5

    // Check that indices are correct
    std::vector<int> expected_labels = {1, 2, 5};
    for (int idx : *indices) {
      int label = planes->points[idx].label;
      REQUIRE(std::find(expected_labels.begin(), expected_labels.end(),
                        label) != expected_labels.end());
    }
  }

  SECTION("Parse not-equal expression") {
    auto expr = reusex::core::parse_filter_expression("planes != 1", db);
    REQUIRE(expr != nullptr);

    auto indices = reusex::core::evaluate_filter(*expr, planes->size());
    REQUIRE(indices->size() == 90); // All except label 1
  }

  SECTION("Parse comparison expressions") {
    auto expr_gt = reusex::core::parse_filter_expression("planes > 5", db);
    auto indices_gt = reusex::core::evaluate_filter(*expr_gt, planes->size());
    REQUIRE(indices_gt->size() == 50); // Labels 6-10

    auto expr_ge = reusex::core::parse_filter_expression("planes >= 5", db);
    auto indices_ge = reusex::core::evaluate_filter(*expr_ge, planes->size());
    REQUIRE(indices_ge->size() == 60); // Labels 5-10

    auto expr_lt = reusex::core::parse_filter_expression("planes < 5", db);
    auto indices_lt = reusex::core::evaluate_filter(*expr_lt, planes->size());
    REQUIRE(indices_lt->size() == 40); // Labels 1-4

    auto expr_le = reusex::core::parse_filter_expression("planes <= 5", db);
    auto indices_le = reusex::core::evaluate_filter(*expr_le, planes->size());
    REQUIRE(indices_le->size() == 50); // Labels 1-5
  }

  SECTION("Parse AND expression") {
    auto expr =
        reusex::core::parse_filter_expression("planes >= 3 && planes <= 7", db);
    auto indices = reusex::core::evaluate_filter(*expr, planes->size());
    REQUIRE(indices->size() == 50); // Labels 3-7 (5 labels * 10 points each)
  }

  SECTION("Parse OR expression") {
    auto expr =
        reusex::core::parse_filter_expression("planes == 1 || planes == 10", db);
    auto indices = reusex::core::evaluate_filter(*expr, planes->size());
    REQUIRE(indices->size() == 20); // Labels 1 and 10
  }

  SECTION("Parse parenthesized expression") {
    auto expr = reusex::core::parse_filter_expression(
        "(planes == 1 || planes == 2) && planes != 3", db);
    auto indices = reusex::core::evaluate_filter(*expr, planes->size());
    REQUIRE(indices->size() == 20); // Labels 1 and 2 (3 is excluded)
  }

  SECTION("Invalid syntax throws") {
    REQUIRE_THROWS(reusex::core::parse_filter_expression("planes in 1, 2]", db));
    REQUIRE_THROWS(reusex::core::parse_filter_expression("invalid ==", db));
    REQUIRE_THROWS(reusex::core::parse_filter_expression("", db));
  }

  SECTION("Unknown cloud throws") {
    REQUIRE_THROWS_AS(reusex::core::parse_filter_expression("unknown == 5", db),
                      std::runtime_error);
  }
}

TEST_CASE("Multi-cloud filter expressions", "[filter][multi-cloud]") {
  reusex::ProjectDB db(":memory:");

  // Create two label clouds with same size
  auto planes = std::make_shared<reusex::CloudL>();
  planes->resize(100);
  planes->width = 100;
  planes->height = 1;
  for (size_t i = 0; i < 100; ++i) {
    planes->points[i].label = (i < 50) ? 1 : 2;
  }
  db.save_point_cloud("planes", *planes);

  auto rooms = std::make_shared<reusex::CloudL>();
  rooms->resize(100);
  rooms->width = 100;
  rooms->height = 1;
  for (size_t i = 0; i < 100; ++i) {
    rooms->points[i].label = (i < 50) ? 10 : 11;
  }
  db.save_point_cloud("rooms", *rooms);

  SECTION("Multi-cloud expression with OR") {
    // Note: Current implementation has simplified multi-cloud support
    // It evaluates using the first cloud's labels
    // Full multi-cloud support would require AST nodes to track which cloud
    // This test documents current behavior
    auto expr =
        reusex::core::parse_filter_expression("planes == 1 || rooms == 10", db);
    REQUIRE(expr->clouds.size() == 2);
  }

  SECTION("Cloud size mismatch throws") {
    auto small_cloud = std::make_shared<reusex::CloudL>();
    small_cloud->resize(50);
    small_cloud->width = 50;
    small_cloud->height = 1;
    db.save_point_cloud("small", *small_cloud);

    REQUIRE_THROWS_AS(
        reusex::core::parse_filter_expression("planes == 1 || small == 5", db),
        std::runtime_error);
  }
}

TEST_CASE("Filter expression edge cases", "[filter][edge-cases]") {
  reusex::ProjectDB db(":memory:");

  // Create label cloud with negative and zero labels
  auto labels = std::make_shared<reusex::CloudL>();
  labels->resize(100);
  labels->width = 100;
  labels->height = 1;
  for (size_t i = 0; i < 100; ++i) {
    labels->points[i].label =
        static_cast<int32_t>(i) - 50; // Labels from -50 to 49
  }
  db.save_point_cloud("test", *labels);

  SECTION("Filter with negative values") {
    auto expr = reusex::core::parse_filter_expression("test < 0", db);
    auto indices = reusex::core::evaluate_filter(*expr, labels->size());
    REQUIRE(indices->size() == 50); // Labels -50 to -1
  }

  SECTION("Filter with zero") {
    auto expr = reusex::core::parse_filter_expression("test == 0", db);
    auto indices = reusex::core::evaluate_filter(*expr, labels->size());
    REQUIRE(indices->size() == 1); // Only index 50 has label 0
    REQUIRE((*indices)[0] == 50);
  }

  SECTION("Empty result set") {
    auto expr = reusex::core::parse_filter_expression("test == 999", db);
    auto indices = reusex::core::evaluate_filter(*expr, labels->size());
    REQUIRE(indices->empty());
  }

  SECTION("In-set with single value") {
    auto expr = reusex::core::parse_filter_expression("test in [0]", db);
    auto indices = reusex::core::evaluate_filter(*expr, labels->size());
    REQUIRE(indices->size() == 1);
  }

  SECTION("In-set with empty list") {
    auto expr = reusex::core::parse_filter_expression("test in []", db);
    auto indices = reusex::core::evaluate_filter(*expr, labels->size());
    REQUIRE(indices->empty());
  }
}
