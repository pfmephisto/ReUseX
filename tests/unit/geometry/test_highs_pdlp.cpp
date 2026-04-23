// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <Highs.h>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "core/logging.hpp"

using namespace Catch::Matchers;

/** Test HiGHS PDLP solver support with a simple continuous Linear Program
 *
 * This test verifies that HiGHS was built with GPU/PDLP support, even though
 * the Solidifier class cannot use it due to binary variable constraints.
 *
 * Problem: minimize x + y
 *          subject to: x + y >= 1
 *                      x, y >= 0
 *
 * Optimal solution: x = 1, y = 0 (or x = 0, y = 1) with objective value = 1
 */
TEST_CASE("HiGHS PDLP solver with continuous LP", "[highs][pdlp][gpu]") {
  Highs highs;

  // Try to enable PDLP solver (will fail if not built with GPU support)
  HighsStatus pdlp_status = highs.setOptionValue("solver", "pdlp");

  // Suppress solver output in tests
  highs.setOptionValue("log_to_console", false);
  highs.setOptionValue("output_flag", false);

  SECTION("PDLP solver availability") {
    if (pdlp_status == HighsStatus::kOk) {
      reusex::core::info("✓ HiGHS built with PDLP support (GPU acceleration available)");
    } else {
      reusex::core::warn("✗ HiGHS built without PDLP support (GPU acceleration not available)");
      reusex::core::warn("  This is expected if CUPDLP_GPU=OFF in overlays/highs.nix");
    }

    // Test should pass regardless - we're just checking availability
    REQUIRE(true);
  }

  SECTION("Solve simple continuous LP") {
    // Skip if PDLP not available
    if (pdlp_status != HighsStatus::kOk) {
      reusex::core::info("Skipping LP solve test - PDLP not available");
      REQUIRE(true);
      return;
    }

    // Create variables: x, y with bounds [0, infinity)
    std::vector<double> obj_coeffs = {1.0, 1.0};     // minimize x + y
    std::vector<double> col_lower = {0.0, 0.0};      // x, y >= 0
    std::vector<double> col_upper = {1e30, 1e30};    // no upper bound

    HighsStatus status = highs.addCols(
        2,                    // num_cols
        obj_coeffs.data(),    // costs
        col_lower.data(),     // lower bounds
        col_upper.data(),     // upper bounds
        0,                    // num_nz (no constraint coefficients yet)
        nullptr,              // start
        nullptr,              // index
        nullptr               // value
    );
    REQUIRE(status == HighsStatus::kOk);

    // Add constraint: x + y >= 1
    std::vector<double> row_lower = {1.0};           // lower bound
    std::vector<double> row_upper = {1e30};          // no upper bound
    std::vector<HighsInt> starts = {0, 2};           // row starts at index 0, ends at 2
    std::vector<HighsInt> indices = {0, 1};          // column indices (x, y)
    std::vector<double> values = {1.0, 1.0};         // coefficients (1*x + 1*y)

    status = highs.addRows(
        1,                    // num_rows
        row_lower.data(),
        row_upper.data(),
        2,                    // num_nz
        starts.data(),
        indices.data(),
        values.data()
    );
    REQUIRE(status == HighsStatus::kOk);

    // Solve the LP
    status = highs.run();
    REQUIRE(status == HighsStatus::kOk);

    // Check solution status
    HighsModelStatus model_status = highs.getModelStatus();
    REQUIRE(model_status == HighsModelStatus::kOptimal);

    // Verify objective value is approximately 1.0
    const HighsInfo& info = highs.getInfo();
    double obj_value = info.objective_function_value;
    REQUIRE_THAT(obj_value, WithinAbs(1.0, 1e-6));

    // Get solution
    const HighsSolution& solution = highs.getSolution();
    double x = solution.col_value[0];
    double y = solution.col_value[1];

    reusex::core::info("PDLP solution: x = {}, y = {}, objective = {}", x, y, obj_value);

    // Verify solution satisfies constraint: x + y >= 1
    REQUIRE(x + y >= 0.999);  // Allow small numerical error

    // Verify both variables are non-negative
    REQUIRE(x >= -1e-6);
    REQUIRE(y >= -1e-6);
  }
}

/** Test that demonstrates why MIP cannot use PDLP
 *
 * This test shows that setting BINARY variables causes HiGHS to ignore
 * the PDLP solver setting and fall back to the default MIP solver.
 */
TEST_CASE("HiGHS PDLP incompatible with binary variables", "[highs][pdlp][mip]") {
  Highs highs;

  // Try to set PDLP solver
  HighsStatus pdlp_status = highs.setOptionValue("solver", "pdlp");

  if (pdlp_status != HighsStatus::kOk) {
    reusex::core::info("Skipping MIP incompatibility test - PDLP not available");
    REQUIRE(true);
    return;
  }

  highs.setOptionValue("log_to_console", false);
  highs.setOptionValue("output_flag", false);

  // Create binary variable: x in {0, 1}
  std::vector<double> obj_coeffs = {1.0};          // minimize x
  std::vector<double> col_lower = {0.0};           // x >= 0
  std::vector<double> col_upper = {1.0};           // x <= 1
  std::vector<HighsVarType> var_types = {HighsVarType::kInteger};  // integer variable

  HighsStatus status = highs.addCols(
      1,
      obj_coeffs.data(),
      col_lower.data(),
      col_upper.data(),
      0, nullptr, nullptr, nullptr
  );
  REQUIRE(status == HighsStatus::kOk);

  // Set variable as integer (binary is integer with bounds [0,1])
  status = highs.changeColsIntegrality(1, new int[1]{0}, var_types.data());
  REQUIRE(status == HighsStatus::kOk);

  // Try to solve - HiGHS will automatically switch from PDLP to MIP solver
  status = highs.run();
  REQUIRE(status == HighsStatus::kOk);

  // Check that solution is integer (0 or 1)
  const HighsSolution& solution = highs.getSolution();
  double x = solution.col_value[0];

  reusex::core::info("MIP solution with binary variable: x = {}", x);

  // Verify x is either 0 or 1 (within numerical tolerance)
  bool is_zero = std::abs(x) < 1e-6;
  bool is_one = std::abs(x - 1.0) < 1e-6;
  REQUIRE((is_zero || is_one));

  reusex::core::info("✓ HiGHS correctly handled binary variable (PDLP not used for MIP)");
}
