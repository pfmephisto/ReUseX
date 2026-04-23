// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <reusex/types.hpp>
#include <string>

namespace reusex {
class ProjectDB;
} // namespace reusex

namespace rux::filters {

/// Validation result for filter expressions
struct ValidationResult {
  bool valid;
  std::string error_message;
  std::string resolution_hint;

  operator bool() const { return valid; }
};

/// Evaluate filter expression and generate Indices
/// @param filter_expr Filter expression string (e.g., "planes in [1,2]")
/// @param db ProjectDB containing label clouds
/// @param expected_size Expected point cloud size for validation
/// @return Indices of matching points, or nullptr if filter_expr is empty
/// @throws std::runtime_error on parse error or cloud not found
auto evaluate_filter(const std::string &filter_expr, reusex::ProjectDB &db,
                     size_t expected_size) -> reusex::IndicesPtr;

/// Validate that filter expression is syntactically correct
/// @param filter_expr Expression to validate
/// @return Validation result with error message if invalid
auto validate_expression_syntax(const std::string &expr) -> ValidationResult;

/// Validate that all referenced clouds exist in database
auto validate_clouds_exist(const std::string &expr, reusex::ProjectDB &db)
    -> ValidationResult;

/// Validate that all referenced clouds have matching sizes
auto validate_cloud_sizes(const std::string &expr, reusex::ProjectDB &db)
    -> ValidationResult;

} // namespace rux::filters
