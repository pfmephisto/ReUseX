// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include "filter_utils.hpp"
#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/filter_expression.hpp>
#include <algorithm>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <regex>
#include <spdlog/spdlog.h>

namespace rux::filters {

namespace {

// Extract cloud names referenced in expression
auto extract_cloud_names(const std::string &expr) -> std::vector<std::string> {
  std::vector<std::string> names;

  // Simple regex to find identifiers before operators
  // Matches: word followed by (in, ==, !=, >=, <=, >, <)
  std::regex cloud_pattern(R"((\w+)\s*(?:in|==|!=|>=|<=|>|<))");
  std::smatch match;

  std::string::const_iterator search_start(expr.cbegin());
  while (std::regex_search(search_start, expr.cend(), match, cloud_pattern)) {
    names.push_back(match[1].str());
    search_start = match.suffix().first;
  }

  // Remove duplicates
  std::sort(names.begin(), names.end());
  names.erase(std::unique(names.begin(), names.end()), names.end());

  return names;
}

} // anonymous namespace

auto evaluate_filter(const std::string &filter_expr, ReUseX::ProjectDB &db,
                     size_t expected_size) -> ReUseX::IndicesPtr {
  if (filter_expr.empty()) {
    return nullptr;
  }

  // Parse expression
  auto expr = ReUseX::core::parse_filter_expression(filter_expr, db);

  // Validate cloud size matches expected
  if (!expr->clouds.empty()) {
    size_t actual_size = expr->clouds[0].size();
    if (actual_size != expected_size) {
      throw std::runtime_error(fmt::format(
          "Cloud size mismatch: filter references clouds with {} points, "
          "but expected {} points",
          actual_size, expected_size));
    }
  }

  // Evaluate filter
  auto indices = ReUseX::core::evaluate_filter(*expr, expected_size);

  // Log results
  spdlog::info("Filter '{}' matched {} points ({:.1f}% of cloud)", filter_expr,
               indices->size(), 100.0 * indices->size() / expected_size);

  // Warn if result is empty
  if (indices->empty()) {
    spdlog::warn("Filter matched 0 points - output will be empty");
  }

  // Warn if result is very small
  if (indices->size() < 100 && indices->size() > 0) {
    spdlog::warn("Filter matched only {} points ({:.2f}% of cloud) - "
                 "this may be too small for reliable processing",
                 indices->size(), 100.0 * indices->size() / expected_size);
  }

  return indices;
}

auto validate_expression_syntax(const std::string &expr) -> ValidationResult {
  if (expr.empty()) {
    return {false, "Empty filter expression", "Provide a valid filter string"};
  }

  // Try to tokenize (without loading clouds from DB)
  // We can't fully validate without a DB, but we can check basic syntax
  try {
    // Basic syntax checks
    size_t open_paren = std::count(expr.begin(), expr.end(), '(');
    size_t close_paren = std::count(expr.begin(), expr.end(), ')');
    if (open_paren != close_paren) {
      return {false, "Mismatched parentheses",
              "Ensure all '(' have matching ')'"};
    }

    size_t open_bracket = std::count(expr.begin(), expr.end(), '[');
    size_t close_bracket = std::count(expr.begin(), expr.end(), ']');
    if (open_bracket != close_bracket) {
      return {false, "Mismatched brackets", "Ensure all '[' have matching ']'"};
    }

    // Check for 'in' keyword usage
    if (expr.find(" in ") != std::string::npos ||
        expr.find(" in[") != std::string::npos) {
      // Find position after 'in'
      size_t in_pos = expr.find(" in ");
      if (in_pos == std::string::npos) {
        in_pos = expr.find(" in[");
      }
      size_t bracket_pos = expr.find('[', in_pos);
      if (bracket_pos == std::string::npos) {
        return {false, "'in' operator requires bracket syntax",
                "Use 'cloud in [value1, value2, ...]'"};
      }
    }

    return {true, "", ""};
  } catch (const std::exception &e) {
    return {false, fmt::format("Syntax error: {}", e.what()),
            "Check expression follows pattern: <cloud> <op> <value>"};
  }
}

auto validate_clouds_exist(const std::string &expr, ReUseX::ProjectDB &db)
    -> ValidationResult {
  auto cloud_names = extract_cloud_names(expr);

  for (const auto &name : cloud_names) {
    if (!db.has_point_cloud(name)) {
      // Get available clouds
      auto available = db.list_point_clouds();

      // Try to find similar names (basic fuzzy match)
      std::vector<std::string> suggestions;
      for (const auto &avail : available) {
        // Simple substring match
        if (avail.find(name) != std::string::npos ||
            name.find(avail) != std::string::npos) {
          suggestions.push_back(avail);
        }
      }

      std::string hint;
      if (!suggestions.empty()) {
        hint = fmt::format("Did you mean '{}'? ", suggestions[0]);
      }
      hint += fmt::format("Available clouds: {}. "
                          "Run 'rux list clouds <project.rux>' for full list.",
                          fmt::join(available, ", "));

      return {false, fmt::format("Label cloud '{}' not found", name), hint};
    }
  }

  return {true, "", ""};
}

auto validate_cloud_sizes(const std::string &expr, ReUseX::ProjectDB &db)
    -> ValidationResult {
  auto cloud_names = extract_cloud_names(expr);

  if (cloud_names.size() <= 1) {
    return {true, "", ""}; // Single cloud, no mismatch possible
  }

  std::optional<size_t> expected_size;
  std::string first_cloud;

  for (const auto &name : cloud_names) {
    auto cloud = db.point_cloud_label(name);

    if (!expected_size) {
      expected_size = cloud->size();
      first_cloud = name;
    } else if (cloud->size() != *expected_size) {
      return {false,
              fmt::format("Cloud '{}' has {} points, but '{}' has {} points",
                          name, cloud->size(), first_cloud, *expected_size),
              "All referenced clouds must have the same number of points. "
              "Use only one cloud in filter, or re-run segmentation."};
    }
  }

  return {true, "", ""};
}

} // namespace rux::filters
