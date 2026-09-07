// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <route_match.hpp>

#include <cctype>
#include <cstddef>
#include <vector>

namespace ruxd {

namespace {

// Split a path into its non-empty segments. Dropping empty segments is what
// normalizes trailing/duplicated slashes.
std::vector<std::string_view> split_segments(std::string_view path) {
  std::vector<std::string_view> segments;
  std::size_t i = 0;
  while (i < path.size()) {
    while (i < path.size() && path[i] == '/') {
      ++i;
    }
    const std::size_t start = i;
    while (i < path.size() && path[i] != '/') {
      ++i;
    }
    if (i > start) {
      segments.push_back(path.substr(start, i - start));
    }
  }
  return segments;
}

// A whole segment of the form `<...>` — any Crow route parameter.
bool is_placeholder(std::string_view segment) {
  return segment.size() >= 2 && segment.front() == '<' && segment.back() == '>';
}

// Crow's catch-all parameter, which swallows the remaining segments.
bool is_catchall(std::string_view segment) { return segment == "<path>"; }

char lower(char c) {
  return static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
}

} // namespace

std::string_view request_path(std::string_view url) {
  const std::size_t cut = url.find_first_of("?#");
  return cut == std::string_view::npos ? url : url.substr(0, cut);
}

bool route_pattern_matches(std::string_view pattern, std::string_view url) {
  const std::vector<std::string_view> rule = split_segments(pattern);
  const std::vector<std::string_view> path = split_segments(request_path(url));

  for (std::size_t i = 0; i < rule.size(); ++i) {
    if (is_catchall(rule[i])) {
      return path.size() > i; // `<path>` needs at least one segment
    }
    if (i >= path.size()) {
      return false; // rule is longer than the request path
    }
    if (is_placeholder(rule[i])) {
      continue; // single-segment wildcard
    }
    if (rule[i] != path[i]) {
      return false;
    }
  }
  return path.size() == rule.size();
}

bool http_method_equals(std::string_view a, std::string_view b) {
  if (a.size() != b.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (lower(a[i]) != lower(b[i])) {
      return false;
    }
  }
  return true;
}

bool constant_time_equals(std::string_view presented,
                          std::string_view expected) {
  // A length mismatch is folded into the accumulator instead of returning
  // early, so the comparison below still runs over the full presented value.
  volatile unsigned char diff = presented.size() == expected.size() ? 0U : 1U;

  const std::size_t n = presented.size();
  const std::size_t m = expected.size();
  for (std::size_t i = 0; i < n; ++i) {
    const auto lhs = static_cast<unsigned char>(presented[i]);
    // When the lengths differ, `expected` is reused cyclically purely as a
    // same-length dummy; `diff` is already non-zero so the outcome is fixed.
    const auto rhs = m == 0 ? 0U : static_cast<unsigned char>(expected[i % m]);
    diff = static_cast<unsigned char>(diff | (lhs ^ rhs));
  }
  return diff == 0U;
}

} // namespace ruxd
