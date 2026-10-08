// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Locale-independent text -> number. std::stod / strtod / std::stof follow
// the C locale's LC_NUMERIC: a process that calls setlocale(LC_ALL, "") —
// every QApplication does — reads "799.85" as 799 under da_DK, and stops a
// strtod loop dead at the '.' (Q2 review: the Qt client's intrinsics). These
// use std::from_chars, which never consults the locale.

#include <charconv>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>

namespace reusex::utils {

/// Parse a number at the start of @p s, after optional whitespace and a '+'
/// (strtod accepts both; from_chars does not). Sets @p consumed to the
/// characters used (0 when there is no number). Out-of-range values are
/// reported as no number.
template <typename T>
std::optional<T> parse_number(std::string_view s,
                              std::size_t *consumed = nullptr) {
  std::size_t i = 0;
  while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' ||
                          s[i] == '\r' || s[i] == '\f' || s[i] == '\v'))
    ++i;
  if (i < s.size() && s[i] == '+' &&
      !(i + 1 < s.size() && (s[i + 1] == '-' || s[i + 1] == '+')))
    ++i;
  T value{};
  const auto r = std::from_chars(s.data() + i, s.data() + s.size(), value);
  if (r.ec != std::errc{}) {
    if (consumed)
      *consumed = 0;
    return std::nullopt;
  }
  if (consumed)
    *consumed = static_cast<std::size_t>(r.ptr - s.data());
  return value;
}

/// std::stod's contract without its locale: throws std::invalid_argument
/// when @p s does not start with a number; @p pos gets the characters used.
inline double to_double(std::string_view s, std::size_t *pos = nullptr) {
  std::size_t used = 0;
  const auto v = parse_number<double>(s, &used);
  if (!v)
    throw std::invalid_argument("not a number: '" + std::string(s) + "'");
  if (pos)
    *pos = used;
  return *v;
}

/// std::stof's contract without its locale.
inline float to_float(std::string_view s, std::size_t *pos = nullptr) {
  std::size_t used = 0;
  const auto v = parse_number<float>(s, &used);
  if (!v)
    throw std::invalid_argument("not a number: '" + std::string(s) + "'");
  if (pos)
    *pos = used;
  return *v;
}

} // namespace reusex::utils
