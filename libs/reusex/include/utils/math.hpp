// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <numbers>

namespace reusex::utils {

/** @brief Convert degrees to radians. */
template <typename T>
constexpr inline T deg_to_rad(T degrees) {
  return degrees * std::numbers::pi_v<T> / T(180);
}

/** @brief Convert radians to degrees. */
template <typename T>
constexpr inline T rad_to_deg(T radians) {
  return radians * T(180) / std::numbers::pi_v<T>;
}
template <typename T>
inline T remap(const T value, const T min_in, const T max_in,
               const T min_out = T(0), T max_out = T(1)) {
  return static_cast<T>(
    static_cast<double>(value - min_in) / static_cast<double>(max_in - min_in) *
    static_cast<double>(max_out - min_out) + static_cast<double>(min_out)
  );
}
} // namespace reusex::utils
