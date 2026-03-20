// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

namespace ReUseX::utils {
template <typename T>
inline T remap(const T value, const T min_in, const T max_in,
               const T min_out = T(0), T max_out = T(1)) {
  return static_cast<T>(
    static_cast<double>(value - min_in) / static_cast<double>(max_in - min_in) *
    static_cast<double>(max_out - min_out) + static_cast<double>(min_out)
  );
}
} // namespace ReUseX::utils
