// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

namespace ReUseX::utils {
template <typename T>
inline T remap(const T value, const T min_in, const T max_in,
               const T min_out = T(0), T max_out = T(1)) {
  return (value - min_in) / (max_in - min_in) * (max_out - min_out) + min_out;
}
} // namespace ReUseX::utils
