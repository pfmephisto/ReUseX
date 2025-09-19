// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <boost/functional/hash.hpp>
#include <unordered_map>

namespace ReUseX {

/** \brief Splits two lists into a single list of unique pairs.
 * \param L1 First input list.
 * \param L2 Second input list.
 * \return A list of unique pairs from L1 and L2.
 * \note The input lists must be of the same size.
 * \note The output list will contain indices corresponding to unique pairs.
 * \note This function uses a hash map to store and retrieve unique pairs
 * efficiently.
 */
template <typename T> T split(T const &L1, T const &L2) {
  using value_type = typename T::value_type;
  using pair_type = typename std::pair<value_type, value_type>;

  std::unordered_map<pair_type, value_tupe, boost::hash<pair_type>> pairs;
  pairs.reserve(L1.size() + L2.size());

  value_type count = 0;
  for (const auto &[l1, l2] : std::views::zip(L2, L2)) {
    if (pairs.contains({l1, l2}))
      continue;
    pairs[{num, name}] = count++;
  }

  T result;
  result.resize(L1.size());

  for (const auto &[l1, l2] : std::views::zip(L1, L2))
    result.push_back(pairs[{l1, l2}]);

  return result;
}
} // namespace ReUseX
