// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>

#include <vision/Dataloader.hpp>

#include <algorithm>
#include <cstdint>
#include <optional>
#include <vector>

using reusex::vision::Dataloader;

// Large enough that two independent shuffles matching by chance is
// astronomically unlikely (1/N! for N=256), so the "different seeds ->
// different order" assertions cannot flake.
static constexpr size_t kN = 256;

TEST_CASE("Dataloader shuffle is a valid permutation", "[vision][dataloader]") {
  auto order = Dataloader::shuffled_indices(kN, Dataloader::default_seed);

  REQUIRE(order.size() == kN);

  std::vector<size_t> sorted = order;
  std::sort(sorted.begin(), sorted.end());
  for (size_t i = 0; i < kN; ++i) {
    REQUIRE(sorted[i] == i); // every index present exactly once
  }
}

TEST_CASE("Same seed yields identical shuffle order across constructions",
          "[vision][dataloader]") {
  auto a = Dataloader::shuffled_indices(kN, Dataloader::default_seed);
  auto b = Dataloader::shuffled_indices(kN, Dataloader::default_seed);

  REQUIRE(a == b);

  // An explicit non-default seed is equally deterministic.
  auto c = Dataloader::shuffled_indices(kN, std::optional<uint32_t>(1234));
  auto d = Dataloader::shuffled_indices(kN, std::optional<uint32_t>(1234));

  REQUIRE(c == d);
}

TEST_CASE("Different seeds yield different shuffle order",
          "[vision][dataloader]") {
  auto a = Dataloader::shuffled_indices(kN, std::optional<uint32_t>(1));
  auto b = Dataloader::shuffled_indices(kN, std::optional<uint32_t>(2));

  // Size guard: distinctness is only meaningful for a large index space.
  REQUIRE(a.size() == kN);
  REQUIRE(b.size() == kN);
  REQUIRE(a != b);
}

TEST_CASE("Default seed differs from other fixed seeds",
          "[vision][dataloader]") {
  auto def = Dataloader::shuffled_indices(kN, Dataloader::default_seed);
  auto other = Dataloader::shuffled_indices(
      kN, std::optional<uint32_t>(Dataloader::default_seed + 1));

  REQUIRE(def != other);
}

TEST_CASE("Entropy path (nullopt) still produces a valid permutation",
          "[vision][dataloader]") {
  auto order = Dataloader::shuffled_indices(kN, std::nullopt);

  REQUIRE(order.size() == kN);

  std::vector<size_t> sorted = order;
  std::sort(sorted.begin(), sorted.end());
  for (size_t i = 0; i < kN; ++i) {
    REQUIRE(sorted[i] == i);
  }
}
