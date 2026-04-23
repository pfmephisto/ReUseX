// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/core/processing_observer.hpp>

#include <catch2/catch_test_macros.hpp>

namespace {
class TestObserver final : public reusex::core::IProgressObserver {};
} // namespace

TEST_CASE("Global processing observer can be registered", "[core][observer]") {
  TestObserver observer;
  reusex::core::set_progress_observer(&observer);

  REQUIRE(reusex::core::get_progress_observer() == &observer);

  reusex::core::reset_progress_observer();
  REQUIRE(reusex::core::get_progress_observer() == nullptr);
}

TEST_CASE("Global processing observer reset clears registration",
          "[core][observer]") {
  TestObserver observer;
  reusex::core::set_progress_observer(&observer);
  reusex::core::reset_progress_observer();

  REQUIRE(reusex::core::get_progress_observer() == nullptr);
}
