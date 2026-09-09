// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/core/processing_observer.hpp>

#include <catch2/catch_test_macros.hpp>

namespace {
class TestObserver final : public reusex::core::IProgressObserver {};
} // namespace

TEST_CASE("SetProgressObserver_ValidObserver_RegistersAsGlobalObserver",
          "[core][observer]") {
  TestObserver observer;
  reusex::core::set_progress_observer(&observer);

  REQUIRE(reusex::core::get_progress_observer() == &observer);

  reusex::core::reset_progress_observer();
  REQUIRE(reusex::core::get_progress_observer() == nullptr);
}

TEST_CASE("ResetProgressObserver_AfterRegistration_ClearsGlobalObserver",
          "[core][observer]") {
  TestObserver observer;
  reusex::core::set_progress_observer(&observer);
  reusex::core::reset_progress_observer();

  REQUIRE(reusex::core::get_progress_observer() == nullptr);
}
