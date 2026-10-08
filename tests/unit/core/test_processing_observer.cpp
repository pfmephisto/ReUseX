// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/core/processing_observer.hpp>

#include <catch2/catch_test_macros.hpp>

#include <thread>

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

namespace {
class CountingObserver final : public reusex::core::IProgressObserver {
    public:
  void on_process_started(reusex::core::Stage, size_t) override { ++started; }
  void on_process_updated(reusex::core::Stage, size_t n) override {
    updated += n;
  }
  void on_process_finished(reusex::core::Stage) override { ++finished; }
  int started = 0;
  size_t updated = 0;
  int finished = 0;
};
} // namespace

TEST_CASE("ScopedProgressObserver_Nested_InnermostWinsAndRestores",
          "[core][observer]") {
  CountingObserver global, outer, inner;
  reusex::core::set_progress_observer(&global);
  CHECK(reusex::core::current_progress_observer() == &global);
  {
    reusex::core::ScopedProgressObserver a(&outer);
    CHECK(reusex::core::current_progress_observer() == &outer);
    {
      reusex::core::ScopedProgressObserver b(&inner);
      CHECK(reusex::core::current_progress_observer() == &inner);
    }
    CHECK(reusex::core::current_progress_observer() == &outer);
    // A scope never touches the process-global observer.
    CHECK(reusex::core::get_progress_observer() == &global);
  }
  CHECK(reusex::core::current_progress_observer() == &global);
  reusex::core::reset_progress_observer();
}

TEST_CASE(
    "ProgressObserver_UpdatedFromAnotherThread_ReportsToConstructingScope",
    "[core][observer]") {
  CountingObserver scoped;
  {
    reusex::core::ScopedProgressObserver scope(&scoped);
    reusex::core::ProgressObserver progress(reusex::core::Stage::region_growing,
                                            10);
    // A pool thread has no scope of its own; the observer captured at
    // construction still receives its updates.
    std::thread worker([&] { progress.update(10); });
    worker.join();
  }
  CHECK(scoped.started == 1);
  CHECK(scoped.updated == 10);
  CHECK(scoped.finished == 1);
}

TEST_CASE("ScopedProgressObserver_OnOtherThread_DoesNotLeak",
          "[core][observer]") {
  CountingObserver mine;
  reusex::core::ScopedProgressObserver scope(&mine);
  reusex::core::IProgressObserver *seen = &mine;
  std::thread other([&] { seen = reusex::core::current_progress_observer(); });
  other.join();
  CHECK(seen == nullptr); // No global installed, no scope on that thread.
}
