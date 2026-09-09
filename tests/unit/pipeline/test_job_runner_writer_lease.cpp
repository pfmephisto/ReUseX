// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// JobRunner writer-lock tests (#265, Phase 4).
//
// The runner owns the project's writer lock and holds it for the whole
// execution of a stage, which is what makes the GUI's editor endpoints
// genuinely exclusive with the pipeline rather than merely unlikely to collide
// with it. These tests pin that in both directions: a stage cannot start while
// a lease is out, and a lease cannot be taken while a stage runs.
//
// A fake StageExecutor stands in for the pipeline throughout, and every
// hand-off goes through a condition variable or a future — no sleep decides
// whether a case passes (STANDARDS §6).

#include <catch2/catch_test_macros.hpp>

#include <pipeline/JobRunner.hpp>
#include <pipeline/stages.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <mutex>
#include <string>
#include <thread>

using namespace reusex::pipeline;
using namespace std::chrono_literals;

namespace {

/// A one-shot latch: `arrive()` on one thread, `wait()` on another.
class Latch {
    public:
  void arrive() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      arrived_ = true;
    }
    cv_.notify_all();
  }

  void wait() {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this] { return arrived_; });
  }

    private:
  std::mutex mutex_;
  std::condition_variable cv_;
  bool arrived_ = false;
};

/// Poll until @p predicate holds, or give up after a generous timeout. Used
/// only to wait for a state that must eventually arrive, never to decide a
/// pass/fail on timing.
template <typename Predicate> bool wait_for(Predicate predicate) {
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate())
      return true;
    std::this_thread::sleep_for(2ms);
  }
  return predicate();
}

/// Take the lease from *another* thread. Re-locking a std::timed_mutex the
/// calling thread already owns is undefined behaviour, so a second attempt
/// must never be made on the thread holding the first.
bool lease_taken_elsewhere(JobRunner &runner, std::chrono::milliseconds wait) {
  return std::async(std::launch::async,
                    [&runner, wait] {
                      auto lease = runner.try_acquire_writer(wait);
                      return lease.owns_lock();
                    })
      .get();
}

} // namespace

TEST_CASE("JobRunnerWriterLease_HeldLease_IsExclusiveAndReleasedOnScopeExit",
          "[pipeline][jobs][writer]") {
  JobRunner runner("/nonexistent/project.rux", [](const StageContext &) {
    return StageResult::success("ok");
  });

  {
    auto lease = runner.try_acquire_writer(1s);
    REQUIRE(lease.owns_lock());

    // A second attempt fails rather than waiting the caller out: a stage holds
    // this lock for minutes, so "busy" is the only honest prompt answer.
    CHECK_FALSE(lease_taken_elsewhere(runner, 10ms));
  }

  // Destruction releases it, with no explicit unlock anywhere.
  auto after = runner.try_acquire_writer(1s);
  CHECK(after.owns_lock());
}

TEST_CASE("JobRunnerWriterLease_ExplicitUnlock_BecomesAvailableImmediately",
          "[pipeline][jobs][writer]") {
  JobRunner runner("/nonexistent/project.rux", [](const StageContext &) {
    return StageResult::success("ok");
  });

  auto lease = runner.try_acquire_writer(1s);
  REQUIRE(lease.owns_lock());
  lease.unlock();
  CHECK_FALSE(lease.owns_lock());

  CHECK(lease_taken_elsewhere(runner, 1s));
}

TEST_CASE("JobRunnerWriterLease_StageExecuting_UnavailableUntilRunnerIdle",
          "[pipeline][jobs][writer]") {
  Latch entered;
  Latch release;
  std::atomic<bool> finished{false};

  JobRunner runner("/nonexistent/project.rux",
                   [&entered, &release, &finished](const StageContext &) {
                     // The worker already holds the writer lock by the time
                     // the executor is called.
                     entered.arrive();
                     release.wait();
                     finished = true;
                     return StageResult::success("ok");
                   });

  const auto id = runner.submit(JobStage::planes);
  entered.wait();

  // The stage is mid-flight, so the lock is genuinely unavailable — this is
  // the 503 path of the editor endpoints.
  CHECK_FALSE(lease_taken_elsewhere(runner, 20ms));
  CHECK(runner.is_busy());
  CHECK_FALSE(finished.load());

  release.arrive();
  runner.wait_idle();

  CHECK(finished.load());
  auto lease = runner.try_acquire_writer(1s);
  CHECK(lease.owns_lock());

  const auto record = runner.job(id);
  REQUIRE(record.has_value());
  CHECK(record->status == JobStatus::succeeded);
}

TEST_CASE("JobRunnerWriterLease_HeldWhileStageQueued_DelaysStageUntilReleased",
          "[pipeline][jobs][writer]") {
  std::atomic<bool> executed{false};

  JobRunner runner("/nonexistent/project.rux",
                   [&executed](const StageContext &) {
                     executed = true;
                     return StageResult::success("ok");
                   });

  {
    auto lease = runner.try_acquire_writer(1s);
    REQUIRE(lease.owns_lock());

    const auto id = runner.submit(JobStage::planes);

    // The job reaches `running` (that transition happens before the lock is
    // taken, so the UI shows it as started rather than stuck in the queue),
    // but the stage body cannot run: this thread owns the lock, so `executed`
    // can never become true while the lease is alive.
    REQUIRE(wait_for([&] {
      const auto record = runner.job(id);
      return record && record->status == JobStatus::running;
    }));
    CHECK_FALSE(executed.load());
  }

  runner.wait_idle();
  CHECK(executed.load());
}
