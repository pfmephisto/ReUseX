// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// JobScheduler tests (ruxd multi-case spec, phase S2): one worker pool over
// many project queues — fairness, the one-running-job-per-queue rule, the
// per-job progress observer, and the job store. Fake executors throughout; no
// project is ever opened.

#include <catch2/catch_test_macros.hpp>

#include <core/processing_observer.hpp>
#include <core/stages.hpp>
#include <pipeline/JobScheduler.hpp>
#include <pipeline/JobStore.hpp>
#include <pipeline/stages.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace reusex::pipeline;
namespace core = reusex::core;

namespace {

class Gate {
    public:
  void wait() {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this] { return open_; });
  }
  void open() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      open_ = true;
    }
    cv_.notify_all();
  }

    private:
  std::mutex mutex_;
  std::condition_variable cv_;
  bool open_ = false;
};

template <typename Predicate> bool wait_for(Predicate predicate) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate())
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return predicate();
}

/// Records the order jobs start in, tagged by the queue's project path.
struct StartLog {
  std::mutex mutex;
  std::vector<std::string> order;
  void add(std::string entry) {
    std::lock_guard<std::mutex> lock(mutex);
    order.push_back(std::move(entry));
  }
  std::vector<std::string> snapshot() {
    std::lock_guard<std::mutex> lock(mutex);
    return order;
  }
};

} // namespace

TEST_CASE("JobScheduler_TwoQueuesOneWorker_ServesRoundRobin",
          "[pipeline][jobs][scheduler]") {
  JobScheduler scheduler({1});
  StartLog log;
  Gate first_running;
  Gate release;
  std::atomic<bool> held{false};

  auto executor = [&](const StageContext &ctx) {
    log.add(ctx.project.string() + ":" + ctx.parameters);
    // Hold the very first job so the rest of the backlog is in place before
    // the scheduler makes its next pick.
    if (!held.exchange(true)) {
      first_running.open();
      release.wait();
    }
    return StageResult::success();
  };
  auto a = scheduler.make_queue("a", "A", executor);
  auto b = scheduler.make_queue("b", "B", executor);

  a->submit(JobStage::planes, R"({"n":1})");
  first_running.wait();
  a->submit(JobStage::planes, R"({"n":2})");
  a->submit(JobStage::planes, R"({"n":3})");
  b->submit(JobStage::planes, R"({"n":1})");
  release.open();

  a->wait_idle();
  b->wait_idle();

  // B's one job is not stuck behind A's whole backlog.
  const std::vector<std::string> expected{R"(A:{"n":1})", R"(B:{"n":1})",
                                          R"(A:{"n":2})", R"(A:{"n":3})"};
  CHECK(log.snapshot() == expected);
}

TEST_CASE("JobScheduler_ManyWorkers_AtMostOneRunningJobPerQueue",
          "[pipeline][jobs][scheduler]") {
  JobScheduler scheduler({4});
  std::mutex mutex;
  std::map<std::string, int> running_now;
  std::map<std::string, int> running_max;
  int total_now = 0;
  int total_max = 0;

  auto executor = [&](const StageContext &ctx) {
    const std::string key = ctx.project.string();
    {
      std::lock_guard<std::mutex> lock(mutex);
      running_max[key] = std::max(running_max[key], ++running_now[key]);
      total_max = std::max(total_max, ++total_now);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    {
      std::lock_guard<std::mutex> lock(mutex);
      --running_now[key];
      --total_now;
    }
    return StageResult::success();
  };
  auto a = scheduler.make_queue("a", "A", executor);
  auto b = scheduler.make_queue("b", "B", executor);
  for (int i = 0; i < 4; ++i) {
    a->submit(JobStage::planes);
    b->submit(JobStage::planes);
  }
  a->wait_idle();
  b->wait_idle();

  // Never two of one project's jobs at once, however many workers are free…
  CHECK(running_max["A"] == 1);
  CHECK(running_max["B"] == 1);
  // …while two projects do run side by side (four workers, two queues).
  CHECK(total_max == 2);
  CHECK(scheduler.running_count() == 0);
}

TEST_CASE("JobScheduler_TwoConcurrentJobs_ProgressDoesNotCross",
          "[pipeline][jobs][scheduler][progress]") {
  JobScheduler scheduler({2});

  // Both stages must be inside their executors at the same time, so the
  // progress really is concurrent.
  std::mutex mutex;
  std::condition_variable cv;
  int arrived = 0;
  auto rendezvous = [&] {
    std::unique_lock<std::mutex> lock(mutex);
    ++arrived;
    cv.notify_all();
    cv.wait(lock, [&] { return arrived >= 2; });
  };

  auto executor = [&](const StageContext &ctx) {
    const size_t total = ctx.project == "A" ? 100 : 7;
    const auto stage = ctx.project == "A" ? core::Stage::region_growing
                                          : core::Stage::ray_tracing;
    core::ProgressObserver progress(stage, total);
    rendezvous();
    // Report from a pool-like thread with no thread-local state, the way a
    // TBB parallel_for body does: it must still land on this job.
    std::thread pool([&] {
      for (size_t i = 0; i < total; ++i)
        progress.update(1);
    });
    pool.join();
    return StageResult::success();
  };
  auto a = scheduler.make_queue("a", "A", executor);
  auto b = scheduler.make_queue("b", "B", executor);

  std::mutex events_mutex;
  std::vector<JobEvent> a_events, b_events;
  a->add_listener([&](const JobEvent &e) {
    std::lock_guard<std::mutex> lock(events_mutex);
    a_events.push_back(e);
  });
  b->add_listener([&](const JobEvent &e) {
    std::lock_guard<std::mutex> lock(events_mutex);
    b_events.push_back(e);
  });

  const auto a_id = a->submit(JobStage::planes);
  const auto b_id = b->submit(JobStage::planes);
  a->wait_idle();
  b->wait_idle();

  const auto a_job = a->job(a_id);
  const auto b_job = b->job(b_id);
  REQUIRE(a_job);
  REQUIRE(b_job);
  CHECK(a_job->progress_total == 100);
  CHECK(a_job->progress_current == 100);
  CHECK(a_job->progress_stage == core::Stage::region_growing);
  CHECK(b_job->progress_total == 7);
  CHECK(b_job->progress_current == 7);
  CHECK(b_job->progress_stage == core::Stage::ray_tracing);

  std::lock_guard<std::mutex> lock(events_mutex);
  for (const auto &e : a_events) {
    CHECK(e.job.id == a_id);
    if (e.type == JobEvent::Type::progress)
      CHECK(e.job.progress_total == 100);
  }
  for (const auto &e : b_events) {
    CHECK(e.job.id == b_id);
    if (e.type == JobEvent::Type::progress)
      CHECK(e.job.progress_total == 7);
  }
}

TEST_CASE("JobScheduler_QueuesAreIsolated_ByKey",
          "[pipeline][jobs][scheduler]") {
  JobScheduler scheduler({1});
  auto ok = [](const StageContext &) { return StageResult::success(); };
  auto a = scheduler.make_queue("a", "A", ok);
  auto b = scheduler.make_queue("b", "B", ok);

  const auto id = a->submit(JobStage::rooms);
  a->wait_idle();
  CHECK(a->job(id).has_value());
  // Another case cannot read or cancel it by id.
  CHECK_FALSE(b->job(id).has_value());
  CHECK_FALSE(b->cancel(id));
  CHECK(b->jobs().empty());

  CHECK_THROWS_AS(scheduler.make_queue("a", "A2", ok), std::invalid_argument);
}

TEST_CASE("JobScheduler_QueueClosedAndReopened_HistorySurvivesInStore",
          "[pipeline][jobs][scheduler]") {
  JobScheduler scheduler({1});
  Gate gate;
  auto held = [&](const StageContext &) {
    gate.wait();
    return StageResult::success();
  };

  std::string done, running, queued;
  std::vector<JobEvent> finished;
  {
    auto q = scheduler.make_queue("case", "P", held);
    q->add_listener([&](const JobEvent &e) {
      if (e.type == JobEvent::Type::finished)
        finished.push_back(e);
    });
    running = q->submit(JobStage::clouds);
    REQUIRE(wait_for([&] { return q->is_busy(); }));
    queued = q->submit(JobStage::planes);
    gate.open(); // Closing waits for the running job.
  }
  // The backlog was reported, not silently dropped.
  REQUIRE(finished.size() == 2);

  // The case closed (idle eviction) and opened again: same key, same history.
  auto again = scheduler.make_queue("case", "P", held);
  const auto listed = again->jobs();
  REQUIRE(listed.size() == 2);
  CHECK(listed[0].id == queued);
  CHECK(listed[0].status == JobStatus::cancelled);
  CHECK(listed[1].id == running);
  CHECK(listed[1].status == JobStatus::succeeded);
}

TEST_CASE("JobScheduler_SharedStore_ReceivesTransitionsNotTicks",
          "[pipeline][jobs][scheduler]") {
  struct CountingStore : IJobStore {
    InMemoryJobStore inner;
    std::atomic<int> saves{0};
    void save(std::string_view q, const JobRecord &r) override {
      ++saves;
      inner.save(q, r);
    }
    std::optional<JobRecord> find(std::string_view q,
                                  std::string_view id) const override {
      return inner.find(q, id);
    }
    std::vector<JobRecord> list(std::string_view q) const override {
      return inner.list(q);
    }
  };
  auto store = std::make_shared<CountingStore>();
  JobScheduler scheduler({1}, store);
  auto q = scheduler.make_queue("c", "P", [](const StageContext &) {
    core::ProgressObserver progress(core::Stage::region_growing, 1000);
    for (int i = 0; i < 1000; ++i)
      progress.update(1);
    return StageResult::success();
  });
  q->submit(JobStage::planes);
  q->wait_idle();
  // submitted + started + finished; the thousand ticks never reach the store.
  CHECK(store->saves.load() == 3);
}
