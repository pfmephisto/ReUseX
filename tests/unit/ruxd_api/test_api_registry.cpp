// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// ProjectRegistry and ProjectContext (ruxd multi-case spec, phase S2): lazy
// open, idle close, the open-case cap, forced close, and per-case WebSocket
// broadcast. A fake clock drives the idle timeout and fake stage executors
// stand in for the pipeline.

#include <catch2/catch_test_macros.hpp>

#include <api/ProjectContext.hpp>
#include <api/ProjectRegistry.hpp>
#include <api/api.hpp>
#include <api/cases.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/pipeline/JobScheduler.hpp>

#include "../../support/temp_path.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
namespace pipeline = reusex::pipeline;
using namespace ruxd::api;
using reusex::test_support::TempDir;

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

/// A directory of @p n real projects named a.rux, b.rux, …
struct CaseDir {
  explicit CaseDir(int n) {
    for (int i = 0; i < n; ++i)
      reusex::ProjectDB db(dir.path / (std::string(1, char('a' + i)) + ".rux"),
                           /*readOnly=*/false);
    store = std::make_shared<LocalCaseStore>(dir.path, fs::path{});
  }
  TempDir dir{"test_api_registry"};
  std::shared_ptr<LocalCaseStore> store;
};

/// A manually advanced clock.
struct FakeClock {
  ProjectRegistry::Clock::time_point now = ProjectRegistry::Clock::now();
  ProjectRegistry::ClockFn fn() {
    return [this] { return now; };
  }
};

RegistryOptions manual(std::size_t max_open = 16) {
  RegistryOptions options;
  options.max_open = max_open;
  options.idle_timeout = std::chrono::minutes(10);
  options.sweep_interval = std::chrono::seconds(0); // tests sweep themselves
  return options;
}

} // namespace

TEST_CASE("ProjectRegistry_Acquire_OpensLazilyOnce", "[ruxd_api][registry]") {
  CaseDir cases(2);
  pipeline::JobScheduler scheduler;
  std::atomic<int> opened{0};
  ProjectRegistry registry(
      cases.store,
      [&](const CaseInfo &info) {
        ++opened;
        return std::make_shared<ProjectContext>(
            info.id, info.path, scheduler, pipeline::default_stage_executor());
      },
      manual());

  CHECK(registry.open_count() == 0); // nothing opens up front
  auto a1 = registry.acquire("a");
  auto a2 = registry.acquire("a");
  REQUIRE(a1);
  CHECK(a1 == a2);
  CHECK(opened == 1);
  CHECK(a1->id() == "a");
  CHECK(a1->schema_version() > 0);
  CHECK(registry.acquire("nope") == nullptr);
  CHECK(registry.acquire("../a") == nullptr);
  CHECK(registry.open_ids() == std::vector<std::string>{"a"});
}

TEST_CASE("ProjectRegistry_IdleCase_ClosedBySweepAfterTimeout",
          "[ruxd_api][registry]") {
  CaseDir cases(2);
  pipeline::JobScheduler scheduler;
  FakeClock clock;
  Gate gate;
  ProjectRegistry registry(
      cases.store,
      [&](const CaseInfo &info) {
        return std::make_shared<ProjectContext>(
            info.id, info.path, scheduler, [&](const pipeline::StageContext &) {
              gate.wait();
              return pipeline::StageResult::success();
            });
      },
      manual(), clock.fn());

  std::weak_ptr<ProjectContext> weak;
  {
    auto ctx = registry.acquire("a");
    weak = ctx;
    clock.now += std::chrono::minutes(20);
    // Leased by a request in flight: never closed, however old.
    CHECK(registry.sweep() == 0);
  }
  // Released, but not idle for long enough since its last use.
  clock.now -= std::chrono::minutes(15);
  CHECK(registry.sweep() == 0);

  SECTION("an open tab (subscriber) keeps it open") {
    int dummy = 0;
    weak.lock()->subscribe(&dummy, [](const std::string &) {});
    clock.now += std::chrono::hours(1);
    CHECK(registry.sweep() == 0);
    weak.lock()->unsubscribe(&dummy);
    CHECK(registry.sweep() == 1);
  }
  SECTION("a running job keeps it open") {
    registry.acquire("a")->jobs().submit(pipeline::JobStage::planes);
    clock.now += std::chrono::hours(1);
    CHECK(registry.sweep() == 0);
    gate.open();
    weak.lock()->jobs().wait_idle();
    CHECK(registry.sweep() == 1);
  }
  SECTION("idle past the timeout: closed, WAL anchor and all") {
    clock.now += std::chrono::minutes(11);
    CHECK(registry.sweep() == 1);
  }
  gate.open();
  CHECK(weak.expired());
  CHECK(registry.open_count() == 0);
  // And it reopens on demand.
  CHECK(registry.acquire("a"));
}

TEST_CASE("ProjectRegistry_Cap_EvictsLeastRecentlyUsedIdleCase",
          "[ruxd_api][registry]") {
  CaseDir cases(3);
  pipeline::JobScheduler scheduler;
  FakeClock clock;
  ProjectRegistry registry(
      cases.store,
      [&](const CaseInfo &info) {
        return std::make_shared<ProjectContext>(
            info.id, info.path, scheduler, pipeline::default_stage_executor());
      },
      manual(/*max_open=*/2), clock.fn());

  registry.acquire("a");
  clock.now += std::chrono::seconds(1);
  registry.acquire("b");
  clock.now += std::chrono::seconds(1);
  registry.acquire("a"); // a is now the more recently used
  clock.now += std::chrono::seconds(1);

  registry.acquire("c");
  CHECK(registry.open_ids() == std::vector<std::string>{"a", "c"});

  // Everything open is in use: a fourth case is refused, not forced.
  auto hold_a = registry.acquire("a");
  auto hold_c = registry.acquire("c");
  try {
    registry.acquire("b");
    FAIL("expected 503");
  } catch (const HttpError &e) {
    CHECK(e.status() == 503);
  }
}

TEST_CASE("ProjectRegistry_ForceClose_RefusedWhileBusy",
          "[ruxd_api][registry]") {
  CaseDir cases(1);
  pipeline::JobScheduler scheduler;
  Gate gate;
  ProjectRegistry registry(
      cases.store,
      [&](const CaseInfo &info) {
        return std::make_shared<ProjectContext>(
            info.id, info.path, scheduler, [&](const pipeline::StageContext &) {
              gate.wait();
              return pipeline::StageResult::success();
            });
      },
      manual());

  auto ctx = registry.acquire("a");
  ctx->jobs().submit(pipeline::JobStage::planes);
  REQUIRE(wait_for([&] { return ctx->jobs().is_busy(); }));
  CHECK_FALSE(registry.force_close("a", std::chrono::milliseconds(10)));
  gate.open();
  ctx->jobs().wait_idle();

  // A lease still held past the wait: put back, not half-closed.
  CHECK_FALSE(registry.force_close("a", std::chrono::milliseconds(20)));
  CHECK(registry.find_open("a") == ctx);
  std::weak_ptr<ProjectContext> weak = ctx;
  ctx.reset();
  CHECK(registry.force_close("a", std::chrono::seconds(1)));
  CHECK(weak.expired()); // closed by force_close itself, files released
  CHECK(registry.force_close("not-open", std::chrono::milliseconds(1)));
}

TEST_CASE("ProjectContext_Broadcast_ReachesOnlyItsOwnSubscribers",
          "[ruxd_api][registry][events]") {
  CaseDir cases(2);
  pipeline::JobScheduler scheduler({2});
  auto ok = [](const pipeline::StageContext &) {
    return pipeline::StageResult::success("done");
  };
  auto a = std::make_shared<ProjectContext>("a", cases.dir.path / "a.rux",
                                            scheduler, ok);
  auto b = std::make_shared<ProjectContext>("b", cases.dir.path / "b.rux",
                                            scheduler, ok);

  std::mutex mutex;
  std::vector<nlohmann::json> to_a, to_b, filtered;
  int ka = 0, kb = 0, kf = 0;
  a->subscribe(&ka, [&](const std::string &m) {
    std::lock_guard<std::mutex> lock(mutex);
    to_a.push_back(nlohmann::json::parse(m));
  });
  b->subscribe(&kb, [&](const std::string &m) {
    std::lock_guard<std::mutex> lock(mutex);
    to_b.push_back(nlohmann::json::parse(m));
  });
  b->subscribe(&kf, [&](const std::string &m) {
    std::lock_guard<std::mutex> lock(mutex);
    filtered.push_back(nlohmann::json::parse(m));
  });
  b->set_filter(&kf, std::string("some-other-job"));

  const auto job = a->jobs().submit(pipeline::JobStage::planes);
  a->jobs().wait_idle();
  a->broadcast_message({{"type", "clouds.changed"}, {"names", {"cloud"}}});

  std::lock_guard<std::mutex> lock(mutex);
  // Each subscriber got its own case's hello first.
  REQUIRE(!to_a.empty());
  CHECK(to_a.front()["type"] == "hello");
  CHECK(to_a.front()["case"] == "a");
  REQUIRE(to_b.size() == 1);
  CHECK(to_b.front()["type"] == "hello");
  CHECK(to_b.front()["case"] == "b");
  REQUIRE(filtered.size() == 1); // hello only

  // Case a's job events and data notices went to case a's socket alone.
  bool saw_finished = false;
  for (const auto &m : to_a) {
    CHECK(m["case"] == "a");
    if (m["type"] == "job.finished") {
      saw_finished = true;
      CHECK(m["job"]["id"] == job);
      CHECK(m["project"] == "a.rux");
    }
  }
  CHECK(saw_finished);
  CHECK(to_a.back()["type"] == "clouds.changed");
}
