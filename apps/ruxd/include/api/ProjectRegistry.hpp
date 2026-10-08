// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The open cases of a ruxd process (spec 2026-10-08, phase S2).
//
// Cases open lazily, on the first request that names them, and close again
// once idle: no request in flight, no WebSocket subscriber, no job queued or
// running, and untouched for `idle_timeout`. At most `max_open` are open at
// once; asking for one more first closes the least recently used idle case,
// and answers 503 when none is idle.
//
// A request holds its case through the shared_ptr acquire() returns (a
// *lease*); the registry holds the other reference. "No lease outstanding" is
// therefore `use_count() == 1`, checked under the registry lock, where no new
// lease can be handed out.
//
// Framework-free; tested in tests/unit/ruxd_api/test_api_registry.cpp.

#include "api/ProjectContext.hpp"
#include "api/cases.hpp"

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace ruxd::api {

struct RegistryOptions {
  /// Most cases open at once. Each holds a sqlite connection and a job queue;
  /// the photo cache is the memory that matters.
  std::size_t max_open = 16;
  /// How long an idle case stays open.
  std::chrono::seconds idle_timeout{std::chrono::minutes(10)};
  /// How often the background sweeper looks for idle cases. Zero disables the
  /// sweeper thread (tests call sweep() themselves).
  std::chrono::seconds sweep_interval{30};
};

class ProjectRegistry {
    public:
  using Clock = ProjectContext::Clock;
  using ClockFn = std::function<Clock::time_point()>;
  /// Opens a case. Server passes one that builds a ProjectContext with the
  /// real stage executor and starts its photo warm-up.
  using Opener =
      std::function<std::shared_ptr<ProjectContext>(const CaseInfo &)>;

  ProjectRegistry(std::shared_ptr<ICaseStore> store, Opener opener,
                  RegistryOptions options = {}, ClockFn clock = Clock::now);

  /// Stops the sweeper and closes every case (running jobs are asked to stop
  /// and waited for).
  ~ProjectRegistry();

  ProjectRegistry(const ProjectRegistry &) = delete;
  ProjectRegistry &operator=(const ProjectRegistry &) = delete;

  /// The open context for @p id, opening it if needed. nullptr when the store
  /// has no such case.
  /// @throws HttpError(503) when max_open cases are open and none is idle.
  /// @throws whatever the opener throws (a project that cannot be opened).
  std::shared_ptr<ProjectContext> acquire(std::string_view id);

  /// The context for @p id if it is open now, else nullptr. Never opens.
  std::shared_ptr<ProjectContext> find_open(std::string_view id) const;

  /// Close every idle case untouched for idle_timeout. @return how many.
  std::size_t sweep();

  /// Close @p id now, whatever its leases (deleting a case). Refused (false)
  /// while a job of it is queued or running. Waits up to @p wait for the
  /// in-flight requests to let go; @return true once the case is fully
  /// closed (or was not open).
  bool force_close(std::string_view id, std::chrono::milliseconds wait);

  std::size_t open_count() const;
  std::vector<std::string> open_ids() const;

  const std::shared_ptr<ICaseStore> &store() const noexcept { return store_; }

    private:
  /// No lease outstanding, no subscriber, no job queued or running. Caller
  /// holds mutex_ (so no lease can be handed out meanwhile).
  static bool is_idle(const std::shared_ptr<ProjectContext> &ctx);

  std::shared_ptr<ICaseStore> store_;
  Opener opener_;
  RegistryOptions options_;
  ClockFn clock_;

  mutable std::mutex mutex_; ///< Guards open_.
  std::map<std::string, std::shared_ptr<ProjectContext>, std::less<>> open_;
  std::mutex open_mutex_; ///< Serializes opening, so a case opens once.

  std::mutex sweeper_mutex_;
  std::condition_variable sweeper_cv_;
  bool stopping_ = false;
  std::thread sweeper_;
};

} // namespace ruxd::api
