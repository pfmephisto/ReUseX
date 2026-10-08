// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The open cases of a ruxd process (spec 2026-10-08, phase S2).
//
// Cases open lazily, on the first request that names them, and close again
// once idle: no request in flight, no WebSocket subscriber, no job queued or
// running (nor its events still being delivered), and untouched for
// `idle_timeout`. At most `max_open` are open at once; asking for one more
// first closes the least recently used idle case, and answers 503 when none
// is idle.
//
// A request holds its case through the shared_ptr acquire() returns (a
// *lease*); the registry holds the other reference. "No lease outstanding" is
// therefore `use_count() == 1`, checked under the registry lock, where no new
// lease can be handed out.
//
// ONE LIFECYCLE PER CASE. Every case id the registry knows is in exactly one
// state, changed only under the registry lock:
//
//   (absent) ──acquire──▶ opening ──▶ open ──sweep/evict──▶ closing ──▶
//   (absent)
//                                       │
//                                       └──begin_delete──▶ deleting
//                                       ──end_delete──▶ (absent)
//   (absent) ──begin_delete──────────────────────────────▶ deleting
//
// `opening` and `closing` are transient: an acquire() that meets one waits for
// it to settle, so a case is never opened while its previous context is still
// being torn down (no second WAL anchor, no clash over the job-queue key).
// `deleting` is a tombstone held for the whole delete — closing the context
// AND moving the files — during which acquire() answers 409; end_delete()
// removes it whether the delete was committed or rolled back.
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

/// Outcome of ProjectRegistry::begin_delete().
enum class DeleteStart {
  started,  ///< Tombstoned and closed: move the files, then end_delete().
  busy,     ///< A job is queued or running; nothing changed.
  in_use,   ///< A request still held the case after the wait; nothing changed.
  deleting, ///< Another delete of it is already under way.
};

class ProjectRegistry {
    public:
  using Clock = ProjectContext::Clock;
  using ClockFn = std::function<Clock::time_point()>;
  /// Opens a case. Server passes one that builds a ProjectContext with the
  /// real stage executor.
  using Opener =
      std::function<std::shared_ptr<ProjectContext>(const CaseInfo &)>;

  ProjectRegistry(std::shared_ptr<ICaseStore> store, Opener opener,
                  RegistryOptions options = {}, ClockFn clock = Clock::now);

  /// Stops the sweeper and closes every case (running jobs are asked to stop
  /// and waited for).
  ~ProjectRegistry();

  ProjectRegistry(const ProjectRegistry &) = delete;
  ProjectRegistry &operator=(const ProjectRegistry &) = delete;

  /// Work the background sweeper also does every sweep_interval (ruxd: expire
  /// abandoned uploads). Set before the first sweep; must not throw.
  void set_maintenance(std::function<void()> task);

  /// The open context for @p id, opening it if needed (waiting out an open or
  /// close of it already under way). nullptr when the store has no such case.
  /// @throws HttpError(409) while the case is being deleted.
  /// @throws HttpError(503) when max_open cases are open and none is idle.
  /// @throws whatever the opener throws (a project that cannot be opened).
  std::shared_ptr<ProjectContext> acquire(std::string_view id);

  /// The context for @p id if it is open now, else nullptr. Never opens.
  std::shared_ptr<ProjectContext> find_open(std::string_view id) const;

  /// Close every idle case untouched for idle_timeout. @return how many.
  std::size_t sweep();

  /// Start deleting @p id: tombstone it and, when it is open, close it HERE
  /// once in-flight requests let go (up to @p wait), so its WAL is
  /// checkpointed and its files closed before the caller moves them. An open
  /// or close of it already under way gets the same @p wait to settle
  /// (`in_use` otherwise). Open tabs get `case.closed` only once the close is
  /// certain. On anything but `started`, nothing changed. After `started`,
  /// call end_delete().
  ///
  /// The close is committed before the files move: if moving them then fails
  /// and end_delete() rolls the delete back, the case exists again but the
  /// tabs that had it open have already been told `case.closed` and gone to
  /// the case list. They can reopen it from there; nothing is lost.
  DeleteStart begin_delete(std::string_view id, std::chrono::milliseconds wait);

  /// Lift the tombstone of a delete begin_delete() started — after the files
  /// moved, or after that failed (the case can then be opened again).
  void end_delete(std::string_view id);

  std::size_t open_count() const;
  std::vector<std::string> open_ids() const;

  const std::shared_ptr<ICaseStore> &store() const noexcept { return store_; }

    private:
  enum class State { opening, open, closing, deleting };
  struct Entry {
    State state = State::opening;
    std::shared_ptr<ProjectContext> ctx; ///< Set only while `open`.
  };

  /// Open, with no lease outstanding (use_count 1 under mutex_), no
  /// subscriber and no job work. Caller holds mutex_.
  static bool is_idle(const Entry &entry);

  /// Close @p contexts outside the lock, then drop their `closing` entries.
  void finish_closing(
      std::vector<std::pair<std::string, std::shared_ptr<ProjectContext>>>
          closing);

  std::shared_ptr<ICaseStore> store_;
  Opener opener_;
  RegistryOptions options_;
  ClockFn clock_;
  std::function<void()> maintenance_;

  mutable std::mutex mutex_;        ///< Guards entries_.
  std::condition_variable changed_; ///< Some entry changed state.
  std::map<std::string, Entry, std::less<>> entries_;

  std::mutex sweeper_mutex_;
  std::condition_variable sweeper_cv_;
  bool stopping_ = false;
  std::thread sweeper_;
};

} // namespace ruxd::api
