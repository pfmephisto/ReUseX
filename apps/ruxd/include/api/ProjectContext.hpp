// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Everything ruxd keeps per open case (spec 2026-10-08, phase S2).
//
// One ProjectContext per case the server currently has open:
//  * the WAL anchor — a read-write connection held for as long as the case is
//    open, so the WAL index survives between per-request connections and the
//    last close checkpoints the WAL into the `.rux`;
//  * the case's job queue on the process-wide JobScheduler, which owns the
//    project's writer lock;
//  * the PhotoEvidenceCache behind GET /survey/photos, warmed on open;
//  * the WebSocket subscribers of `/api/v1/cases/{cid}/events`, and the
//    broadcast that sends them job events and `clouds.changed`.
//
// Framework-free: a subscriber is a pair of callbacks, so this compiles
// without Crow and is tested in the light binary
// (tests/unit/ruxd_api/test_api_registry.cpp).

#include "api/photo_cache.hpp"

#include <reusex/pipeline/JobScheduler.hpp>

#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace reusex {
class ProjectDB;
}

namespace ruxd::api {

class ProjectContext {
    public:
  using Clock = std::chrono::steady_clock;
  /// Sends one text frame to a subscriber. Called with the subscriber table
  /// locked, so it must not block and must not call back into this context.
  using SendFn = std::function<void(const std::string &)>;

  /// Opens @p project read-write (creating and migrating it if needed) as the
  /// WAL anchor and registers a queue named @p id on @p scheduler.
  /// @throws std::runtime_error when the project cannot be opened.
  ProjectContext(std::string id, std::filesystem::path project,
                 reusex::pipeline::JobScheduler &scheduler,
                 reusex::pipeline::StageExecutor executor);

  /// Stops the warm-up, closes the job queue (a running job is asked to stop
  /// and waited for), drops the subscribers, and closes the WAL anchor last.
  ~ProjectContext();

  ProjectContext(const ProjectContext &) = delete;
  ProjectContext &operator=(const ProjectContext &) = delete;

  const std::string &id() const noexcept { return id_; }
  const std::filesystem::path &project() const noexcept { return project_; }
  /// The `.rux` file name — what the wire calls `project`.
  std::string file_name() const { return project_.filename().string(); }
  /// The schema version the anchor saw on open.
  int schema_version() const noexcept { return schema_version_; }

  reusex::pipeline::JobQueue &jobs() noexcept { return *queue_; }
  PhotoEvidenceCache &photo_cache() noexcept { return photo_cache_; }

  /// A job is queued or running, or its event is still being delivered
  /// (one lock: JobQueue::has_work).
  bool is_busy() const;

  /// Compute the survey's photo evidence on a background thread, so the rest
  /// of Kortlægning's photos are ready by the time they are asked for. Best
  /// effort; thread-safe; only the first call does anything. Called on a
  /// case's first survey request — never on open, so merely opening a case
  /// (or listing cases) costs no photo work.
  void start_photo_warmup();

  // --- idle tracking (ProjectRegistry) -------------------------------------

  void touch(Clock::time_point now) noexcept;
  Clock::time_point last_used() const noexcept;

  // --- WebSocket subscribers ------------------------------------------------

  /// Add a subscriber under @p key (e.g. the connection's address) and send it
  /// the `hello` snapshot.
  void subscribe(const void *key, SendFn send);
  /// Remove one; unknown keys are ignored.
  void unsubscribe(const void *key);
  /// Set (or clear, with nullopt) a subscriber's job filter.
  void set_filter(const void *key, std::optional<std::string> job_id);
  std::size_t subscriber_count() const;

  /// The `hello` frame for a new subscriber.
  nlohmann::json hello() const;
  /// A job event to every subscriber whose filter matches.
  void broadcast(const reusex::pipeline::JobEvent &event);
  /// A non-job message (`clouds.changed`, `case.closed`) to every subscriber.
  void broadcast_message(const nlohmann::json &message);

    private:
  void launch_photo_warmup();

  struct Subscriber {
    SendFn send;
    std::optional<std::string> filter; ///< nullopt = every event.
  };

  const std::string id_;
  const std::filesystem::path project_;

  // MEMBER ORDER IS LOAD-BEARING (destruction runs in reverse). The WAL anchor
  // is declared first so it closes last, after the queue's worker and the
  // warm-up have released their connections; the subscriber table is
  // declared before the queue so a job event published while the queue
  // shuts down still finds it.
  std::unique_ptr<reusex::ProjectDB> wal_anchor_;
  int schema_version_ = 0;

  mutable std::mutex subscribers_mutex_;
  std::map<const void *, Subscriber> subscribers_;

  PhotoEvidenceCache photo_cache_;
  std::atomic<bool> stopping_{false};
  std::once_flag warmup_once_;
  std::thread warmup_;

  std::unique_ptr<reusex::pipeline::JobQueue> queue_;
  std::size_t listener_ = 0;

  std::atomic<Clock::rep> last_used_;
};

} // namespace ruxd::api
