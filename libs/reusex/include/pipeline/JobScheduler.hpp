// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// A process-wide pipeline job scheduler over many projects (spec 2026-10-08,
// phase S2: ruxd serves several cases from one process).
//
// One JobScheduler owns a bounded pool of worker threads. Each project gets a
// JobQueue from it — a FIFO of that project's jobs, with the same submit /
// status / cancel / listener / writer-lock surface JobRunner always had. The
// rules:
//
//  * **At most one running job per queue.** A project's jobs run strictly in
//    submission order, one at a time, under that project's writer lock — two
//    stages writing the same `.rux` at once is never allowed.
//  * **At most `workers` running jobs in total.** The default is one: the GPU
//    is shared, and the stages are TBB-parallel internally already.
//  * **Round-robin between queues.** A free worker takes the next job from the
//    next queue in rotation that has work and is not already running, so a
//    project that queues ten jobs cannot starve one that queues one.
//
// PROGRESS is per job: the worker installs a core::ScopedProgressObserver for
// the job it runs, so two jobs running at once on two workers each see only
// their own stage's events (core/processing_observer.hpp explains why updates
// made from pool threads still arrive).
//
// RECORDS go to an IJobStore at every state transition (JobStore.hpp): in
// memory for `ruxd --local`, Postgres in server mode. Live progress stays here.

#include "reusex/pipeline/JobStore.hpp"
#include "reusex/pipeline/job_types.hpp"
#include "reusex/pipeline/stages.hpp"

#include <chrono>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace reusex::pipeline {

struct JobSchedulerOptions {
  /// Worker threads, i.e. the most jobs that run at the same time across all
  /// queues. Clamped to at least 1.
  std::size_t workers = 1;
};

class JobQueue;

/// The worker pool. Safe to use from any thread.
class JobScheduler {
    public:
  /// @param store Where job records go; nullptr = a fresh InMemoryJobStore.
  explicit JobScheduler(JobSchedulerOptions options = {},
                        std::shared_ptr<IJobStore> store = nullptr);

  /// Cancels every queued job (reported as cancelled), asks every running job
  /// to stop, and joins the workers. Queues may outlive the scheduler; they
  /// then refuse new submissions.
  ~JobScheduler();

  JobScheduler(const JobScheduler &) = delete;
  JobScheduler &operator=(const JobScheduler &) = delete;

  /// A new queue for @p project. @p key names it in the job store (ruxd uses
  /// the case id) and must be unique among this scheduler's live queues.
  /// @throws std::invalid_argument for an empty key, a key already in use, or
  ///         an executor that is not callable.
  std::unique_ptr<JobQueue>
  make_queue(std::string key, std::filesystem::path project,
             StageExecutor executor = default_stage_executor());

  std::size_t workers() const noexcept;

  /// Jobs executing right now, across every queue.
  std::size_t running_count() const;

  /// The store records are written to.
  std::shared_ptr<IJobStore> store() const;

  struct Core; ///< Implementation detail, shared with JobQueue.

    private:
  std::shared_ptr<Core> core_;
};

/// One project's jobs on a JobScheduler. Every member is thread-safe.
class JobQueue {
    public:
  /// Closes the queue: queued jobs are reported cancelled, a running job is
  /// asked to stop, and the destructor waits until it has AND until every
  /// event already raised has reached the listeners — so an object owning
  /// the queue may destroy it and then its listener's captures safely. Must
  /// not run on a thread that is inside one of this queue's listeners.
  ~JobQueue();

  JobQueue(const JobQueue &) = delete;
  JobQueue &operator=(const JobQueue &) = delete;

  const std::string &key() const noexcept;
  const std::filesystem::path &project() const noexcept;

  /// Enqueue a stage run. Returns the new job id.
  /// @param parameters JSON object string; "" means "stage defaults".
  /// @throws std::runtime_error if @p parameters is not a JSON object, or the
  ///         queue or its scheduler is shutting down.
  std::string submit(JobStage stage, std::string parameters = {});

  /// Request cancellation (queued: cancelled now; running: cancel token set;
  /// terminal: no-op). @return false if this queue has no such job.
  bool cancel(std::string_view id);

  /// Snapshot of one of this queue's jobs, or nullopt if unknown or evicted.
  std::optional<JobRecord> job(std::string_view id) const;

  /// Every retained job of this queue, most recently submitted first.
  std::vector<JobRecord> jobs() const;

  /// Jobs of this queue still waiting to start.
  std::size_t queued_count() const;

  /// True while one of this queue's jobs is executing.
  bool is_busy() const;

  /// True while a job is queued or running, or an event is still being
  /// delivered to the listeners — read under ONE lock, so a job moving from
  /// queued to running can never make it read false.
  bool has_work() const;

  /// Block until this queue has nothing queued, nothing running and every
  /// event delivered. Do not call it from a listener.
  void wait_idle();

  /// Try to take the project's exclusive writer lock (see JobRunner).
  [[nodiscard]] WriterLease
  try_acquire_writer(std::chrono::milliseconds timeout);

  /// Register a listener for this queue's events. Returns a removal token.
  std::size_t add_listener(JobListener listener);
  void remove_listener(std::size_t token);

  struct State; ///< Implementation detail.

    private:
  friend class JobScheduler;
  JobQueue(std::shared_ptr<JobScheduler::Core> core,
           std::shared_ptr<State> state);

  std::shared_ptr<JobScheduler::Core> core_;
  std::shared_ptr<State> state_;
};

} // namespace reusex::pipeline
