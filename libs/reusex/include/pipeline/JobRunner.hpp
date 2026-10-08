// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// In-process pipeline job runner for ONE project (#265, Phase 1).
//
// A FIFO queue of stage runs on a single worker thread, with a submit /
// status / cancel surface and per-job progress events. Since phase S2 of the
// ruxd multi-case spec it is a thin wrapper: a JobScheduler with one worker
// and one JobQueue (JobScheduler.hpp). Use the scheduler directly to run
// several projects' queues on one bounded worker pool — that is what ruxd
// does — and this class where one project is all there is (the Qt client,
// tests).
//
// PROGRESS: each job's stage reports through a core::ScopedProgressObserver
// installed on the worker thread for that job, chained to whatever global
// observer was installed (e.g. the rux CLI progress bar) — so progress is
// per-job even when several schedulers or workers run stages at once.
//
// THREAD SAFETY: every public member is safe to call from any thread. The
// ProjectDB instance used to execute a job is created and destroyed on the
// worker thread and never escapes it — ProjectDB itself is not thread-safe.
//
// WRITER EXCLUSION: the runner also owns the project's *writer lock* (see
// try_acquire_writer). The worker holds it for the whole execution of a stage,
// so anything else that wants to write to the same `.rux` — the GUI's editor
// endpoints, for instance — can take the same lock and be genuinely exclusive
// with the pipeline rather than merely hoping sqlite sorts it out.

#include "reusex/core/stages.hpp"
#include "reusex/pipeline/job_types.hpp"
#include "reusex/pipeline/stages.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace reusex::pipeline {

/// Tuning knobs for the job store.
struct JobRunnerOptions {
  /// Terminal (succeeded/failed/cancelled) jobs retained before the oldest
  /// are dropped. Queued and running jobs are never evicted.
  ///
  /// 256 is an upper bound, not a working set. A JobRecord is a few hundred
  /// bytes — ids, timestamps, the parameter blob and the result summary — so
  /// the whole retained history costs well under a megabyte, while a day of
  /// GUI use produces tens of jobs, not hundreds. The cap exists so a server
  /// left open for a week cannot grow without limit; it is deliberately far
  /// above anything a user would scroll back through, because the cost of
  /// forgetting a job someone still holds an id for is much higher than the
  /// cost of keeping it.
  size_t max_terminal_jobs = 256;
};

/// FIFO, single-worker, in-process pipeline job runner.
///
/// JOB LIFETIME: the store is in-memory and bounded (#286). Terminal jobs past
/// `JobRunnerOptions::max_terminal_jobs` are evicted oldest-first by submission
/// order, so `job()` returning nullopt means "unknown **or evicted**". Nothing
/// survives a restart.
///
/// WHY NOT PERSIST: `ruxd --local` is an in-process server for a single
/// project, and a job record is a view of work the process itself is doing — a
/// queue position, a cancel token, a live progress counter. None of that is
/// meaningful once the process is gone, and persisting it would make the runner
/// the second writer of a history the pipeline already writes. The durable
/// record is `pipeline_log`, which every stage writes and which carries the
/// driving job id in `pipeline_log.parameters.job_id`; that join is the
/// documented recovery path for a client holding an id the runner no longer
/// knows. Durable job *identity* — jobs that outlive the worker executing them
/// — belongs to Phase 6, where ruxd keeps them in PostgreSQL and one job can
/// migrate between workers; adding a second, weaker persistence layer here
/// first would only have to be undone.
class JobRunner {
    public:
  /// @param project  The `.rux` project every job of this runner operates on.
  /// @param executor Stage execution strategy; defaults to the real pipeline.
  ///                 Injecting a fake makes the state machine testable.
  explicit JobRunner(std::filesystem::path project,
                     StageExecutor executor = default_stage_executor());

  /// As above, with an explicit job-store policy.
  JobRunner(std::filesystem::path project, StageExecutor executor,
            JobRunnerOptions options);

  /// Requests cancellation of anything in flight and joins the worker.
  ~JobRunner();

  JobRunner(const JobRunner &) = delete;
  JobRunner &operator=(const JobRunner &) = delete;

  /// The project every job of this runner targets.
  const std::filesystem::path &project() const noexcept;

  /// Enqueue a stage run. Returns the new job id.
  /// @param parameters JSON object string; "" means "stage defaults".
  /// @throws std::runtime_error if @p parameters is not a JSON object.
  std::string submit(JobStage stage, std::string parameters = {});

  /// Request cancellation.
  /// - queued job  -> immediately cancelled, never executed
  /// - running job -> cancel token set; the stage stops at its next check
  /// - terminal    -> no-op
  /// @return false if no job with that id exists.
  bool cancel(std::string_view id);

  /// Snapshot of one job, or nullopt if the id is unknown **or evicted**.
  ///
  /// The two are indistinguishable here by design: the runner does not keep a
  /// tombstone for a job it has dropped. A caller that needs to tell them apart
  /// looks in `pipeline_log`, which is the durable record — every stage run is
  /// a row there, joined back to its job via `pipeline_log.parameters.job_id`.
  std::optional<JobRecord> job(std::string_view id) const;

  /// Snapshot of every retained job, most recently submitted first.
  ///
  /// Terminal jobs beyond `JobRunnerOptions::max_terminal_jobs` have been
  /// evicted and do not appear; queued and running jobs always do.
  std::vector<JobRecord> jobs() const;

  /// Number of jobs still waiting to start.
  size_t queued_count() const;

  /// True while a job is executing.
  bool is_busy() const;

  /// Block until the queue is empty and no job is running.
  /// Test and shutdown helper; do not call from a listener.
  void wait_idle();

  /// Try to take the project's exclusive writer lock, giving up after
  /// @p timeout.
  ///
  /// The worker holds this lock for the entire execution of a stage, so a
  /// caller that obtains it knows no stage is midway through writing. This is
  /// the mechanism that lets a second writer (the GUI's editor endpoints)
  /// exist at all without racing the pipeline: sqlite would serialize the two
  /// connections anyway, but only at statement granularity and only by
  /// returning SQLITE_BUSY — which cannot protect a read-modify-write such as
  /// renaming one entry of a whole-cloud label map.
  ///
  /// @param timeout How long to wait. Keep it short on a request path: a
  ///        running stage holds the lock for minutes, and answering "busy"
  ///        promptly is far better than stalling the caller until it does.
  /// @return A held lease, or an unheld one (`!owns_lock()`) on timeout.
  ///
  /// Do not call this from a JobListener: the worker publishes events while
  /// holding the lock, so waiting on it from a listener would deadlock.
  [[nodiscard]] WriterLease
  try_acquire_writer(std::chrono::milliseconds timeout);

  /// Register a listener. Returns a token for remove_listener().
  size_t add_listener(JobListener listener);

  /// Unregister a listener. Safe to call with an unknown token.
  void remove_listener(size_t token);

    private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace reusex::pipeline
