// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// In-process pipeline job runner (#265, Phase 1).
//
// Owns a FIFO queue of stage runs and a single worker thread, bridges the
// global core::IProgressObserver singleton into per-job progress events, and
// exposes a submit / status / cancel surface that both `rux gui` (today) and
// ruxd (Phase 6) can put behind the same HTTP contract.
//
// WHY ONE WORKER: progress reporting goes through the process-global observer
// registered with core::set_progress_observer(). Two stages running
// concurrently would interleave into a single observer with no way to tell
// their events apart, so jobs are serialized. The stages themselves are
// TBB-parallel internally and already saturate the machine.
//
// THREAD SAFETY: every public member is safe to call from any thread. The
// ProjectDB instance used to execute a job is created and destroyed on the
// worker thread and never escapes it — ProjectDB itself is not thread-safe.

#include "reusex/core/stages.hpp"
#include "reusex/pipeline/stages.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace reusex::pipeline {

/// Lifecycle of a submitted job.
///
///   queued ──▶ running ──▶ succeeded
///     │           ├──────▶ failed
///     │           └──────▶ cancelled
///     └──────────────────▶ cancelled   (cancelled before it ever started)
enum class JobStatus {
  queued,
  running,
  succeeded,
  failed,
  cancelled,
};

/// Canonical lower-case status name used on the wire.
std::string_view to_string(JobStatus status);

/// Parse a canonical status name. Returns nullopt for an unknown name.
std::optional<JobStatus> parse_job_status(std::string_view name);

/// True for statuses a job can never leave.
bool is_terminal(JobStatus status);

/// UTC timestamp in ISO-8601 form (e.g. "2026-09-08T11:22:33Z").
std::string iso8601_utc_now();

/// A job as reported by the runner. Snapshot; never a live view.
struct JobRecord {
  std::string id; ///< Opaque server-generated identifier (GUID).
  JobStage stage = JobStage::clouds;
  JobStatus status = JobStatus::queued;
  std::string parameters;   ///< JSON object as submitted ("" = defaults).
  std::string error;        ///< Failure reason; empty unless status == failed.
  std::string submitted_at; ///< ISO-8601 UTC.
  std::string started_at;   ///< ISO-8601 UTC; empty while queued.
  std::string finished_at;  ///< ISO-8601 UTC; empty until terminal.
  bool cancel_requested = false;

  /// Live progress, mirrored from the stage's core::ProgressObserver.
  core::Stage progress_stage = core::Stage::idle;
  size_t progress_current = 0;
  size_t progress_total = 0; ///< 0 = indeterminate.
};

/// One notification about a job. Carries the full record so a listener never
/// has to call back into the runner (which would risk lock re-entrancy).
struct JobEvent {
  /// Monotonic emission order, starting at 1.
  ///
  /// Events are *published* without the runner lock held (a listener must never
  /// run under it), so two events can reach a listener out of order — a
  /// `job.submitted` raised on an HTTP thread can lose the race with the
  /// `job.started` the worker raises microseconds later. The sequence number is
  /// assigned under the lock at the moment the state actually changed, so it is
  /// the authoritative ordering; clients must sort by it rather than by
  /// arrival.
  uint64_t sequence = 0;

  enum class Type {
    submitted, ///< Accepted onto the queue.
    started,   ///< Picked up by the worker.
    progress,  ///< Progress counters changed (throttled).
    finished,  ///< Reached a terminal status.
  };

  Type type = Type::submitted;
  std::string timestamp; ///< ISO-8601 UTC.
  JobRecord job;
};

/// Canonical lower-case event-type name used on the wire.
std::string_view to_string(JobEvent::Type type);

/// Listener callback. Invoked on the worker (or submitting) thread with no
/// runner lock held; implementations must be thread-safe and must not block.
using JobListener = std::function<void(const JobEvent &)>;

/// FIFO, single-worker, in-process pipeline job runner.
class JobRunner {
    public:
  /// @param project  The `.rux` project every job of this runner operates on.
  /// @param executor Stage execution strategy; defaults to the real pipeline.
  ///                 Injecting a fake makes the state machine testable.
  explicit JobRunner(std::filesystem::path project,
                     StageExecutor executor = default_stage_executor());

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

  /// Snapshot of one job, or nullopt if the id is unknown.
  std::optional<JobRecord> job(std::string_view id) const;

  /// Snapshot of every known job, most recently submitted first.
  std::vector<JobRecord> jobs() const;

  /// Number of jobs still waiting to start.
  size_t queued_count() const;

  /// True while a job is executing.
  bool is_busy() const;

  /// Block until the queue is empty and no job is running.
  /// Test and shutdown helper; do not call from a listener.
  void wait_idle();

  /// Register a listener. Returns a token for remove_listener().
  size_t add_listener(JobListener listener);

  /// Unregister a listener. Safe to call with an unknown token.
  void remove_listener(size_t token);

    private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace reusex::pipeline
