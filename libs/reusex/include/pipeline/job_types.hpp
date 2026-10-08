// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The vocabulary shared by every job runner: a job's record, its status, the
// events it raises, and the writer lease that makes a running stage exclusive
// with other writers of the same project. Split out of JobRunner.hpp so the
// JobScheduler and the job stores can use it without the runner.

#include "reusex/core/stages.hpp"
#include "reusex/pipeline/stages.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
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
  /// Who submitted it — an opaque id the caller chose (ruxd: the user id);
  /// empty when nobody in particular did. Stored, never interpreted here.
  std::string submitted_by;
  bool cancel_requested = false;

  /// Live progress, mirrored from the stage's core::ProgressObserver.
  core::Stage progress_stage = core::Stage::idle;
  size_t progress_current = 0;
  size_t progress_total = 0; ///< 0 = indeterminate.

  /// The stage's own summary of what it produced, and the artifacts it wrote.
  /// Populated only when `status == succeeded`; empty otherwise. A failed or
  /// cancelled job reports through `error` instead — the two are never both
  /// set, so a client never has to decide which one it is being told.
  std::string result_summary;
  std::vector<StageArtifact> result_outputs;
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

/// Exclusive right to write to a runner's project.
///
/// Empty (`!lease.owns_lock()`) when the lock could not be taken in time.
/// Releases on destruction, so a handler simply keeps it alive for as long as
/// its `ProjectDB` write handle is open.
using WriterLease = std::unique_lock<std::timed_mutex>;

} // namespace reusex::pipeline
