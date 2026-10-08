// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Where job records live (spec 2026-10-08, phase S2).
//
// The JobScheduler keeps the *live* state of a job — its place in the queue,
// its cancel token, its progress counters — in memory, because none of that
// means anything once the process is gone. What it hands to an IJobStore is
// the job's *record* at each state transition (submitted, started, finished),
// so a store backed by a database sees a handful of writes per job, never a
// stream of progress ticks.
//
// `ruxd --local` uses InMemoryJobStore. The multi-user server (phase S3)
// backs the same interface with its Postgres `jobs` table.
//
// Records are grouped by *queue*: one queue per project (a ruxd case). Job
// ids are GUIDs, unique across queues, but every lookup names the queue too,
// so one case can never read another case's job by guessing its id.

#include "reusex/pipeline/job_types.hpp"

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace reusex::pipeline {

/// Persistence for job records. Implementations must be thread-safe.
class IJobStore {
    public:
  virtual ~IJobStore() = default;

  /// Insert @p record under @p queue, or replace the record with the same id.
  /// Called at every state transition, never for progress ticks.
  virtual void save(std::string_view queue, const JobRecord &record) = 0;

  /// One record, or nullopt when @p id is unknown in @p queue (or was evicted
  /// by the store's retention policy).
  virtual std::optional<JobRecord> find(std::string_view queue,
                                        std::string_view id) const = 0;

  /// Every retained record of @p queue, most recently submitted first.
  virtual std::vector<JobRecord> list(std::string_view queue) const = 0;
};

/// The in-memory store: bounded per queue, nothing survives a restart.
///
/// Retention is the one JobRunner always had (#286): terminal records past
/// @p max_terminal_jobs per queue are dropped oldest-first by submission
/// order; queued and running records are never dropped.
class InMemoryJobStore final : public IJobStore {
    public:
  explicit InMemoryJobStore(std::size_t max_terminal_jobs = 256);
  ~InMemoryJobStore() override;

  void save(std::string_view queue, const JobRecord &record) override;
  std::optional<JobRecord> find(std::string_view queue,
                                std::string_view id) const override;
  std::vector<JobRecord> list(std::string_view queue) const override;

    private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace reusex::pipeline
