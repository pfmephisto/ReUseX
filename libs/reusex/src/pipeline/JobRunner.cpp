// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/pipeline/JobRunner.hpp"

#include "reusex/pipeline/JobScheduler.hpp"

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <ctime>
#include <deque>
#include <map>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>

namespace reusex::pipeline {
namespace {

constexpr std::array<std::pair<JobStatus, std::string_view>, 5> kStatusNames{{
    {JobStatus::queued, "queued"},
    {JobStatus::running, "running"},
    {JobStatus::succeeded, "succeeded"},
    {JobStatus::failed, "failed"},
    {JobStatus::cancelled, "cancelled"},
}};

constexpr std::array<std::pair<JobEvent::Type, std::string_view>, 4>
    kEventNames{{
        {JobEvent::Type::submitted, "job.submitted"},
        {JobEvent::Type::started, "job.started"},
        {JobEvent::Type::progress, "job.progress"},
        {JobEvent::Type::finished, "job.finished"},
    }};

} // namespace

std::string_view to_string(JobStatus status) {
  for (const auto &[value, name] : kStatusNames)
    if (value == status)
      return name;
  return "unknown";
}

std::optional<JobStatus> parse_job_status(std::string_view name) {
  for (const auto &[value, candidate] : kStatusNames)
    if (candidate == name)
      return value;
  return std::nullopt;
}

bool is_terminal(JobStatus status) {
  return status == JobStatus::succeeded || status == JobStatus::failed ||
         status == JobStatus::cancelled;
}

std::string_view to_string(JobEvent::Type type) {
  for (const auto &[value, name] : kEventNames)
    if (value == type)
      return name;
  return "job.unknown";
}

std::string iso8601_utc_now() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t seconds = std::chrono::system_clock::to_time_t(now);
  std::tm utc{};
#if defined(_WIN32)
  gmtime_s(&utc, &seconds);
#else
  gmtime_r(&seconds, &utc);
#endif
  return fmt::format("{:04d}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}Z",
                     utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                     utc.tm_hour, utc.tm_min, utc.tm_sec);
}

// ===========================================================================
// JobRunner::Impl
// ===========================================================================

/// A JobRunner is a one-worker JobScheduler with a single queue.
class JobRunner::Impl {
    public:
  Impl(std::filesystem::path project, StageExecutor executor,
       JobRunnerOptions options)
      : scheduler_(JobSchedulerOptions{1}, std::make_shared<InMemoryJobStore>(
                                               options.max_terminal_jobs)),
        queue_(scheduler_.make_queue("default", std::move(project),
                                     std::move(executor))) {}

  ~Impl() {
    // The queue first: it cancels the backlog and waits for the running job,
    // while the scheduler's worker is still there to finish it.
    queue_.reset();
  }

  JobQueue &queue() { return *queue_; }
  const JobQueue &queue() const { return *queue_; }

    private:
  JobScheduler scheduler_;
  std::unique_ptr<JobQueue> queue_;
};

// ===========================================================================
// JobRunner
// ===========================================================================

JobRunner::JobRunner(std::filesystem::path project, StageExecutor executor)
    : JobRunner(std::move(project), std::move(executor), JobRunnerOptions{}) {}

JobRunner::JobRunner(std::filesystem::path project, StageExecutor executor,
                     JobRunnerOptions options)
    : impl_(std::make_unique<Impl>(std::move(project), std::move(executor),
                                   options)) {}

JobRunner::~JobRunner() = default;

const std::filesystem::path &JobRunner::project() const noexcept {
  return impl_->queue().project();
}

std::string JobRunner::submit(JobStage stage, std::string parameters) {
  return impl_->queue().submit(stage, std::move(parameters));
}

bool JobRunner::cancel(std::string_view id) {
  return impl_->queue().cancel(id);
}

std::optional<JobRecord> JobRunner::job(std::string_view id) const {
  return impl_->queue().job(id);
}

std::vector<JobRecord> JobRunner::jobs() const { return impl_->queue().jobs(); }

size_t JobRunner::queued_count() const { return impl_->queue().queued_count(); }

bool JobRunner::is_busy() const { return impl_->queue().is_busy(); }

void JobRunner::wait_idle() { impl_->queue().wait_idle(); }

WriterLease JobRunner::try_acquire_writer(std::chrono::milliseconds timeout) {
  return impl_->queue().try_acquire_writer(timeout);
}

size_t JobRunner::add_listener(JobListener listener) {
  return impl_->queue().add_listener(std::move(listener));
}

void JobRunner::remove_listener(size_t token) {
  impl_->queue().remove_listener(token);
}

} // namespace reusex::pipeline
