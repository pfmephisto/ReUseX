// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/pipeline/JobRunner.hpp"

#include "reusex/core/guid.hpp"
#include "reusex/core/logging.hpp"
#include "reusex/core/processing_observer.hpp"

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

/// Minimum wall-clock gap between two progress events for the same job. The
/// stages call update() per point/voxel; without throttling a WebSocket client
/// would drown. Start/finish transitions are never throttled.
constexpr auto kProgressInterval = std::chrono::milliseconds(100);

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

class JobRunner::Impl {
    public:
  Impl(std::filesystem::path project, StageExecutor executor)
      : project_(std::move(project)), executor_(std::move(executor)) {
    if (!executor_)
      throw std::invalid_argument("JobRunner: stage executor must be callable");
    worker_ = std::thread([this] { run(); });
  }

  ~Impl() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
      cancel_flag_.store(true, std::memory_order_release);
    }
    queue_cv_.notify_all();
    if (worker_.joinable())
      worker_.join();
  }

  const std::filesystem::path &project() const noexcept { return project_; }

  std::string submit(JobStage stage, std::string parameters) {
    // Validate up front so a malformed request fails at submit time with a
    // useful message instead of dying inside the worker (STANDARDS §5).
    if (!parameters.empty()) {
      auto parsed = nlohmann::json::parse(parameters, nullptr,
                                          /*allow_exceptions=*/false);
      if (parsed.is_discarded() || !parsed.is_object())
        throw std::runtime_error(
            "job parameters must be a JSON object (or empty for defaults)");
    }

    JobRecord record;
    record.id = core::generate_guid();
    record.stage = stage;
    record.status = JobStatus::queued;
    record.parameters = std::move(parameters);
    record.submitted_at = iso8601_utc_now();

    JobEvent event;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_)
        throw std::runtime_error("JobRunner is shutting down");
      order_.push_back(record.id);
      queue_.push_back(record.id);
      records_.emplace(record.id, record);
      event = make_event(JobEvent::Type::submitted, record);
    }
    queue_cv_.notify_one();
    info("job {} queued: stage '{}'", event.job.id, to_string(stage));
    emit(event);
    return event.job.id;
  }

  bool cancel(std::string_view id) {
    std::optional<JobEvent> finished;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto it = records_.find(std::string(id));
      if (it == records_.end())
        return false;

      JobRecord &record = it->second;
      if (is_terminal(record.status))
        return true; // Idempotent: already done, nothing to stop.

      record.cancel_requested = true;

      if (record.status == JobStatus::running) {
        // Cooperative: the stage sees the flag at its next check.
        cancel_flag_.store(true, std::memory_order_release);
        return true;
      }

      // Still queued — drop it before it ever starts.
      queue_.erase(std::remove(queue_.begin(), queue_.end(), record.id),
                   queue_.end());
      record.status = JobStatus::cancelled;
      record.finished_at = iso8601_utc_now();
      record.error = "cancelled before execution";
      finished = make_event(JobEvent::Type::finished, record);
    }
    if (finished) {
      info("job {} cancelled before execution", finished->job.id);
      emit(*finished);
      idle_cv_.notify_all();
    }
    return true;
  }

  std::optional<JobRecord> job(std::string_view id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = records_.find(std::string(id));
    if (it == records_.end())
      return std::nullopt;
    return it->second;
  }

  std::vector<JobRecord> jobs() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<JobRecord> out;
    out.reserve(order_.size());
    for (auto it = order_.rbegin(); it != order_.rend(); ++it) {
      auto found = records_.find(*it);
      if (found != records_.end())
        out.push_back(found->second);
    }
    return out;
  }

  size_t queued_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

  bool is_busy() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return running_;
  }

  void wait_idle() {
    std::unique_lock<std::mutex> lock(mutex_);
    idle_cv_.wait(lock, [this] { return queue_.empty() && !running_; });
  }

  size_t add_listener(JobListener listener) {
    std::lock_guard<std::mutex> lock(listeners_mutex_);
    const size_t token = next_listener_token_++;
    listeners_.emplace(token, std::move(listener));
    return token;
  }

  void remove_listener(size_t token) {
    std::lock_guard<std::mutex> lock(listeners_mutex_);
    listeners_.erase(token);
  }

  // --- progress bridging ---------------------------------------------------

  void on_stage_started(core::Stage stage, size_t total) {
    std::optional<JobEvent> event;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (JobRecord *record = current_locked()) {
        record->progress_stage = stage;
        record->progress_total = total;
        record->progress_current = 0;
        event = make_event(JobEvent::Type::progress, *record);
      }
    }
    last_progress_emit_ = std::chrono::steady_clock::now();
    if (event)
      emit(*event);
  }

  void on_stage_updated(core::Stage stage, size_t increment) {
    std::optional<JobEvent> event;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      JobRecord *record = current_locked();
      if (record == nullptr)
        return;
      record->progress_stage = stage;
      record->progress_current += increment;

      const auto now = std::chrono::steady_clock::now();
      if (now - last_progress_emit_ < kProgressInterval)
        return;
      last_progress_emit_ = now;
      event = make_event(JobEvent::Type::progress, *record);
    }
    if (event)
      emit(*event);
  }

  void on_stage_finished(core::Stage stage) {
    std::optional<JobEvent> event;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (JobRecord *record = current_locked()) {
        record->progress_stage = stage;
        if (record->progress_total > 0)
          record->progress_current = record->progress_total;
        event = make_event(JobEvent::Type::progress, *record);
      }
    }
    last_progress_emit_ = std::chrono::steady_clock::now();
    if (event)
      emit(*event);
  }

    private:
  /// Bridges the process-global progress observer onto the running job, while
  /// keeping whatever observer was installed before (e.g. the rux CLI progress
  /// bar) working — it is chained, not replaced.
  class ObserverBridge : public core::IProgressObserver {
      public:
    explicit ObserverBridge(Impl &owner)
        : owner_(owner), previous_(core::get_progress_observer()) {
      core::set_progress_observer(this);
    }
    ~ObserverBridge() override { core::set_progress_observer(previous_); }

    ObserverBridge(const ObserverBridge &) = delete;
    ObserverBridge &operator=(const ObserverBridge &) = delete;

    void on_process_started(core::Stage stage, size_t total) override {
      if (previous_ != nullptr)
        previous_->on_process_started(stage, total);
      owner_.on_stage_started(stage, total);
    }
    void on_process_updated(core::Stage stage, size_t increment) override {
      if (previous_ != nullptr)
        previous_->on_process_updated(stage, increment);
      owner_.on_stage_updated(stage, increment);
    }
    void on_process_finished(core::Stage stage) override {
      if (previous_ != nullptr)
        previous_->on_process_finished(stage);
      owner_.on_stage_finished(stage);
    }

      private:
    Impl &owner_;
    core::IProgressObserver *previous_ = nullptr;
  };

  /// The record of the job currently executing. Caller must hold mutex_.
  JobRecord *current_locked() {
    if (!running_)
      return nullptr;
    auto it = records_.find(current_id_);
    return it == records_.end() ? nullptr : &it->second;
  }

  /// Caller must hold mutex_ (the record is copied into the event).
  static JobEvent make_event(JobEvent::Type type, const JobRecord &record) {
    JobEvent event;
    event.type = type;
    event.timestamp = iso8601_utc_now();
    event.job = record;
    return event;
  }

  void emit(const JobEvent &event) {
    // Copy the listener set, then invoke without any lock held: a listener may
    // legitimately call back into the runner (e.g. to render a status page).
    std::vector<JobListener> targets;
    {
      std::lock_guard<std::mutex> lock(listeners_mutex_);
      targets.reserve(listeners_.size());
      for (const auto &[token, listener] : listeners_)
        targets.push_back(listener);
    }
    for (const auto &listener : targets) {
      try {
        listener(event);
      } catch (const std::exception &e) {
        warn("job listener threw: {}", e.what());
      }
    }
  }

  void run() {
    for (;;) {
      std::string id;
      StageContext ctx;
      JobEvent started;
      std::vector<JobEvent> abandoned;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        queue_cv_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
        if (stopping_) {
          // Shutdown must not block on a backlog: report every still-queued
          // job as cancelled rather than silently dropping it (STANDARDS §5).
          for (const auto &pending : queue_) {
            auto it = records_.find(pending);
            if (it == records_.end() || is_terminal(it->second.status))
              continue;
            it->second.status = JobStatus::cancelled;
            it->second.cancel_requested = true;
            it->second.error = "runner shut down before execution";
            it->second.finished_at = iso8601_utc_now();
            abandoned.push_back(
                make_event(JobEvent::Type::finished, it->second));
          }
          queue_.clear();
        }
        if (stopping_) {
          lock.unlock();
          for (const auto &event : abandoned)
            emit(event);
          idle_cv_.notify_all();
          return;
        }
        if (queue_.empty())
          continue;

        id = queue_.front();
        queue_.pop_front();

        auto it = records_.find(id);
        if (it == records_.end())
          continue;
        JobRecord &record = it->second;
        if (record.status != JobStatus::queued)
          continue; // Cancelled between enqueue and pickup.

        record.status = JobStatus::running;
        record.started_at = iso8601_utc_now();
        running_ = true;
        current_id_ = id;
        cancel_flag_.store(record.cancel_requested, std::memory_order_release);

        ctx.project = project_;
        ctx.stage = record.stage;
        ctx.parameters = record.parameters;
        ctx.cancel_token = &cancel_flag_;

        started = make_event(JobEvent::Type::started, record);
      }

      info("job {} started: stage '{}'", id, to_string(ctx.stage));
      emit(started);

      StageResult result;
      try {
        ObserverBridge bridge(*this);
        result = executor_(ctx);
      } catch (const std::exception &e) {
        result = StageResult::failure(e.what());
      } catch (...) {
        result = StageResult::failure("unknown error");
      }

      JobEvent finished;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        cancel_flag_.store(false, std::memory_order_release);
        auto it = records_.find(id);
        if (it != records_.end()) {
          JobRecord &record = it->second;
          if (result.cancelled || record.cancel_requested)
            record.status = JobStatus::cancelled;
          else
            record.status =
                result.ok ? JobStatus::succeeded : JobStatus::failed;
          if (record.status != JobStatus::succeeded)
            record.error = result.message;
          record.finished_at = iso8601_utc_now();
          finished = make_event(JobEvent::Type::finished, record);
        }
        current_id_.clear();
      }

      info("job {} {}: {}", id, to_string(finished.job.status),
           result.message.empty() ? "(no detail)" : result.message);
      emit(finished);
      idle_cv_.notify_all();
    }
  }

  std::filesystem::path project_;
  StageExecutor executor_;

  mutable std::mutex mutex_;
  std::condition_variable queue_cv_;
  std::condition_variable idle_cv_;
  std::deque<std::string> queue_;
  std::vector<std::string> order_;
  std::unordered_map<std::string, JobRecord> records_;
  std::string current_id_;
  bool running_ = false;
  bool stopping_ = false;

  std::atomic_bool cancel_flag_{false};
  std::chrono::steady_clock::time_point last_progress_emit_{};

  mutable std::mutex listeners_mutex_;
  std::map<size_t, JobListener> listeners_;
  size_t next_listener_token_ = 1;

  std::thread worker_;
};

// ===========================================================================
// JobRunner
// ===========================================================================

JobRunner::JobRunner(std::filesystem::path project, StageExecutor executor)
    : impl_(std::make_unique<Impl>(std::move(project), std::move(executor))) {}

JobRunner::~JobRunner() = default;

const std::filesystem::path &JobRunner::project() const noexcept {
  return impl_->project();
}

std::string JobRunner::submit(JobStage stage, std::string parameters) {
  return impl_->submit(stage, std::move(parameters));
}

bool JobRunner::cancel(std::string_view id) { return impl_->cancel(id); }

std::optional<JobRecord> JobRunner::job(std::string_view id) const {
  return impl_->job(id);
}

std::vector<JobRecord> JobRunner::jobs() const { return impl_->jobs(); }

size_t JobRunner::queued_count() const { return impl_->queued_count(); }

bool JobRunner::is_busy() const { return impl_->is_busy(); }

void JobRunner::wait_idle() { impl_->wait_idle(); }

size_t JobRunner::add_listener(JobListener listener) {
  return impl_->add_listener(std::move(listener));
}

void JobRunner::remove_listener(size_t token) { impl_->remove_listener(token); }

} // namespace reusex::pipeline
