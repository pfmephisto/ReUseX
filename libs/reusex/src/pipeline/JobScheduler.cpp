// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/pipeline/JobScheduler.hpp"

#include "reusex/core/guid.hpp"
#include "reusex/core/logging.hpp"
#include "reusex/core/processing_observer.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>

namespace reusex::pipeline {
namespace {

/// Minimum wall-clock gap between two progress events for the same job. The
/// stages call update() per point/voxel; without throttling a WebSocket client
/// would drown. Start/finish transitions are never throttled.
constexpr auto kProgressInterval = std::chrono::milliseconds(100);

} // namespace

// ===========================================================================
// State
// ===========================================================================

/// One queue's state. Everything not marked otherwise is guarded by
/// JobScheduler::Core::mutex.
struct JobQueue::State {
  State(std::string k, std::filesystem::path p, StageExecutor e)
      : key(std::move(k)), project(std::move(p)), executor(std::move(e)) {}

  const std::string key;
  const std::filesystem::path project;
  const StageExecutor executor;

  std::deque<std::string> pending; ///< Queued ids, FIFO.
  /// Queued and running jobs, with live progress. Terminal jobs live only in
  /// the store.
  std::unordered_map<std::string, JobRecord> live;
  bool running = false;
  std::string current_id;
  bool closed = false;
  uint64_t sequence = 0; ///< See JobEvent::sequence; per queue.
  /// Events made but not yet delivered to every listener. A queue is not
  /// idle — and may not be destroyed — while this is non-zero: a listener
  /// typically captures the object that owns the queue.
  std::size_t emitting = 0;

  /// The running job's cancel token (one running job per queue).
  std::atomic_bool cancel_flag{false};

  /// The project's writer lock. Not guarded by Core::mutex; lock order is
  /// writer -> Core::mutex (the worker publishes progress while holding it).
  std::timed_mutex writer;

  std::mutex listeners_mutex; ///< Guards the two members below.
  std::map<std::size_t, JobListener> listeners;
  std::size_t next_listener_token = 1;
};

struct JobScheduler::Core {
  JobSchedulerOptions options;
  std::shared_ptr<IJobStore> store;

  mutable std::mutex mutex;
  std::condition_variable work_cv; ///< A queue may have become runnable.
  std::condition_variable idle_cv; ///< A job finished or was cancelled.

  /// Live queues in rotation order; `cursor` is where the next pick starts.
  std::vector<std::shared_ptr<JobQueue::State>> rotation;
  std::size_t cursor = 0;
  std::size_t running = 0;
  bool stopping = false;

  std::vector<std::thread> workers;

  // --- helpers (caller holds `mutex` unless noted) --------------------------

  /// Every event made here MUST then go through emit(): making it counts it
  /// as in flight (State::emitting), in the same critical section as the
  /// state change it reports, so no observer of that state change can see
  /// the queue idle before its listeners have run.
  JobEvent make_event(JobQueue::State &q, JobEvent::Type type,
                      const JobRecord &record) {
    ++q.emitting;
    JobEvent event;
    event.sequence = ++q.sequence;
    event.type = type;
    event.timestamp = iso8601_utc_now();
    event.job = record;
    return event;
  }

  /// Without `mutex`: invoke @p q's listeners, then mark the event delivered.
  /// A listener may call back into the queue (e.g. to render a status page),
  /// so no lock is held while it runs.
  void emit(JobQueue::State &q, const JobEvent &event) {
    deliver(q, event);
    {
      std::lock_guard<std::mutex> lock(mutex);
      --q.emitting;
    }
    idle_cv.notify_all();
  }

  static void deliver(JobQueue::State &q, const JobEvent &event) {
    std::vector<JobListener> targets;
    {
      std::lock_guard<std::mutex> lock(q.listeners_mutex);
      targets.reserve(q.listeners.size());
      for (const auto &[token, listener] : q.listeners)
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

  /// Cancel every queued job of @p q, returning the events to emit.
  std::vector<JobEvent> abandon_pending(JobQueue::State &q,
                                        std::string_view reason) {
    std::vector<JobEvent> events;
    for (const auto &id : q.pending) {
      auto it = q.live.find(id);
      if (it == q.live.end())
        continue;
      JobRecord &record = it->second;
      record.status = JobStatus::cancelled;
      record.cancel_requested = true;
      record.error = std::string(reason);
      record.finished_at = iso8601_utc_now();
      events.push_back(make_event(q, JobEvent::Type::finished, record));
      store->save(q.key, record);
      q.live.erase(it);
    }
    q.pending.clear();
    if (q.running) {
      q.cancel_flag.store(true, std::memory_order_release);
      auto it = q.live.find(q.current_id);
      if (it != q.live.end())
        it->second.cancel_requested = true;
    }
    return events;
  }

  /// The next queue to serve: the first in rotation (from `cursor`) that is
  /// open, idle and has work. Advances the cursor past it — round robin.
  std::shared_ptr<JobQueue::State> pick() {
    const std::size_t n = rotation.size();
    for (std::size_t i = 0; i < n; ++i) {
      const std::size_t index = (cursor + i) % n;
      auto &q = rotation[index];
      if (!q->closed && !q->running && !q->pending.empty()) {
        cursor = (index + 1) % n;
        return q;
      }
    }
    return nullptr;
  }

  void worker_loop();
  /// Run a job already marked running by worker_loop(), then finish it.
  void execute(const std::shared_ptr<JobQueue::State> &q,
               const StageContext &ctx, const JobEvent &started);
};

// ===========================================================================
// Progress bridge
// ===========================================================================

namespace {

/// Routes one job's stage progress onto its record. Installed per job on the
/// worker thread via core::ScopedProgressObserver, and chained to whatever
/// observer was current before (the rux CLI's progress bar, say), so that one
/// keeps working.
///
/// Ticks are counted in an atomic and only every kProgressInterval does one
/// take the scheduler lock to publish the count: stages call update() per
/// point or voxel from TBB pool threads, and with several workers a lock per
/// tick would make every running stage contend on one mutex.
class ProgressBridge final : public core::IProgressObserver {
    public:
  ProgressBridge(JobScheduler::Core &core, JobQueue::State &q)
      : core_(core), q_(q), previous_(core::current_progress_observer()) {}

  void on_process_started(core::Stage stage, size_t total) override {
    if (previous_ != nullptr)
      previous_->on_process_started(stage, total);
    current_.store(0, std::memory_order_relaxed);
    stage_.store(static_cast<int>(stage), std::memory_order_relaxed);
    last_emit_ns_.store(now_ns(), std::memory_order_relaxed);
    std::optional<JobEvent> event;
    {
      std::lock_guard<std::mutex> lock(core_.mutex);
      if (JobRecord *record = current()) {
        record->progress_stage = stage;
        record->progress_total = total;
        record->progress_current = 0;
        event = core_.make_event(q_, JobEvent::Type::progress, *record);
      }
    }
    if (event)
      core_.emit(q_, *event);
  }

  void on_process_updated(core::Stage stage, size_t increment) override {
    if (previous_ != nullptr)
      previous_->on_process_updated(stage, increment);
    current_.fetch_add(increment, std::memory_order_relaxed);
    stage_.store(static_cast<int>(stage), std::memory_order_relaxed);

    // Throttle without the lock: only the thread that wins the timestamp
    // swap publishes.
    const auto now = now_ns();
    auto last = last_emit_ns_.load(std::memory_order_relaxed);
    if (now - last < interval_ns() || !last_emit_ns_.compare_exchange_strong(
                                          last, now, std::memory_order_relaxed))
      return;

    std::optional<JobEvent> event;
    {
      std::lock_guard<std::mutex> lock(core_.mutex);
      JobRecord *record = current();
      if (record == nullptr)
        return;
      record->progress_stage =
          static_cast<core::Stage>(stage_.load(std::memory_order_relaxed));
      record->progress_current = current_.load(std::memory_order_relaxed);
      event = core_.make_event(q_, JobEvent::Type::progress, *record);
    }
    core_.emit(q_, *event);
  }

  void on_process_finished(core::Stage stage) override {
    if (previous_ != nullptr)
      previous_->on_process_finished(stage);
    std::optional<JobEvent> event;
    {
      std::lock_guard<std::mutex> lock(core_.mutex);
      if (JobRecord *record = current()) {
        record->progress_stage = stage;
        record->progress_current =
            record->progress_total > 0
                ? record->progress_total
                : current_.load(std::memory_order_relaxed);
        event = core_.make_event(q_, JobEvent::Type::progress, *record);
      }
    }
    last_emit_ns_.store(now_ns(), std::memory_order_relaxed);
    if (event)
      core_.emit(q_, *event);
  }

    private:
  static std::int64_t now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }
  static constexpr std::int64_t interval_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               kProgressInterval)
        .count();
  }

  /// The queue's running record. Caller holds core_.mutex.
  JobRecord *current() {
    if (!q_.running)
      return nullptr;
    auto it = q_.live.find(q_.current_id);
    return it == q_.live.end() ? nullptr : &it->second;
  }

  JobScheduler::Core &core_;
  JobQueue::State &q_;
  core::IProgressObserver *previous_ = nullptr;
  std::atomic<size_t> current_{0};
  std::atomic<int> stage_{0};
  std::atomic<std::int64_t> last_emit_ns_{0};
};

} // namespace

// ===========================================================================
// Worker
// ===========================================================================

void JobScheduler::Core::worker_loop() {
  for (;;) {
    std::shared_ptr<JobQueue::State> q;
    StageContext ctx;
    JobEvent started;
    {
      // Picking a queue and marking its job running is one critical section:
      // otherwise a second worker could pick the same queue, or a cancel could
      // empty it, in between.
      std::unique_lock<std::mutex> lock(mutex);
      for (;;) {
        if (stopping)
          return;
        q = pick();
        if (q)
          break;
        work_cv.wait(lock);
      }
      const std::string id = q->pending.front();
      q->pending.pop_front();
      JobRecord &record = q->live.at(id); // Pending ids are always live.
      record.status = JobStatus::running;
      record.started_at = iso8601_utc_now();
      q->running = true;
      q->current_id = id;
      ++running;
      q->cancel_flag.store(record.cancel_requested, std::memory_order_release);

      ctx.project = q->project;
      ctx.stage = record.stage;
      ctx.parameters = record.parameters;
      ctx.cancel_token = &q->cancel_flag;
      ctx.job_id = record.id;

      started = make_event(*q, JobEvent::Type::started, record);
      store->save(q->key, record);
    }
    execute(q, ctx, started);
  }
}

void JobScheduler::Core::execute(const std::shared_ptr<JobQueue::State> &q,
                                 const StageContext &ctx,
                                 const JobEvent &started) {
  const std::string &id = ctx.job_id;
  info("job {} started: stage '{}' ({})", id, to_string(ctx.stage), q->key);
  emit(*q, started);

  StageResult result;
  {
    // Hold the project's writer lock across the whole stage, so the editor
    // endpoints writing the same file from request threads are genuinely
    // exclusive with it. Taken after `started` is emitted, so a job blocked
    // behind a long editor write still shows as running rather than silently
    // sitting in the queue.
    std::lock_guard<std::timed_mutex> writer(q->writer);
    try {
      ProgressBridge bridge(*this, *q);
      core::ScopedProgressObserver scope(&bridge);
      result = q->executor(ctx);
    } catch (const std::exception &e) {
      result = StageResult::failure(e.what());
    } catch (...) {
      result = StageResult::failure("unknown error");
    }
  }

  JobEvent finished;
  {
    std::lock_guard<std::mutex> lock(mutex);
    q->running = false;
    --running;
    q->cancel_flag.store(false, std::memory_order_release);
    auto it = q->live.find(id);
    if (it != q->live.end()) {
      JobRecord &record = it->second;
      // Only the stage's own report decides the outcome. A cancel REQUEST the
      // stage could not honour (it finished first, or it does not poll the
      // token) must not be dressed up as a cancellation — the work was done
      // and persisted. `cancel_requested` stays visible either way.
      if (result.cancelled)
        record.status = JobStatus::cancelled;
      else
        record.status = result.ok ? JobStatus::succeeded : JobStatus::failed;
      if (record.status != JobStatus::succeeded) {
        record.error = result.message;
      } else {
        // What the run produced, so a client knows what to re-fetch. Only on
        // success: a failure's message is already in `error`.
        record.result_summary = result.message;
        record.result_outputs = result.outputs;
        if (record.cancel_requested)
          info("job {} had a cancel request the stage could not honour; it "
               "completed normally",
               id);
      }
      record.finished_at = iso8601_utc_now();
      finished = make_event(*q, JobEvent::Type::finished, record);
      store->save(q->key, record);
      q->live.erase(it);
    }
    q->current_id.clear();
  }

  info("job {} {}: {}", id, to_string(finished.job.status),
       result.message.empty() ? "(no detail)" : result.message);
  emit(*q, finished);
  idle_cv.notify_all();
  // This queue may have more work, which another worker can now take.
  work_cv.notify_all();
}

// ===========================================================================
// JobScheduler
// ===========================================================================

JobScheduler::JobScheduler(JobSchedulerOptions options,
                           std::shared_ptr<IJobStore> store)
    : core_(std::make_shared<Core>()) {
  core_->options = options;
  core_->options.workers = std::max<std::size_t>(1, options.workers);
  core_->store =
      store ? std::move(store) : std::make_shared<InMemoryJobStore>();
  for (std::size_t i = 0; i < core_->options.workers; ++i)
    core_->workers.emplace_back([core = core_.get()] { core->worker_loop(); });
}

JobScheduler::~JobScheduler() {
  std::vector<std::pair<std::shared_ptr<JobQueue::State>, JobEvent>> abandoned;
  {
    std::lock_guard<std::mutex> lock(core_->mutex);
    core_->stopping = true;
    for (const auto &q : core_->rotation) {
      // Joining below can block for the whole remaining runtime of a stage
      // that does not poll the cancel token. Say so, rather than look hung.
      if (q->running) {
        auto it = q->live.find(q->current_id);
        if (it != q->live.end()) {
          const auto stage = it->second.stage;
          if (stage_supports_cancellation(stage))
            info("shutting down: asking job {} (stage '{}') to stop",
                 q->current_id, to_string(stage));
          else
            warn("shutting down: waiting for job {} (stage '{}') to finish — "
                 "that stage cannot be interrupted",
                 q->current_id, to_string(stage));
        }
      }
      for (auto &event :
           core_->abandon_pending(*q, "runner shut down before execution"))
        abandoned.emplace_back(q, std::move(event));
    }
  }
  core_->work_cv.notify_all();
  for (const auto &[q, event] : abandoned)
    core_->emit(*q, event);
  for (auto &worker : core_->workers)
    if (worker.joinable())
      worker.join();
  core_->idle_cv.notify_all();
}

std::unique_ptr<JobQueue>
JobScheduler::make_queue(std::string key, std::filesystem::path project,
                         StageExecutor executor) {
  if (key.empty())
    throw std::invalid_argument("JobScheduler: queue key must not be empty");
  if (!executor)
    throw std::invalid_argument(
        "JobScheduler: stage executor must be callable");
  auto state = std::make_shared<JobQueue::State>(key, std::move(project),
                                                 std::move(executor));
  {
    std::lock_guard<std::mutex> lock(core_->mutex);
    for (const auto &q : core_->rotation)
      if (q->key == key)
        throw std::invalid_argument("JobScheduler: queue '" + key +
                                    "' already exists");
    core_->rotation.push_back(state);
  }
  return std::unique_ptr<JobQueue>(new JobQueue(core_, std::move(state)));
}

std::size_t JobScheduler::workers() const noexcept {
  return core_->options.workers;
}

std::size_t JobScheduler::running_count() const {
  std::lock_guard<std::mutex> lock(core_->mutex);
  return core_->running;
}

std::shared_ptr<IJobStore> JobScheduler::store() const { return core_->store; }

// ===========================================================================
// JobQueue
// ===========================================================================

JobQueue::JobQueue(std::shared_ptr<JobScheduler::Core> core,
                   std::shared_ptr<State> state)
    : core_(std::move(core)), state_(std::move(state)) {}

JobQueue::~JobQueue() {
  auto &core = *core_;
  std::vector<JobEvent> abandoned;
  {
    std::lock_guard<std::mutex> lock(core.mutex);
    state_->closed = true;
    abandoned = core.abandon_pending(*state_, "queue closed before execution");
  }
  for (const auto &event : abandoned)
    core.emit(*state_, event);
  core.idle_cv.notify_all();

  std::unique_lock<std::mutex> lock(core.mutex);
  // Not just "not running": the finished event's listeners must have run
  // too, or one could still be reaching into whatever owns this queue.
  core.idle_cv.wait(
      lock, [this] { return !state_->running && state_->emitting == 0; });
  auto &rotation = core.rotation;
  auto it = std::find(rotation.begin(), rotation.end(), state_);
  if (it != rotation.end()) {
    const auto index = static_cast<std::size_t>(it - rotation.begin());
    rotation.erase(it);
    if (core.cursor > index)
      --core.cursor;
    if (core.cursor >= rotation.size())
      core.cursor = 0;
  }
}

const std::string &JobQueue::key() const noexcept { return state_->key; }

const std::filesystem::path &JobQueue::project() const noexcept {
  return state_->project;
}

std::string JobQueue::submit(JobStage stage, std::string parameters) {
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

  auto &core = *core_;
  JobEvent event;
  {
    std::lock_guard<std::mutex> lock(core.mutex);
    if (core.stopping || state_->closed)
      throw std::runtime_error("the job queue is shutting down");
    state_->pending.push_back(record.id);
    state_->live.emplace(record.id, record);
    event = core.make_event(*state_, JobEvent::Type::submitted, record);
    core.store->save(state_->key, record);
  }
  core.work_cv.notify_all();
  info("job {} queued: stage '{}' ({})", event.job.id, to_string(stage),
       state_->key);
  core.emit(*state_, event);
  return event.job.id;
}

bool JobQueue::cancel(std::string_view id) {
  auto &core = *core_;
  std::optional<JobEvent> finished;
  {
    std::lock_guard<std::mutex> lock(core.mutex);
    auto it = state_->live.find(std::string(id));
    if (it == state_->live.end())
      // Terminal (idempotent no-op) or unknown.
      return core.store->find(state_->key, id).has_value();

    JobRecord &record = it->second;
    record.cancel_requested = true;
    if (record.status == JobStatus::running) {
      // Cooperative: the stage sees the flag at its next check.
      state_->cancel_flag.store(true, std::memory_order_release);
      return true;
    }

    // Still queued — drop it before it ever starts.
    auto &pending = state_->pending;
    pending.erase(std::remove(pending.begin(), pending.end(), record.id),
                  pending.end());
    record.status = JobStatus::cancelled;
    record.finished_at = iso8601_utc_now();
    record.error = "cancelled before execution";
    finished = core.make_event(*state_, JobEvent::Type::finished, record);
    core.store->save(state_->key, record);
    state_->live.erase(it);
  }
  info("job {} cancelled before execution", finished->job.id);
  core.emit(*state_, *finished);
  core.idle_cv.notify_all();
  return true;
}

std::optional<JobRecord> JobQueue::job(std::string_view id) const {
  std::lock_guard<std::mutex> lock(core_->mutex);
  auto it = state_->live.find(std::string(id));
  if (it != state_->live.end())
    return it->second;
  return core_->store->find(state_->key, id);
}

std::vector<JobRecord> JobQueue::jobs() const {
  std::lock_guard<std::mutex> lock(core_->mutex);
  auto records = core_->store->list(state_->key);
  // The store saw queued/running jobs only at their transitions; the live copy
  // carries the current progress.
  for (auto &record : records) {
    auto it = state_->live.find(record.id);
    if (it != state_->live.end())
      record = it->second;
  }
  return records;
}

std::size_t JobQueue::queued_count() const {
  std::lock_guard<std::mutex> lock(core_->mutex);
  return state_->pending.size();
}

bool JobQueue::is_busy() const {
  std::lock_guard<std::mutex> lock(core_->mutex);
  return state_->running;
}

bool JobQueue::has_work() const {
  std::lock_guard<std::mutex> lock(core_->mutex);
  return state_->running || !state_->pending.empty() || state_->emitting != 0;
}

void JobQueue::wait_idle() {
  std::unique_lock<std::mutex> lock(core_->mutex);
  core_->idle_cv.wait(lock, [this] {
    return state_->pending.empty() && !state_->running && state_->emitting == 0;
  });
}

WriterLease JobQueue::try_acquire_writer(std::chrono::milliseconds timeout) {
  WriterLease lease(state_->writer, std::defer_lock);
  (void)lease.try_lock_for(timeout); // owns_lock() is authoritative
  return lease;
}

std::size_t JobQueue::add_listener(JobListener listener) {
  std::lock_guard<std::mutex> lock(state_->listeners_mutex);
  const std::size_t token = state_->next_listener_token++;
  state_->listeners.emplace(token, std::move(listener));
  return token;
}

void JobQueue::remove_listener(std::size_t token) {
  std::lock_guard<std::mutex> lock(state_->listeners_mutex);
  state_->listeners.erase(token);
}

} // namespace reusex::pipeline
