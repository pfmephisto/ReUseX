// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "api/ProjectContext.hpp"

#include "api/api.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/logging.hpp>

#include <spdlog/spdlog.h>

#include <map>
#include <set>
#include <utility>

namespace ruxd::api {

namespace pipeline = reusex::pipeline;
using json = nlohmann::json;

ProjectContext::ProjectContext(std::string id, std::filesystem::path project,
                               pipeline::JobScheduler &scheduler,
                               pipeline::StageExecutor executor)
    : id_(std::move(id)), project_(std::move(project)),
      last_used_(Clock::now().time_since_epoch().count()) {
  // Open read-write once, on the way up: this creates and migrates the
  // database if needed, so every later per-request connection can be
  // read-only, and a broken project fails here, at open, rather than on the
  // first fetch (STANDARDS §5).
  //
  // The connection is then kept for as long as the case is open, as the WAL
  // anchor. Every request opens and closes its own connection (ProjectDB is
  // not thread-safe), and without an anchor the last of them to close tries a
  // checkpoint under the file's exclusive lock and drops the WAL index, which
  // the next request then has to rebuild — readers landing in those windows
  // saw SQLITE_BUSY. The anchor holds no transaction and is never queried
  // after this, so it blocks no writer and no checkpoint. It is read-write on
  // purpose: as the last connection to close it checkpoints the WAL into the
  // main file, which a read-only connection cannot.
  wal_anchor_ = std::make_unique<reusex::ProjectDB>(project_,
                                                    /*readOnly=*/false);
  schema_version_ = wal_anchor_->schema_version();

  queue_ = scheduler.make_queue(id_, project_, std::move(executor));
  listener_ = queue_->add_listener([this](const pipeline::JobEvent &event) {
    // A finished stage may have rewritten instances, poses or depth.
    if (event.type == pipeline::JobEvent::Type::finished)
      photo_cache_.invalidate();
    broadcast(event);
  });
  spdlog::info("Case '{}' opened: {} (schema v{})", id_, file_name(),
               schema_version_);
}

ProjectContext::~ProjectContext() {
  stopping_ = true;
  if (warmup_.joinable())
    warmup_.join();
  if (queue_) {
    // Closing the queue cancels its backlog and waits for a running job; its
    // events still reach the subscribers, which are destroyed after this.
    queue_.reset();
  }
  {
    std::lock_guard<std::mutex> lock(subscribers_mutex_);
    subscribers_.clear();
  }
  // Last, once the job worker and every request have released their
  // connections: as the last connection it checkpoints and removes the WAL.
  wal_anchor_.reset();
  spdlog::info("Case '{}' closed", id_);
}

bool ProjectContext::is_busy() const {
  return queue_->is_busy() || queue_->queued_count() > 0;
}

void ProjectContext::start_photo_warmup() {
  if (warmup_.joinable())
    return;
  warmup_ = std::thread([this] {
    try {
      reusex::ProjectDB db(project_, /*readOnly=*/true);
      std::map<std::string, std::set<std::uint32_t>> clouds;
      for (const auto &part : db.survey_parts())
        if (part.cloud_name && part.instance_id && *part.instance_id != 0)
          clouds[*part.cloud_name].insert(*part.instance_id);
      for (const auto &[cloud, wanted] : clouds) {
        if (stopping_)
          return;
        photo_cache_.get(db, cloud, wanted, &stopping_);
      }
    } catch (const reusex::core::OperationCancelled &) {
      spdlog::debug("Photo evidence warm-up of '{}' cancelled", id_);
    } catch (const std::exception &e) {
      spdlog::warn("Photo evidence warm-up of '{}' skipped: {}", id_, e.what());
    }
  });
}

void ProjectContext::touch(Clock::time_point now) noexcept {
  last_used_.store(now.time_since_epoch().count(), std::memory_order_relaxed);
}

ProjectContext::Clock::time_point ProjectContext::last_used() const noexcept {
  return Clock::time_point(
      Clock::duration(last_used_.load(std::memory_order_relaxed)));
}

// --- subscribers -----------------------------------------------------------

json ProjectContext::hello() const {
  json out = hello_json(queue_->jobs(), project_);
  out["case"] = id_;
  return out;
}

void ProjectContext::subscribe(const void *key, SendFn send) {
  const std::string greeting = hello().dump();
  std::lock_guard<std::mutex> lock(subscribers_mutex_);
  // Snapshot on connect, so a client that joins mid-run is immediately
  // consistent without a separate GET /jobs. Sent under the lock, like every
  // send (see broadcast()).
  try {
    send(greeting);
  } catch (const std::exception &e) {
    spdlog::debug("WebSocket send failed: {}", e.what());
  }
  subscribers_[key] = Subscriber{std::move(send), std::nullopt};
}

void ProjectContext::unsubscribe(const void *key) {
  std::lock_guard<std::mutex> lock(subscribers_mutex_);
  subscribers_.erase(key);
}

void ProjectContext::set_filter(const void *key,
                                std::optional<std::string> job_id) {
  std::lock_guard<std::mutex> lock(subscribers_mutex_);
  auto it = subscribers_.find(key);
  if (it != subscribers_.end())
    it->second.filter = std::move(job_id);
}

std::size_t ProjectContext::subscriber_count() const {
  std::lock_guard<std::mutex> lock(subscribers_mutex_);
  return subscribers_.size();
}

void ProjectContext::broadcast(const pipeline::JobEvent &event) {
  json message = job_event_json(event, file_name());
  message["case"] = id_;
  const std::string payload = message.dump();

  // LOCKING INVARIANT — the sends happen INSIDE subscribers_mutex_ on purpose.
  //
  // A subscriber's send callback reaches a crow::websocket::connection we do
  // not own; Crow frees it right after its close handler runs, and that close
  // handler unsubscribes under this same mutex. So a subscriber present in the
  // table cannot be freed while we hold the lock. Snapshotting and sending
  // after unlocking — the obvious-looking version — would race a closing tab
  // against the job worker and send into freed memory. Holding the lock across
  // the send is cheap: Crow's send_text() only posts the frame to its
  // io_context, it never runs a handler inline.
  std::lock_guard<std::mutex> lock(subscribers_mutex_);
  for (auto &[key, subscriber] : subscribers_) {
    if (!event_matches_subscription(event, subscriber.filter))
      continue;
    try {
      subscriber.send(payload);
    } catch (const std::exception &e) {
      spdlog::debug("WebSocket send failed: {}", e.what());
    }
  }
}

void ProjectContext::broadcast_message(const json &message) {
  json out = message;
  out["case"] = id_;
  const std::string payload = out.dump();
  std::lock_guard<std::mutex> lock(subscribers_mutex_);
  for (auto &[key, subscriber] : subscribers_) {
    try {
      subscriber.send(payload);
    } catch (const std::exception &e) {
      spdlog::debug("WebSocket send failed: {}", e.what());
    }
  }
}

} // namespace ruxd::api
