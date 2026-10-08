// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "api/ProjectRegistry.hpp"

#include "api/api.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <utility>

namespace ruxd::api {

namespace {
/// Least time begin_delete() waits for a closed context's last reference.
constexpr std::chrono::milliseconds kReleaseWait{2000};
} // namespace

ProjectRegistry::ProjectRegistry(std::shared_ptr<ICaseStore> store,
                                 Opener opener, RegistryOptions options,
                                 ClockFn clock)
    : store_(std::move(store)), opener_(std::move(opener)), options_(options),
      clock_(std::move(clock)) {
  if (!store_ || !opener_)
    throw std::invalid_argument("ProjectRegistry needs a case store and an "
                                "opener");
  options_.max_open = std::max<std::size_t>(1, options_.max_open);
  if (options_.sweep_interval.count() > 0)
    sweeper_ = std::thread([this] {
      std::unique_lock<std::mutex> lock(sweeper_mutex_);
      while (!stopping_) {
        sweeper_cv_.wait_for(lock, options_.sweep_interval);
        if (stopping_)
          break;
        lock.unlock();
        try {
          sweep();
        } catch (const std::exception &e) {
          spdlog::warn("Closing idle cases failed: {}", e.what());
        }
        lock.lock();
      }
    });
}

ProjectRegistry::~ProjectRegistry() {
  {
    std::lock_guard<std::mutex> lock(sweeper_mutex_);
    stopping_ = true;
  }
  sweeper_cv_.notify_all();
  if (sweeper_.joinable())
    sweeper_.join();

  std::vector<std::pair<std::string, std::shared_ptr<ProjectContext>>> closing;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    // An open or close in flight finishes first (Crow is stopped by now, so
    // nothing new starts).
    changed_.wait(lock, [this] {
      return std::none_of(entries_.begin(), entries_.end(), [](const auto &e) {
        return e.second.state == State::opening ||
               e.second.state == State::closing;
      });
    });
    for (auto &[id, entry] : entries_)
      if (entry.state == State::open) {
        entry.state = State::closing;
        closing.emplace_back(id, std::move(entry.ctx));
      }
  }
  finish_closing(std::move(closing)); // Each waits for its running job.
}

void ProjectRegistry::set_maintenance(std::function<void()> task) {
  std::lock_guard<std::mutex> lock(sweeper_mutex_);
  maintenance_ = std::move(task);
}

bool ProjectRegistry::is_idle(const Entry &entry) {
  return entry.state == State::open && entry.ctx.use_count() == 1 &&
         entry.ctx->subscriber_count() == 0 && !entry.ctx->is_busy();
}

void ProjectRegistry::finish_closing(
    std::vector<std::pair<std::string, std::shared_ptr<ProjectContext>>>
        closing) {
  if (closing.empty())
    return;
  // Destroy outside the lock: a context's destructor joins its warm-up and
  // its job queue.
  for (auto &[id, ctx] : closing)
    ctx.reset();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto &[id, ctx] : closing) {
      auto it = entries_.find(id);
      if (it != entries_.end() && it->second.state == State::closing)
        entries_.erase(it);
    }
  }
  changed_.notify_all();
}

std::shared_ptr<ProjectContext> ProjectRegistry::acquire(std::string_view id) {
  for (;;) {
    std::unique_lock<std::mutex> lock(mutex_);
    for (;;) {
      auto it = entries_.find(id);
      if (it == entries_.end())
        break;
      if (it->second.state == State::open) {
        it->second.ctx->touch(clock_());
        return it->second.ctx;
      }
      if (it->second.state == State::deleting)
        throw HttpError(409, "case '" + std::string(id) + "' is being deleted");
      // opening or closing: wait for it to settle, then look again.
      changed_.wait(lock);
    }

    lock.unlock();
    const auto info = store_->find(id); // A directory scan; not under the lock.
    if (!info)
      return nullptr;
    lock.lock();
    if (entries_.find(id) != entries_.end())
      continue; // Someone else got there first; take it from the top.

    std::vector<std::pair<std::string, std::shared_ptr<ProjectContext>>>
        evicted;
    auto live = [this] {
      return static_cast<std::size_t>(
          std::count_if(entries_.begin(), entries_.end(), [](const auto &e) {
            return e.second.state == State::open ||
                   e.second.state == State::opening;
          }));
    };
    while (live() >= options_.max_open) {
      auto lru = entries_.end();
      for (auto it = entries_.begin(); it != entries_.end(); ++it)
        if (is_idle(it->second) &&
            (lru == entries_.end() ||
             it->second.ctx->last_used() < lru->second.ctx->last_used()))
          lru = it;
      if (lru == entries_.end())
        throw HttpError(503, "too many cases are open and in use (" +
                                 std::to_string(live()) + "); retry shortly");
      spdlog::info("Closing case '{}' to make room for '{}'", lru->first,
                   info->id);
      lru->second.state = State::closing;
      evicted.emplace_back(lru->first, std::move(lru->second.ctx));
    }
    entries_[info->id] = Entry{State::opening, nullptr};
    lock.unlock();
    finish_closing(std::move(evicted));

    std::shared_ptr<ProjectContext> ctx;
    try {
      ctx = opener_(*info);
    } catch (...) {
      {
        std::lock_guard<std::mutex> relock(mutex_);
        entries_.erase(info->id);
      }
      changed_.notify_all();
      throw;
    }
    ctx->touch(clock_());
    {
      std::lock_guard<std::mutex> relock(mutex_);
      Entry &entry = entries_[info->id];
      entry.state = State::open;
      entry.ctx = ctx;
    }
    changed_.notify_all();
    return ctx;
  }
}

std::shared_ptr<ProjectContext>
ProjectRegistry::find_open(std::string_view id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = entries_.find(id);
  return it != entries_.end() && it->second.state == State::open
             ? it->second.ctx
             : nullptr;
}

std::size_t ProjectRegistry::sweep() {
  const auto cutoff = clock_() - options_.idle_timeout;
  std::vector<std::pair<std::string, std::shared_ptr<ProjectContext>>> closing;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto &[id, entry] : entries_)
      if (is_idle(entry) && entry.ctx->last_used() <= cutoff) {
        spdlog::info("Closing idle case '{}'", id);
        entry.state = State::closing;
        closing.emplace_back(id, std::move(entry.ctx));
      }
  }
  const std::size_t count = closing.size();
  finish_closing(std::move(closing));

  std::function<void()> task;
  {
    std::lock_guard<std::mutex> lock(sweeper_mutex_);
    task = maintenance_;
  }
  if (task)
    task();
  return count;
}

DeleteStart ProjectRegistry::begin_delete(std::string_view id,
                                          std::chrono::milliseconds wait) {
  const auto deadline = std::chrono::steady_clock::now() + wait;
  std::unique_lock<std::mutex> lock(mutex_);
  // An open (a migration, say) or close under way settles first — within the
  // same bound as the lease wait below, so a delete never ties up a request
  // thread for a whole migration (review N6).
  const bool settled = changed_.wait_until(lock, deadline, [&] {
    auto it = entries_.find(id);
    return it == entries_.end() || it->second.state == State::open ||
           it->second.state == State::deleting;
  });
  if (!settled)
    return DeleteStart::in_use;
  auto it = entries_.find(id);
  if (it == entries_.end()) {
    entries_[std::string(id)] = Entry{State::deleting, nullptr};
    return DeleteStart::started;
  }
  if (it->second.state == State::deleting)
    return DeleteStart::deleting;
  if (it->second.ctx->is_busy())
    return DeleteStart::busy;

  // Tombstone now: from here on no new lease is handed out (acquire answers
  // 409) and the sweeper leaves it alone. The context stays in the entry, so
  // "only the registry holds it" is still use_count() == 1.
  it->second.state = State::deleting;
  auto rollback = [&](DeleteStart why) {
    auto again = entries_.find(id);
    again->second.state = State::open;
    lock.unlock();
    changed_.notify_all();
    return why;
  };
  for (;;) {
    auto again = entries_.find(id);
    if (again->second.ctx.use_count() == 1)
      break;
    if (std::chrono::steady_clock::now() >= deadline)
      return rollback(DeleteStart::in_use);
    lock.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    lock.lock();
  }
  auto entry = entries_.find(id);
  // A request that held it may have queued a job before letting go.
  if (entry->second.ctx->is_busy())
    return rollback(DeleteStart::busy);

  std::shared_ptr<ProjectContext> ctx = std::move(entry->second.ctx);
  lock.unlock();
  // The close is certain now: tell the open tabs, then close it HERE so the
  // WAL is checkpointed and the files released before they move.
  ctx->broadcast_message({{"type", "case.closed"}});
  std::weak_ptr<ProjectContext> weak = ctx;
  ctx.reset();
  // Belt to the braces: nothing outside the registry may hold a context
  // without a lease (sockets keep only its SubscriberHub), so this is already
  // the last reference. Should anything ever take one anyway, wait for it
  // (bounded) rather than move files under an open WAL anchor.
  const auto release_deadline =
      std::chrono::steady_clock::now() + std::max(wait, kReleaseWait);
  while (!weak.expired()) {
    if (std::chrono::steady_clock::now() >= release_deadline) {
      spdlog::error("Case '{}' is still referenced after its close; its files "
                    "move with its WAL anchor open",
                    id);
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return DeleteStart::started;
}

void ProjectRegistry::end_delete(std::string_view id) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = entries_.find(id);
    if (it != entries_.end() && it->second.state == State::deleting)
      entries_.erase(it);
  }
  changed_.notify_all();
}

std::size_t ProjectRegistry::open_count() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return static_cast<std::size_t>(
      std::count_if(entries_.begin(), entries_.end(), [](const auto &e) {
        return e.second.state == State::open;
      }));
}

std::vector<std::string> ProjectRegistry::open_ids() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> ids;
  for (const auto &[id, entry] : entries_)
    if (entry.state == State::open)
      ids.push_back(id);
  return ids;
}

} // namespace ruxd::api
