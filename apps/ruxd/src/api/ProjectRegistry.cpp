// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "api/ProjectRegistry.hpp"

#include "api/api.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <utility>

namespace ruxd::api {

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

  std::map<std::string, std::shared_ptr<ProjectContext>, std::less<>> closing;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    closing.swap(open_);
  }
  closing.clear(); // Each waits for its running job, outside the lock.
}

bool ProjectRegistry::is_idle(const std::shared_ptr<ProjectContext> &ctx) {
  return ctx.use_count() == 1 && ctx->subscriber_count() == 0 &&
         !ctx->is_busy();
}

std::shared_ptr<ProjectContext> ProjectRegistry::acquire(std::string_view id) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto it = open_.find(id); it != open_.end()) {
      it->second->touch(clock_());
      return it->second;
    }
  }

  const auto info = store_->find(id);
  if (!info)
    return nullptr;

  // One opener at a time: two requests racing for the same new case must not
  // open it twice (two WAL anchors, two queues for one project).
  std::lock_guard<std::mutex> opening(open_mutex_);
  std::vector<std::shared_ptr<ProjectContext>> evicted;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto it = open_.find(id); it != open_.end()) {
      it->second->touch(clock_());
      return it->second;
    }
    while (open_.size() >= options_.max_open) {
      auto lru = open_.end();
      for (auto it = open_.begin(); it != open_.end(); ++it)
        if (is_idle(it->second) &&
            (lru == open_.end() ||
             it->second->last_used() < lru->second->last_used()))
          lru = it;
      if (lru == open_.end())
        throw HttpError(503, "too many cases are open and in use (" +
                                 std::to_string(open_.size()) +
                                 "); retry shortly");
      spdlog::info("Closing case '{}' to make room for '{}'", lru->first,
                   info->id);
      evicted.push_back(std::move(lru->second));
      open_.erase(lru);
    }
  }
  evicted.clear(); // Closed outside the registry lock.

  auto ctx = opener_(*info);
  ctx->touch(clock_());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    open_.emplace(info->id, ctx);
  }
  return ctx;
}

std::shared_ptr<ProjectContext>
ProjectRegistry::find_open(std::string_view id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = open_.find(id);
  return it == open_.end() ? nullptr : it->second;
}

std::size_t ProjectRegistry::sweep() {
  const auto cutoff = clock_() - options_.idle_timeout;
  std::vector<std::shared_ptr<ProjectContext>> closing;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto it = open_.begin(); it != open_.end();) {
      if (is_idle(it->second) && it->second->last_used() <= cutoff) {
        spdlog::info("Closing idle case '{}'", it->first);
        closing.push_back(std::move(it->second));
        it = open_.erase(it);
      } else {
        ++it;
      }
    }
  }
  const std::size_t count = closing.size();
  closing.clear();
  return count;
}

bool ProjectRegistry::force_close(std::string_view id,
                                  std::chrono::milliseconds wait) {
  std::shared_ptr<ProjectContext> ctx;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = open_.find(id);
    if (it == open_.end())
      return true;
    if (it->second->is_busy())
      return false;
    ctx = std::move(it->second);
    open_.erase(it);
  }
  // Tell open tabs the case went away; their sockets stop receiving.
  ctx->broadcast_message({{"type", "case.closed"}});

  // Wait for in-flight requests to drop their leases, then close it HERE, so
  // the WAL anchor has checkpointed and closed before the caller moves the
  // files.
  const auto deadline = std::chrono::steady_clock::now() + wait;
  while (ctx.use_count() > 1 && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  if (ctx.use_count() == 1) {
    ctx.reset();
    return true;
  }

  // A request still holds it: put it back rather than leave a half-closed
  // case that a new request would open a second time.
  std::lock_guard<std::mutex> lock(mutex_);
  open_.emplace(std::string(id), std::move(ctx));
  return false;
}

std::size_t ProjectRegistry::open_count() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return open_.size();
}

std::vector<std::string> ProjectRegistry::open_ids() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> ids;
  for (const auto &[id, ctx] : open_)
    ids.push_back(id);
  return ids;
}

} // namespace ruxd::api
