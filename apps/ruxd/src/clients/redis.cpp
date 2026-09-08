// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <clients.hpp>

#include <sw/redis++/redis++.h>

#include <memory>
#include <utility>

namespace ruxd {

RedisClient::RedisClient(std::string url) : url_(std::move(url)) {}

RedisClient::~RedisClient() = default;

PingResult RedisClient::ping() {
  try {
    // Lazily create the (thread-safe, internally pooled) Redis instance on
    // first use. Construction does not connect; the first command does.
    //
    // The shared_ptr member is only ever touched under init_mutex_: concurrent
    // /readyz requests would otherwise race a read of redis_ against the write
    // that installs it (#282). Take a local copy while holding the lock and
    // issue the command through that — sw::redis::Redis is itself thread-safe,
    // so the actual ping must not (and need not) hold the mutex.
    std::shared_ptr<sw::redis::Redis> redis;
    {
      std::lock_guard<std::mutex> lock(init_mutex_);
      if (!redis_) {
        redis_ = std::make_shared<sw::redis::Redis>(url_);
      }
      redis = redis_;
    }
    const std::string pong = redis->ping();
    return {pong == "PONG", pong};
  } catch (const std::exception &e) {
    return {false, e.what()};
  }
}

} // namespace ruxd
