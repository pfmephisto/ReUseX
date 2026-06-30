// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <clients.hpp>

#include <sw/redis++/redis++.h>

#include <utility>

namespace ruxd {

RedisClient::RedisClient(std::string url) : url_(std::move(url)) {}

RedisClient::~RedisClient() = default;

PingResult RedisClient::ping() {
  try {
    // Lazily create the (thread-safe, internally pooled) Redis instance on
    // first use. Construction does not connect; the first command does.
    {
      std::lock_guard<std::mutex> lock(init_mutex_);
      if (!redis_) {
        redis_ = std::make_shared<sw::redis::Redis>(url_);
      }
    }
    const std::string pong = redis_->ping();
    return {pong == "PONG", pong};
  } catch (const std::exception &e) {
    return {false, e.what()};
  }
}

} // namespace ruxd
