// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/background.hpp>

#include <chrono>
#include <condition_variable>
#include <mutex>

namespace rux::qt {
namespace {

std::mutex &mutex() {
  static std::mutex m;
  return m;
}
std::condition_variable &cv() {
  static std::condition_variable c;
  return c;
}
int &count() {
  static int n = 0;
  return n;
}

} // namespace

BackgroundWork::BackgroundWork() {
  std::lock_guard<std::mutex> lock(mutex());
  ++count();
}

BackgroundWork::~BackgroundWork() {
  {
    std::lock_guard<std::mutex> lock(mutex());
    --count();
  }
  cv().notify_all();
}

int background_work_in_flight() {
  std::lock_guard<std::mutex> lock(mutex());
  return count();
}

bool wait_for_background_work(int timeout_ms) {
  std::unique_lock<std::mutex> lock(mutex());
  return cv().wait_for(
      lock, std::chrono::milliseconds(timeout_ms < 0 ? 0 : timeout_ms),
      [] { return count() == 0; });
}

} // namespace rux::qt
