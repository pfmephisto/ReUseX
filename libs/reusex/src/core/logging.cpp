// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ReUseX/core/logging.hpp>

#include <array>
#include <atomic>
#include <cstdio>
#include <memory>

namespace ReUseX::core {
namespace {
std::atomic<LogLevel> g_level{LogLevel::info};

void default_log_handler(LogLevel level, std::string_view message) {
  constexpr std::array<std::string_view, 7> names = {"trace", "debug", "info",
                                                      "warn",  "error", "critical",
                                                      "off"};
  const auto index = static_cast<std::size_t>(level);
  if (index >= names.size()) {
    return;
  }
  std::fprintf(stderr, "[reusex] [%.*s] %.*s\n",
               static_cast<int>(names[index].size()), names[index].data(),
               static_cast<int>(message.size()), message.data());
}

// Note: std::atomic<std::shared_ptr<...>> may use non-lock-free primitives on
// some platforms, but it keeps dispatch independent from a single global mutex
// and avoids serializing downstream handler execution in ReUseX.
std::atomic<std::shared_ptr<LogHandler>> g_handler{
    std::make_shared<LogHandler>(default_log_handler)};
} // namespace

void set_log_handler(LogHandler handler) {
  auto next_handler = std::make_shared<LogHandler>(
      handler ? std::move(handler) : LogHandler{default_log_handler});
  g_handler.store(std::move(next_handler), std::memory_order_release);
}

void reset_log_handler() { set_log_handler(default_log_handler); }

void set_log_level(LogLevel level) { g_level.store(level, std::memory_order_relaxed); }

auto get_log_level() -> LogLevel { return g_level.load(std::memory_order_relaxed); }

auto should_log(LogLevel level) -> bool {
  return level >= get_log_level() && level != LogLevel::off;
}

void log_message(LogLevel level, std::string_view message) {
  const auto handler_ptr = g_handler.load(std::memory_order_acquire);
  if (!handler_ptr || !(*handler_ptr)) {
    return;
  }
  (*handler_ptr)(level, message);
}

} // namespace ReUseX::core
