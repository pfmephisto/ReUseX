// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ReUseX/core/logging.hpp>

#include <array>
#include <cstdio>
#include <mutex>

namespace ReUseX::core {
namespace {
std::mutex g_logger_mutex;
LogLevel g_level = LogLevel::info;

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

LogHandler g_handler = default_log_handler;
} // namespace

void set_log_handler(LogHandler handler) {
  std::lock_guard<std::mutex> lock(g_logger_mutex);
  g_handler = handler ? std::move(handler) : LogHandler{default_log_handler};
}

void reset_log_handler() { set_log_handler(default_log_handler); }

void set_log_level(LogLevel level) {
  std::lock_guard<std::mutex> lock(g_logger_mutex);
  g_level = level;
}

auto get_log_level() -> LogLevel {
  std::lock_guard<std::mutex> lock(g_logger_mutex);
  return g_level;
}

auto should_log(LogLevel level) -> bool {
  return level >= get_log_level() && level != LogLevel::off;
}

void log_message(LogLevel level, std::string_view message) {
  LogHandler handler;
  {
  std::lock_guard<std::mutex> lock(g_logger_mutex);
  handler = g_handler;
  }
  if (!handler) {
    return;
  }
  handler(level, message);
}

} // namespace ReUseX::core
