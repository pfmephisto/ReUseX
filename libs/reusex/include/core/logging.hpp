// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
#pragma once
#include <fmt/format.h>
#include <fmt/std.h>

#include <chrono>
#include <functional>
#include <string>
#include <string_view>
#include <utility>

namespace reusex::core {

enum class LogLevel {
  trace = 0,
  debug = 1,
  info = 2,
  warn = 3,
  error = 4,
  critical = 5,
  off = 6,
};

using LogHandler =
    std::function<void(LogLevel level, std::string_view message)>;

void set_log_handler(LogHandler handler);
void reset_log_handler();
void set_log_level(LogLevel level);
auto get_log_level() -> LogLevel;
auto should_log(LogLevel level) -> bool;
void log_message(LogLevel level, std::string_view message);

class stopwatch {
    public:
  // Lightweight replacement for spdlog::stopwatch used by existing timing
  // logs. elapsed() reports seconds as a double.
  stopwatch() = default;

  void reset() { start_ = clock::now(); }

  [[nodiscard]] auto elapsed() const -> double {
    return std::chrono::duration<double>(clock::now() - start_).count();
  }

    private:
  using clock = std::chrono::steady_clock;
  std::chrono::time_point<clock> start_ = clock::now();
};

template <typename... Args>
inline void log(LogLevel level, fmt::format_string<Args...> format,
                Args &&...args) {
  // Early return avoids formatting cost when the message is filtered out.
  if (!should_log(level)) [[unlikely]] {
    return;
  }
  log_message(level, fmt::format(format, std::forward<Args>(args)...));
}

inline void log(LogLevel level, std::string_view message) {
  if (!should_log(level)) [[unlikely]] {
    return;
  }
  log_message(level, message);
}

template <typename... Args>
inline void trace(fmt::format_string<Args...> format, Args &&...args) {
  log(LogLevel::trace, format, std::forward<Args>(args)...);
}

inline void trace(std::string_view message) { log(LogLevel::trace, message); }

template <typename... Args>
inline void debug(fmt::format_string<Args...> format, Args &&...args) {
  log(LogLevel::debug, format, std::forward<Args>(args)...);
}

inline void debug(std::string_view message) { log(LogLevel::debug, message); }

template <typename... Args>
inline void info(fmt::format_string<Args...> format, Args &&...args) {
  log(LogLevel::info, format, std::forward<Args>(args)...);
}

inline void info(std::string_view message) { log(LogLevel::info, message); }

template <typename... Args>
inline void warn(fmt::format_string<Args...> format, Args &&...args) {
  log(LogLevel::warn, format, std::forward<Args>(args)...);
}

inline void warn(std::string_view message) { log(LogLevel::warn, message); }

template <typename... Args>
inline void error(fmt::format_string<Args...> format, Args &&...args) {
  log(LogLevel::error, format, std::forward<Args>(args)...);
}

inline void error(std::string_view message) { log(LogLevel::error, message); }

template <typename... Args>
inline void critical(fmt::format_string<Args...> format, Args &&...args) {
  log(LogLevel::critical, format, std::forward<Args>(args)...);
}

inline void critical(std::string_view message) {
  log(LogLevel::critical, message);
}

} // namespace reusex::core

// Promote logging functions to ReUseX namespace for convenience
// Only promotes logging functions, avoiding conflicts with external libraries
namespace reusex {
using core::trace;
using core::debug;
using core::info;
using core::warn;
using core::error;
using core::critical;
using core::log;
using core::LogLevel;
using core::stopwatch;
} // namespace reusex

namespace fmt {
template <> struct formatter<reusex::core::stopwatch> : formatter<double> {
  template <typename FormatContext>
  auto format(const reusex::core::stopwatch &sw, FormatContext &ctx) const {
    return formatter<double>::format(sw.elapsed(), ctx);
  }
};
} // namespace fmt
