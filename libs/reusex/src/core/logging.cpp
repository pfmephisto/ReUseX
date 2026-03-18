// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/logging.hpp"

#include <array>
#include <atomic>
#include <cstdio>
#include <memory>

namespace ReUseX::core {
namespace {
std::atomic<LogLevel> g_level{LogLevel::info};

// Default handler is a no-op - library produces no output unless
// a custom handler is registered (e.g., by rux application)
void default_log_handler(LogLevel level, std::string_view message) {
  // Do nothing - silent by default
  (void)level;
  (void)message;
}

// Static storage for default handler (ensures valid lifetime)
LogHandler g_default_handler = default_log_handler;

// Static storage for custom handler (heap-allocated, managed lifetime)
std::unique_ptr<LogHandler> g_custom_handler;

// Raw pointer atomic: always lock-free on 64-bit platforms
// Caller must ensure handler outlives all log calls (same pattern as
// processing_observer)
std::atomic<LogHandler *> g_handler_ptr{&g_default_handler};

// Version counter for thread-local cache invalidation
std::atomic<uint32_t> g_handler_version{0};

// Thread-local cache structure to eliminate repeated atomic loads
struct LoggerCache {
  LogHandler *handler;
  LogLevel level;
  uint32_t version;
};

thread_local LoggerCache g_tls_cache{&g_default_handler, LogLevel::info, 0};

// Fast-path caching helper - zero atomic operations on cache hit
inline auto get_cached_handler() -> LogHandler * {
  const uint32_t current_version =
      g_handler_version.load(std::memory_order_relaxed);

  if (g_tls_cache.version == current_version) [[likely]] {
    // Cache hit - zero atomic operations!
    return g_tls_cache.handler;
  }

  // Cache miss - reload (happens once per thread after handler change)
  g_tls_cache.handler = g_handler_ptr.load(std::memory_order_acquire);
  g_tls_cache.level = g_level.load(std::memory_order_relaxed);
  g_tls_cache.version = current_version;
  return g_tls_cache.handler;
}
} // namespace

void set_log_handler(LogHandler handler) {
  if (!handler) {
    // Reset to default
    g_custom_handler.reset();
    g_handler_ptr.store(&g_default_handler, std::memory_order_release);
  } else {
    // Allocate on heap with static storage (managed by unique_ptr)
    // Caller must ensure handler outlives all log calls (same pattern as
    // processing_observer)
    g_custom_handler = std::make_unique<LogHandler>(std::move(handler));
    g_handler_ptr.store(g_custom_handler.get(), std::memory_order_release);
  }

  // Invalidate all thread-local caches
  g_handler_version.fetch_add(1, std::memory_order_release);
}

void reset_log_handler() { set_log_handler(default_log_handler); }

void set_log_level(LogLevel level) {
  g_level.store(level, std::memory_order_relaxed);
}

auto get_log_level() -> LogLevel {
  return g_level.load(std::memory_order_relaxed);
}

auto should_log(LogLevel level) -> bool {
  // Fast path: only check level, no atomic operations
  // Cache is lazily refreshed in log_message() when actually logging
  const uint32_t current_version =
      g_handler_version.load(std::memory_order_relaxed);

  if (g_tls_cache.version != current_version) [[unlikely]] {
    // Cache stale, refresh
    g_tls_cache.handler = g_handler_ptr.load(std::memory_order_acquire);
    g_tls_cache.level = g_level.load(std::memory_order_relaxed);
    g_tls_cache.version = current_version;
  }

  return level >= g_tls_cache.level && level != LogLevel::off;
}

void log_message(LogLevel level, std::string_view message) {
  // Handler is already cached from should_log() check
  // No need to call get_cached_handler() again
  LogHandler *handler = g_tls_cache.handler;
  if (handler) [[likely]] {
    (*handler)(level, message);
  }
}

} // namespace ReUseX::core
