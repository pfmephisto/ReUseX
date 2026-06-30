// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// ruxd: HTTP service worker for ReUseX.
//
// A long-running server that will eventually execute processing operations
// (e.g. point cloud segmentation) on demand from incoming HTTP requests. For
// now the routes only log and return stub responses — the goal of this first
// iteration is to prove that the server links against the `reusex` library and
// stands up correctly. Real work (calling reusex::geometry::segment_planes,
// etc.) will be wired into the handlers later.
//
// Routes are organized per feature under src/handlers/ and registered here via
// the register_* functions declared in handlers.hpp.

#include <handlers.hpp>

#include <reusex/core/logging.hpp>
#include <reusex/core/version.hpp>

#include <crow.h>
#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <cstdint>
#include <cstdlib>
#include <string>
#include <string_view>

namespace {

// Bridge the ReUseX library logger into spdlog, mirroring the setup in
// apps/rux/src/rux.cpp so library log output is rendered the same way.
void setup_logging() {
  spdlog::init_thread_pool(8192, 1);
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  auto console_logger = std::make_shared<spdlog::async_logger>(
      "ruxd", console_sink, spdlog::thread_pool(),
      spdlog::async_overflow_policy::block);
  spdlog::set_default_logger(console_logger);

  spdlog::set_level(spdlog::level::info);
  spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%^%=7l%$] %v");

  reusex::core::set_log_handler(
      [](reusex::core::LogLevel level, std::string_view message) {
        switch (level) {
        case reusex::core::LogLevel::trace:
          spdlog::trace("{}", message);
          break;
        case reusex::core::LogLevel::debug:
          spdlog::debug("{}", message);
          break;
        case reusex::core::LogLevel::info:
          spdlog::info("{}", message);
          break;
        case reusex::core::LogLevel::warn:
          spdlog::warn("{}", message);
          break;
        case reusex::core::LogLevel::error:
          spdlog::error("{}", message);
          break;
        case reusex::core::LogLevel::critical:
          spdlog::critical("{}", message);
          break;
        case reusex::core::LogLevel::off:
          break;
        }
      });
  reusex::core::set_log_level(reusex::core::LogLevel::info);
}

// Read an unsigned setting from --<flag> <n> or $<env>, falling back to
// fallback when neither is present.
unsigned resolve_uint(int argc, char **argv, std::string_view flag,
                      const char *env, unsigned fallback) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string_view(argv[i]) == flag) {
      return static_cast<unsigned>(std::stoul(argv[i + 1]));
    }
  }
  if (const char *value = std::getenv(env)) {
    return static_cast<unsigned>(std::stoul(value));
  }
  return fallback;
}

} // namespace

int main(int argc, char **argv) {
  setup_logging();

  // Prove the reusex library is linked and callable.
  reusex::core::info("ReUseX library linked, version {}",
                     reusex::core::VERSION);

  const auto port =
      static_cast<std::uint16_t>(resolve_uint(argc, argv, "--port", "RUXD_PORT",
                                              8080));
  // 0 lets Crow pick a sensible default (hardware concurrency).
  const unsigned threads =
      resolve_uint(argc, argv, "--threads", "RUXD_THREADS", 0);

  crow::SimpleApp app;

  ruxd::register_health_routes(app);
  ruxd::register_segment_routes(app);
  ruxd::register_not_found_handler(app);

  app.port(port).multithreaded();
  if (threads > 0) {
    app.concurrency(threads);
  }

  reusex::core::info("ruxd listening on port {} (threads: {})", port,
                     threads > 0 ? std::to_string(threads)
                                 : std::string("auto"));

  // Crow installs its own SIGINT/SIGTERM handler and stops gracefully.
  app.run();

  reusex::core::info("ruxd shutting down");
  return 0;
}
