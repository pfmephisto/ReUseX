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

#include <CLI/CLI.hpp>
#include <crow.h>
#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cstdint>
#include <string>

namespace {

constexpr int kMaxVerbosity = 3;

// Bridge the ReUseX library logger into spdlog, mirroring the setup in
// apps/rux/src/rux.cpp so library log output is rendered the same way.
void setup_logging() {
  spdlog::init_thread_pool(8192, 1);
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  auto console_logger = std::make_shared<spdlog::async_logger>(
      "ruxd", console_sink, spdlog::thread_pool(),
      spdlog::async_overflow_policy::block);
  spdlog::set_default_logger(console_logger);

  spdlog::set_level(spdlog::level::warn); // Default level (raise with -v)
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
  reusex::core::set_log_level(reusex::core::LogLevel::warn);
}

} // namespace

int main(int argc, char **argv) {
  setup_logging();

  CLI::App cli{"ruxd: ReUseX HTTP service worker."};

  std::uint16_t port = 8080;
  // 0 lets Crow pick a sensible default (hardware concurrency).
  unsigned threads = 0;

  cli.add_option("-p,--port", port, "Port to listen on")
      ->envname("RUXD_PORT")
      ->capture_default_str();
  cli.add_option("-t,--threads", threads,
                 "Number of worker threads (0 = auto)")
      ->envname("RUXD_THREADS")
      ->capture_default_str();

  // Verbosity: -v, -vv, -vvv raise both spdlog and the ReUseX library logger
  // from the default warn level to info/debug/trace, mirroring rux.
  cli.add_flag(
         "-v,--verbose",
         [](int count) {
           const int safe_count = std::clamp(count, 0, kMaxVerbosity);
           const auto level = static_cast<spdlog::level::level_enum>(
               kMaxVerbosity - safe_count);
           spdlog::set_level(level);
           reusex::core::set_log_level(
               static_cast<reusex::core::LogLevel>(kMaxVerbosity - safe_count));
         },
         "Increase verbosity, use -vv & -vvv for more details.")
      ->multi_option_policy(CLI::MultiOptionPolicy::Sum)
      ->check(CLI::Range(0, 3));

  CLI11_PARSE(cli, argc, argv);

  // Prove the reusex library is linked and callable.
  reusex::core::info("ReUseX library linked, version {}",
                     reusex::core::VERSION);

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
