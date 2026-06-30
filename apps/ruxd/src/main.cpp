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

#include <reusex/core/logging.hpp>
#include <reusex/core/version.hpp>

#include <crow.h>
#include <nlohmann/json.hpp>
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

// Resolve the listen port from --port <n>, $RUXD_PORT, or the default 8080.
std::uint16_t resolve_port(int argc, char **argv) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string_view(argv[i]) == "--port") {
      return static_cast<std::uint16_t>(std::stoi(argv[i + 1]));
    }
  }
  if (const char *env = std::getenv("RUXD_PORT")) {
    return static_cast<std::uint16_t>(std::stoi(env));
  }
  return 8080;
}

} // namespace

int main(int argc, char **argv) {
  setup_logging();

  // Prove the reusex library is linked and callable.
  reusex::core::info("ReUseX library linked, version {}",
                     reusex::core::VERSION);

  const std::uint16_t port = resolve_port(argc, argv);

  crow::SimpleApp app;

  // Liveness probe.
  CROW_ROUTE(app, "/health")
  ([] {
    nlohmann::json body{{"status", "ok"}};
    crow::response res(crow::status::OK, body.dump());
    res.set_header("Content-Type", "application/json");
    return res;
  });

  // Point cloud plane segmentation. Not yet implemented — logs the request and
  // returns a stub response. The future implementation will deserialize the
  // request payload and call reusex::geometry::segment_planes(...).
  CROW_ROUTE(app, "/segment/planes")
      .methods(crow::HTTPMethod::POST)([](const crow::request &req) {
        reusex::core::info(
            "POST /segment/planes — received segmentation request "
            "(not yet implemented), body size = {} bytes",
            req.body.size());
        nlohmann::json body{
            {"status", "not_implemented"},
            {"message", "segmentation not yet wired up"}};
        crow::response res(crow::status::NOT_IMPLEMENTED, body.dump());
        res.set_header("Content-Type", "application/json");
        return res;
      });

  reusex::core::info("ruxd listening on port {}", port);

  // Crow installs its own SIGINT/SIGTERM handler and stops gracefully.
  app.port(port).multithreaded().run();

  reusex::core::info("ruxd shutting down");
  return 0;
}
