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

#include <clients.hpp>
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

  ruxd::Config cfg;

  // --- HTTP server ---
  cli.add_option("-p,--port", cfg.port, "Port to listen on")
      ->envname("RUXD_PORT")
      ->capture_default_str();
  cli.add_option("-t,--threads", cfg.threads,
                 "Number of worker threads (0 = auto)")
      ->envname("RUXD_THREADS")
      ->capture_default_str();

  // --- PostgreSQL ---
  cli.add_option("--pg-url", cfg.pg_url,
                 "PostgreSQL connection string "
                 "(postgresql://user:pass@host:5432/db)")
      ->envname("DATABASE_URL");

  // --- Redis ---
  cli.add_option("--redis-url", cfg.redis_url, "Redis URI (tcp://host:port)")
      ->envname("REDIS_URL")
      ->capture_default_str();

  // --- S3 / object storage ---
  cli.add_option("--s3-endpoint", cfg.s3_endpoint,
                 "S3 endpoint URL (empty = real AWS)")
      ->envname("AWS_ENDPOINT_URL");
  cli.add_option("--s3-region", cfg.s3_region, "S3 region")
      ->envname("AWS_REGION")
      ->capture_default_str();
  cli.add_option("--s3-bucket", cfg.s3_bucket, "S3 bucket name")
      ->envname("RUXD_S3_BUCKET");
  cli.add_option("--s3-access-key", cfg.s3_access_key, "S3 access key id")
      ->envname("AWS_ACCESS_KEY_ID");
  cli.add_option("--s3-secret-key", cfg.s3_secret_key, "S3 secret access key")
      ->envname("AWS_SECRET_ACCESS_KEY");
  cli.add_flag("--s3-path-style,!--s3-virtual-style", cfg.s3_path_style,
               "Use path-style S3 addressing (required by MinIO/Ceph)");

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

  // Report which backends are configured — never log credentials.
  reusex::core::info("postgres: {}",
                     cfg.pg_url.empty() ? "not configured" : "configured");
  reusex::core::info("redis: {}", cfg.redis_url);
  reusex::core::info("s3: endpoint={} bucket={} path_style={}",
                     cfg.s3_endpoint.empty() ? "(aws default)" : cfg.s3_endpoint,
                     cfg.s3_bucket.empty() ? "(unset)" : cfg.s3_bucket,
                     cfg.s3_path_style);

  // Backend clients connect lazily; constructing them is cheap (the AWS SDK
  // guard aside). Must outlive app.run().
  ruxd::Clients clients(cfg);

  crow::SimpleApp app;

  // Routes register through the endpoint registry, which backs /endpoints and
  // /openapi.json.
  ruxd::EndpointRegistry registry;
  ruxd::register_health_routes(app, registry, clients);
  ruxd::register_segment_routes(app, registry);
  ruxd::register_meta_routes(app, registry);
  ruxd::register_not_found_handler(app);

  // Fail fast if any registered route is missing a handler, and log the route
  // table so the configured surface is reviewable at startup (-v).
  app.validate();
  for (const auto &e : registry.endpoints()) {
    reusex::core::info("route: {:<5} {}{}", e.method, e.path,
                       e.requires_auth ? "  [auth]" : "");
  }

  app.port(cfg.port).multithreaded();
  if (cfg.threads > 0) {
    app.concurrency(cfg.threads);
  }

  reusex::core::info("ruxd listening on port {} (threads: {})", cfg.port,
                     cfg.threads > 0 ? std::to_string(cfg.threads)
                                     : std::string("auto"));

  // Crow installs its own SIGINT/SIGTERM handler and stops gracefully.
  app.run();

  reusex::core::info("ruxd shutting down");
  return 0;
}
