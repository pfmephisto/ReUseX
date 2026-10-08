// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// ruxd::run(): parse the command line (src/cli/cli.cpp, ruxd_cli_lib), then
// run local mode (src/local.cpp) or the service.

#include <cli.hpp>

#include <clients.hpp>
#include <handlers.hpp>
#include <local.hpp>

#include <reusex/core/logging.hpp>
#include <reusex/core/version.hpp>

#include <crow.h>
#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <string>
#include <utility>

namespace {

// Bridge the ReUseX library logger into spdlog, mirroring the setup in
// apps/rux/src/rux.cpp so library log output is rendered the same way.
void setup_logging() {
  spdlog::init_thread_pool(8192, 1);
  auto console_sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
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

namespace ruxd {

int run(int argc, char **argv) {
  setup_logging();

  CLI::App cli{"ruxd: ReUseX HTTP service worker."};
  Invocation inv;
  configure_cli(cli, inv);
  CLI11_PARSE(cli, argc, argv);
  finish_invocation(cli, inv);

  if (inv.is_local()) {
    for (const char *name : {"--pg-url", "--s3-bucket", "--s3-endpoint"})
      if (cli.get_option(name)->count() > 0)
        spdlog::warn("{} is ignored in local mode (no Postgres, Redis or S3)",
                     name);
    return run_local(std::move(inv.local));
  }

  const Config &cfg = inv.config;

  // Prove the reusex library is linked and callable.
  reusex::core::info("ReUseX library linked, version {}",
                     reusex::core::VERSION);

  // Report which backends are configured — never log credentials.
  reusex::core::info("postgres: {}",
                     cfg.pg_url.empty() ? "not configured" : "configured");
  reusex::core::info("redis: {}", cfg.redis_url);
  reusex::core::info(
      "s3: endpoint={} bucket={} path_style={}",
      cfg.s3_endpoint.empty() ? "(aws default)" : cfg.s3_endpoint,
      cfg.s3_bucket.empty() ? "(unset)" : cfg.s3_bucket, cfg.s3_path_style);

  // Backend clients connect lazily; constructing them is cheap (the AWS SDK
  // guard aside). Must outlive app.run().
  Clients clients(cfg);

  App app;

  // Routes register through the endpoint registry, which backs /endpoints and
  // /openapi.json.
  EndpointRegistry registry;
  register_health_routes(app, registry, clients);
  register_segment_routes(app, registry);
  register_meta_routes(app, registry);

  register_not_found_handler(app);

  // Wire the auth middleware to the populated registry. With no token set, auth
  // is disabled — warn if any route nonetheless declares it requires auth.
  app.get_middleware<BearerAuthMiddleware>().configure(&registry,
                                                       cfg.auth_token);
  if (cfg.auth_token.empty()) {
    reusex::core::warn(
        "auth: DISABLED (no --auth-token / RUXD_AUTH_TOKEN set)");
  } else {
    reusex::core::info("auth: enabled (Bearer)");
  }

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

} // namespace ruxd
