// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// ruxd: HTTP service worker for ReUseX.
//
// Two modes:
//
//  * `ruxd --local <file.rux | dir>` serves the web GUI — the bundled
//    frontend plus the REST + WebSocket API in docs/gui/openapi.yaml — for
//    every `.rux` it names, each one a case under /api/v1/cases/{cid}, with no
//    Postgres, Redis or S3 (src/local.cpp, ruxd_api_lib). This is what
//    `rux gui` used to be.
//  * Without --local, the multi-user service: for now the health / readyz /
//    meta routes and backend-client probes in src/handlers/, registered here
//    via the register_* functions declared in handlers.hpp. Users, sessions
//    and Postgres-backed cases are phase S3 of docs/superpowers/specs/
//    2026-10-08-ruxd-multiuser-and-qt-client-design.md.

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

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>
#include <utility>

namespace {

constexpr int kMaxVerbosity = 3;

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

void configure_cli(CLI::App &app, Invocation &inv) {
  // --- HTTP server ---
  app.add_option("-p,--port", inv.config.port, "Port to listen on")
      ->envname("RUXD_PORT")
      ->capture_default_str();
  app.add_option("-t,--threads", inv.config.threads,
                 "Number of worker threads (0 = auto)")
      ->envname("RUXD_THREADS")
      ->capture_default_str();

  // --- PostgreSQL ---
  app.add_option("--pg-url", inv.config.pg_url,
                 "PostgreSQL connection string "
                 "(postgresql://user:pass@host:5432/db)")
      ->envname("DATABASE_URL");
  app.add_option("--pg-pool-size", inv.config.pg_pool_size,
                 "PostgreSQL connection pool size (0 = one per worker thread)")
      ->envname("RUXD_PG_POOL_SIZE")
      ->capture_default_str();
  app.add_option("--pg-acquire-timeout-ms", inv.config.pg_acquire_timeout_ms,
                 "Milliseconds a request waits for a free PostgreSQL "
                 "connection before failing")
      ->envname("RUXD_PG_ACQUIRE_TIMEOUT_MS")
      ->capture_default_str();

  // --- Redis ---
  app.add_option("--redis-url", inv.config.redis_url,
                 "Redis URI (tcp://host:port)")
      ->envname("REDIS_URL")
      ->capture_default_str();

  // --- S3 / object storage ---
  app.add_option("--s3-endpoint", inv.config.s3_endpoint,
                 "S3 endpoint URL (empty = real AWS)")
      ->envname("AWS_ENDPOINT_URL");
  app.add_option("--s3-region", inv.config.s3_region, "S3 region")
      ->envname("AWS_REGION")
      ->capture_default_str();
  app.add_option("--s3-bucket", inv.config.s3_bucket, "S3 bucket name")
      ->envname("RUXD_S3_BUCKET");
  app.add_option("--s3-access-key", inv.config.s3_access_key,
                 "S3 access key id")
      ->envname("AWS_ACCESS_KEY_ID");
  app.add_option("--s3-secret-key", inv.config.s3_secret_key,
                 "S3 secret access key")
      ->envname("AWS_SECRET_ACCESS_KEY");
  app.add_flag("--s3-path-style,!--s3-virtual-style", inv.config.s3_path_style,
               "Use path-style S3 addressing (required by MinIO/Ceph)");

  // --- Auth ---
  app.add_option("--auth-token", inv.config.auth_token,
                 "Bearer token required for authenticated routes "
                 "(empty = auth disabled). With --local: the access token "
                 "every request must present; required beyond loopback")
      ->envname("RUXD_AUTH_TOKEN");

  // --- Local mode (the web GUI for one project; formerly `rux gui`) ---
  // Every other option in this group needs --local: server mode would
  // silently ignore it otherwise.
  auto &local = inv.local;
  local.server.open_browser = false;
  const std::string local_group = "Local mode";
  CLI::Option *local_opt =
      app.add_option(
             "--local", local.target,
             "Serve the web GUI for a .rux file, or for every .rux in a "
             "directory (each one a case), with no Postgres, Redis or S3")
          ->group(local_group);
  app.add_option("--data-dir", local.server.data_dir,
                 "Where created and uploaded cases are stored, one directory "
                 "per case (default: the --local directory; none for a lone "
                 "file, which makes the case list read-only)")
      ->group(local_group)
      ->needs(local_opt);
  app.add_option("--job-workers", local.server.job_workers,
                 "Pipeline jobs that may run at once across all cases (at "
                 "most one per case)")
      ->capture_default_str()
      ->check(CLI::Range(1, 64))
      ->group(local_group)
      ->needs(local_opt);
  app.add_option("--max-open-cases", local.server.max_open_cases,
                 "Most cases kept open at once; idle ones close first")
      ->capture_default_str()
      ->check(CLI::Range(1, 1024))
      ->group(local_group)
      ->needs(local_opt);
  app.add_option("--case-idle-minutes", local.case_idle_minutes,
                 "Close a case nobody has used for this many minutes")
      ->capture_default_str()
      ->check(CLI::Range(1, 24 * 60))
      ->group(local_group)
      ->needs(local_opt);
  app.add_option("--max-upload-mb", local.max_upload_mb,
                 "Largest .rux file accepted as an upload, in MiB")
      ->capture_default_str()
      ->check(CLI::Range(1, 1 << 24))
      ->group(local_group)
      ->needs(local_opt);
  app.add_option("--bind", local.server.bind_address,
                 "Interface to bind in local mode. Anything beyond loopback "
                 "requires --auth-token")
      ->capture_default_str()
      ->group(local_group)
      ->needs(local_opt);
  app.add_option("--allow-origin", local.server.allowed_origins,
                 "Additional browser origin allowed to call the API "
                 "(repeatable). Loopback is always allowed")
      ->group(local_group)
      ->needs(local_opt);
  app.add_option("--assets", local.server.asset_dir,
                 "Directory holding the frontend bundle (else $RUX_GUI_ASSETS, "
                 "then <prefix>/share/reusex/gui)")
      ->check(CLI::ExistingDirectory)
      ->group(local_group)
      ->needs(local_opt);
  app.add_flag("--open-browser", local.server.open_browser,
               "Open the system browser once listening")
      ->group(local_group)
      ->needs(local_opt);
  app.add_flag("--segment-cuda,!--no-segment-cuda", local.server.segment_cuda,
               "Use CUDA/TensorRT for the segment endpoints (default: on); "
               "--no-segment-cuda routes inference through ONNX on the CPU")
      ->group(local_group)
      ->needs(local_opt);
  app.add_option("--sam3-model", local.sam3_model_dir,
                 "Explicit SAM3 model directory (TRT engine dir or ONNX dir). "
                 "When omitted, a managed model is prepared on first use")
      ->group(local_group)
      ->needs(local_opt);
  app.add_option("--models-dir", local.models_dir,
                 "Base directory for managed models (default: "
                 "$REUSEX_MODELS_DIR or the XDG cache dir)")
      ->group(local_group)
      ->needs(local_opt);
  app.add_option("--sam3-manifest-url", local.sam3_manifest_url,
                 "URL of the SAM3 ONNX bundle release manifest (default: "
                 "built-in)")
      ->group(local_group)
      ->needs(local_opt);

  app.footer(R"footer(
Local mode (the web GUI, formerly `rux gui`):
  ruxd --local scan.rux                 one case
  ruxd --local ~/sager                  every .rux in the directory is a case;
                                        new and uploaded cases go there too
  ruxd --local ~/sager --job-workers 2  two cases may run a stage at once
  ruxd --local scan.rux --bind 0.0.0.0 --auth-token <token>
Then open http://127.0.0.1:8420/sager (or http://<host>:<port>/?token=<token>,
which sets a cookie and drops the token from the URL).
On loopback, local mode has no authentication: anything on this machine can
read and change the project and run pipeline stages. A --bind beyond loopback
is refused without --auth-token; with one, the page's own origin is allowed.
Add --allow-origin <origin> only for a frontend served from somewhere else.
)footer");

  // Verbosity: -v, -vv, -vvv raise both spdlog and the ReUseX library logger
  // from the default warn level to info/debug/trace, mirroring rux.
  app.add_flag(
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
}

void finish_invocation(const CLI::App &app, Invocation &inv) {
  if (!inv.is_local())
    return;
  // Local mode has its own default port (the one the Vite proxy and the docs
  // name); an explicit --port or RUXD_PORT still wins.
  inv.local.server.port = app.get_option("--port")->count() > 0
                              ? inv.config.port
                              : kLocalDefaultPort;
  inv.local.server.threads = inv.config.threads;
  inv.local.server.auth_token = inv.config.auth_token;
  inv.local.server.case_idle_timeout =
      std::chrono::minutes(inv.local.case_idle_minutes);
  inv.local.server.upload_limits.max_bytes =
      static_cast<std::uint64_t>(inv.local.max_upload_mb) << 20;
}

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
