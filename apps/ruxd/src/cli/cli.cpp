// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// ruxd's option set and post-parse defaults (ruxd_cli_lib: light, so
// tests/unit/ruxd_cli runs in the light test binary). run() is in src/run.cpp.
//
// ruxd: HTTP service worker for ReUseX.
//
// Two modes:
//
//  * `ruxd --local <file.rux | dir>` serves the web GUI — the bundled
//    frontend plus the REST + WebSocket API in docs/gui/openapi.yaml — for
//    every `.rux` it names, each one a case under /api/v1/cases/{cid}, with no
//    Postgres, Redis or S3 (src/local.cpp, ruxd_api_lib). This is what
//    `rux gui` used to be.
//  * Without --local, the multi-user server: the same frontend and API, with
//    users, sessions, API tokens and case membership in Postgres
//    (--pg-url), and case files in --data-dir (src/server.cpp, phase S3 of
//    docs/superpowers/specs/2026-10-08-ruxd-multiuser-and-qt-client-design.md).
//  * `ruxd admin …` manages that server's users from a shell on it.

#include <cli.hpp>

#include <api/access.hpp>

#include <reusex/core/logging.hpp>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {
constexpr int kMaxVerbosity = 3;
} // namespace

namespace ruxd {

namespace {

/// `ruxd admin <command>`: user and case administration (pg/admin.hpp).
/// Postgres comes from --pg-url / DATABASE_URL, given before or after.
void configure_admin_cli(CLI::App &app, Invocation &inv) {
  using Kind = pg::AdminCommand::Kind;
  auto &cmd = inv.admin;
  CLI::App *admin = app.add_subcommand(
      "admin", "Manage the server's users and cases (needs --pg-url). "
               "Passwords are prompted for, or read from stdin; never argv");
  admin->fallthrough();
  admin->require_subcommand(1);
  auto sub = [&](const char *name, const char *help, Kind kind) {
    CLI::App *s = admin->add_subcommand(name, help);
    s->fallthrough();
    s->callback([&cmd, kind] { cmd.kind = kind; });
    return s;
  };

  auto *create = sub("create-user", "Create a user (the first: with --admin)",
                     Kind::create_user);
  create->add_option("--email", cmd.email, "Their email (the login)")
      ->required();
  create->add_option("--name", cmd.display_name, "Display name");
  create->add_flag("--admin", cmd.is_admin,
                   "An administrator: every case, and user management");

  auto *password =
      sub("set-password", "Set a user's password (ends their sessions)",
          Kind::set_password);
  password->add_option("--email", cmd.email, "Whose")->required();

  sub("list-users", "List every user", Kind::list_users);

  auto *disable = sub("disable-user",
                      "Disable a user (ends their sessions); --enable undoes "
                      "it",
                      Kind::disable_user);
  disable->add_option("--email", cmd.email, "Whose")->required();
  disable->add_flag("--enable", cmd.enable, "Re-enable instead");

  auto *token = sub("create-token",
                    "Create an API token (Authorization: Bearer) for scripts; "
                    "printed once on stdout",
                    Kind::create_token);
  token->add_option("--email", cmd.email, "The user it acts as")->required();
  token->add_option("--name", cmd.token_name, "What it is for")->required();
  token->add_option("--case", cmd.case_id, "Limit it to this case id");
  token
      ->add_option("--expires-days", cmd.expires_days,
                   "Days until it stops working (0 = never; default 90)")
      ->check(CLI::Range(0, 3650));

  auto *tokens =
      sub("list-tokens", "List API tokens (never the tokens themselves)",
          Kind::list_tokens);
  tokens->add_option("--email", cmd.email, "Only this user's");

  auto *revoke =
      sub("revoke-token", "Revoke an API token by id", Kind::revoke_token);
  revoke->add_option("--id", cmd.token_id, "Its id (from list-tokens)")
      ->required();

  auto *reg = sub("register-case",
                  "Serve an existing .rux file as a case, where it is "
                  "(needs --data-dir)",
                  Kind::register_case);
  reg->add_option("--path", cmd.path, "The project file")
      ->required()
      ->check(CLI::ExistingFile);
  reg->add_option("--name", cmd.case_name,
                  "Case name (default: the file name)");
  reg->add_option("--owner", cmd.email, "Email of the user who owns it");
}

} // namespace

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
  app.add_option("--pg-url-file", inv.pg_url_file,
                 "Read the PostgreSQL connection string from this file "
                 "(keeps the password out of the process list)")
      ->envname("DATABASE_URL_FILE")
      ->check(CLI::ExistingFile);
  app.add_option("--pg-pool-size", inv.config.pg_pool_size,
                 "PostgreSQL connection pool size (0 = one per worker thread)")
      ->envname("RUXD_PG_POOL_SIZE")
      ->capture_default_str();
  app.add_option("--pg-acquire-timeout-ms", inv.config.pg_acquire_timeout_ms,
                 "Milliseconds a request waits for a free PostgreSQL "
                 "connection before failing")
      ->envname("RUXD_PG_ACQUIRE_TIMEOUT_MS")
      ->capture_default_str();

  // --- Redis and S3: reserved ---
  // Kept for the S3-snapshot phase (spec, "Deferred"); nothing reads them yet.
  // The client code in src/clients/ stays, with its tests.
  const std::string reserved = "Reserved (unused yet)";
  app.add_option("--redis-url", inv.config.redis_url,
                 "Reserved, unused: Redis URI (tcp://host:port)")
      ->envname("REDIS_URL")
      ->group(reserved);
  app.add_option("--s3-endpoint", inv.config.s3_endpoint,
                 "Reserved, unused: S3 endpoint URL (empty = real AWS)")
      ->envname("AWS_ENDPOINT_URL")
      ->group(reserved);
  app.add_option("--s3-region", inv.config.s3_region,
                 "Reserved, unused: S3 region")
      ->envname("AWS_REGION")
      ->capture_default_str()
      ->group(reserved);
  app.add_option("--s3-bucket", inv.config.s3_bucket,
                 "Reserved, unused: S3 bucket name")
      ->envname("RUXD_S3_BUCKET")
      ->group(reserved);
  app.add_option("--s3-access-key", inv.config.s3_access_key,
                 "Reserved, unused: S3 access key id")
      ->envname("AWS_ACCESS_KEY_ID")
      ->group(reserved);
  app.add_option("--s3-secret-key", inv.config.s3_secret_key,
                 "Reserved, unused: S3 secret access key")
      ->envname("AWS_SECRET_ACCESS_KEY")
      ->group(reserved);
  app.add_flag("--s3-path-style,!--s3-virtual-style", inv.config.s3_path_style,
               "Reserved, unused: path-style S3 addressing (MinIO/Ceph)")
      ->group(reserved);

  // --- Auth ---
  app.add_option("--auth-token", inv.config.auth_token,
                 "Server mode: a superuser Bearer token that may do "
                 "everything (bootstrap, operations; empty = none). With "
                 "--local: the access token every request must present; "
                 "required beyond loopback")
      ->envname("RUXD_AUTH_TOKEN");
  app.add_option("--auth-token-file", inv.auth_token_file,
                 "Read --auth-token from this file (keeps it out of the "
                 "process list)")
      ->envname("RUXD_AUTH_TOKEN_FILE")
      ->check(CLI::ExistingFile);

  // --- The web GUI: both modes (--local and the multi-user server) ---
  auto &local = inv.local;
  local.server.open_browser = false;
  const std::string local_group = "Web GUI";
  CLI::Option *local_opt =
      app.add_option(
             "--local", local.target,
             "Serve the web GUI for a .rux file, or for every .rux in a "
             "directory (each one a case), for one person: no login, no "
             "Postgres, Redis or S3")
          ->group(local_group);
  app.add_option("--data-dir", local.server.data_dir,
                 "Where created and uploaded cases are stored, one directory "
                 "per case. Required in server mode; with --local it "
                 "defaults to the --local directory (none for a lone file, "
                 "which makes the case list read-only)")
      ->envname("RUXD_DATA_DIR")
      ->group(local_group);
  app.add_option("--trusted-proxy", local.server.trusted_proxies,
                 "A reverse proxy (CIDR or address, repeatable) whose "
                 "X-Forwarded-For names the client, for the login back-off. "
                 "From anyone else the header is ignored")
      ->check([](const std::string &value) {
        return api::is_valid_cidr(value)
                   ? std::string()
                   : "'" + value + "' is not an IP address or CIDR";
      })
      ->group(local_group);
  app.add_option("--audit-retention-days", inv.audit_retention_days,
                 "Server mode: days the audit log is kept (0 = for ever)")
      ->capture_default_str()
      ->check(CLI::Range(0, 36500))
      ->group(local_group);
  app.add_option("--cookie-secure", inv.cookie_secure,
                 "Server mode: mark the session cookie Secure: auto (unless "
                 "bound to loopback, or behind a proxy sending "
                 "X-Forwarded-Proto: https), always, never")
      ->check(CLI::IsMember({"auto", "always", "never"}))
      ->capture_default_str()
      ->group(local_group);
  app.add_option("--job-workers", local.server.job_workers,
                 "Pipeline jobs that may run at once across all cases (at "
                 "most one per case)")
      ->capture_default_str()
      ->check(CLI::Range(1, 64))
      ->group(local_group);
  app.add_option("--max-open-cases", local.server.max_open_cases,
                 "Most cases kept open at once; idle ones close first")
      ->capture_default_str()
      ->check(CLI::Range(1, 1024))
      ->group(local_group);
  app.add_option("--case-idle-minutes", local.case_idle_minutes,
                 "Close a case nobody has used for this many minutes")
      ->capture_default_str()
      ->check(CLI::Range(1, 24 * 60))
      ->group(local_group);
  app.add_option("--max-upload-mb", local.max_upload_mb,
                 "Largest .rux file accepted as an upload, in MiB")
      ->capture_default_str()
      ->check(CLI::Range(1, 1 << 24))
      ->group(local_group);
  app.add_option("--bind", local.server.bind_address,
                 "Interface to bind. In local mode anything beyond loopback "
                 "requires --auth-token; server mode always authenticates")
      ->envname("RUXD_BIND")
      ->capture_default_str()
      ->group(local_group);
  app.add_option("--allow-origin", local.server.allowed_origins,
                 "Additional browser origin allowed to call the API "
                 "(repeatable). Loopback is always allowed")
      ->group(local_group);
  app.add_option("--assets", local.server.asset_dir,
                 "Directory holding the frontend bundle (else $RUX_GUI_ASSETS, "
                 "then <prefix>/share/reusex/gui)")
      ->check(CLI::ExistingDirectory)
      ->group(local_group);
  app.add_flag("--open-browser", local.server.open_browser,
               "Open the system browser once listening")
      ->group(local_group)
      ->needs(local_opt);
  app.add_flag("--segment-cuda,!--no-segment-cuda", local.server.segment_cuda,
               "Use CUDA/TensorRT for the segment endpoints (default: on); "
               "--no-segment-cuda routes inference through ONNX on the CPU")
      ->group(local_group);
  app.add_option("--sam3-model", local.sam3_model_dir,
                 "Explicit SAM3 model directory (TRT engine dir or ONNX dir). "
                 "When omitted, a managed model is prepared on first use")
      ->group(local_group);
  app.add_option("--models-dir", local.models_dir,
                 "Base directory for managed models (default: "
                 "$REUSEX_MODELS_DIR or the XDG cache dir)")
      ->group(local_group);
  app.add_option("--sam3-manifest-url", local.sam3_manifest_url,
                 "URL of the SAM3 ONNX bundle release manifest (default: "
                 "built-in)")
      ->group(local_group);

  configure_admin_cli(app, inv);

  app.footer(R"footer(
Server mode (users, sessions and cases in Postgres; files in --data-dir):
  ruxd admin create-user --email anna@example.dk --name Anna --admin
  ruxd --pg-url postgresql://ruxd@db/ruxd --data-dir /srv/ruxd --bind 0.0.0.0
Put TLS in front (a reverse proxy) and pass its public origin with
--allow-origin https://ruxd.example.dk; see docs/gui/README.md.

Local mode (the web GUI for one person, formerly `rux gui`):
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
  // Local mode has its own default port (the one the Vite proxy and the docs
  // name); an explicit --port or RUXD_PORT still wins. Server mode keeps
  // Config's.
  inv.local.server.port =
      !inv.is_local() || app.get_option("--port")->count() > 0
          ? inv.config.port
          : kLocalDefaultPort;
  inv.local.server.threads = inv.config.threads;
  // Local mode's access token; in server mode the same flag is the
  // superuser token, which AuthOptions carries instead.
  inv.local.server.auth_token = inv.is_local() ? inv.config.auth_token : "";
  inv.local.server.cookie_secure =
      inv.cookie_secure == "always"  ? api::CookieSecure::always
      : inv.cookie_secure == "never" ? api::CookieSecure::never
                                     : api::CookieSecure::automatic;
  inv.local.server.case_idle_timeout =
      std::chrono::minutes(inv.local.case_idle_minutes);
  inv.local.server.upload_limits.max_bytes =
      static_cast<std::uint64_t>(inv.local.max_upload_mb) << 20;
  inv.local.server.audit_retention =
      std::chrono::hours(24) * inv.audit_retention_days;
}

std::string read_secret_file(const std::filesystem::path &file) {
  std::ifstream in(file);
  if (!in)
    throw std::runtime_error("cannot read " + file.string());
  std::string text((std::istreambuf_iterator<char>(in)),
                   std::istreambuf_iterator<char>());
  const auto last = text.find_last_not_of(" \t\r\n");
  text.erase(last == std::string::npos ? 0 : last + 1);
  const auto first = text.find_first_not_of(" \t\r\n");
  return first == std::string::npos ? std::string() : text.substr(first);
}

void load_secret_files(Invocation &inv) {
  if (!inv.auth_token_file.empty())
    inv.config.auth_token = read_secret_file(inv.auth_token_file);
  if (!inv.pg_url_file.empty())
    inv.config.pg_url = read_secret_file(inv.pg_url_file);
  if (inv.is_local())
    inv.local.server.auth_token = inv.config.auth_token;
}

std::vector<std::string> secrets_on_argv(int argc, char **argv) {
  std::vector<std::string> out;
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);
    for (const std::string_view name : {"--auth-token", "--pg-url"})
      if (arg == name || (arg.substr(0, name.size()) == name &&
                          arg.size() > name.size() && arg[name.size()] == '='))
        out.emplace_back(name);
  }
  return out;
}

} // namespace ruxd
