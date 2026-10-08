// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// ruxd's multi-user server mode and `ruxd admin` (spec 2026-10-08, phase
// S3): Postgres (migrated on start) for users, sessions, tokens, cases,
// membership, jobs and the audit log; case files in --data-dir; the same
// frontend and API as `ruxd --local`, behind logins.

#include "server.hpp"

#include <api/AuthService.hpp>
#include <pg/admin.hpp>
#include <pg/database.hpp>
#include <pg/stores.hpp>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <iostream>
#include <thread>

namespace ruxd {

namespace {

std::shared_ptr<pg::Database> open_database(const Config &cfg,
                                            std::size_t pool_size) {
  if (cfg.pg_url.empty())
    throw std::runtime_error("server mode needs Postgres: pass --pg-url (or "
                             "DATABASE_URL), or serve one person with "
                             "--local");
  auto db = std::make_shared<pg::Database>(
      cfg.pg_url, pool_size,
      std::chrono::milliseconds(cfg.pg_acquire_timeout_ms));
  const auto applied = db->migrate();
  if (!applied.empty())
    spdlog::info("postgres: applied {} migration(s)", applied.size());
  return db;
}

std::size_t pool_size_for(const Config &cfg) {
  if (cfg.pg_pool_size > 0)
    return cfg.pg_pool_size;
  if (cfg.threads > 0)
    return cfg.threads;
  return std::max(1u, std::thread::hardware_concurrency());
}

} // namespace

int run_server(Invocation inv) {
  try {
    const Config &cfg = inv.config;
    if (inv.local.server.data_dir.empty())
      throw std::runtime_error("server mode needs --data-dir: where case "
                               "files are stored");
    auto db = open_database(cfg, pool_size_for(cfg));

    api::AuthOptions auth_options;
    auth_options.superuser_token = cfg.auth_token;
    auto auth = std::make_shared<api::AuthService>(pg::postgres_auth_stores(db),
                                                   auth_options);
    if (auth->stores().users->list().empty())
      spdlog::warn("No users yet: create the first administrator with "
                   "`ruxd admin create-user --email <you> --admin`");

    auto jobs = std::make_shared<pg::PgJobStore>(db);
    if (const auto n = jobs->fail_interrupted(); n > 0)
      spdlog::warn("{} job(s) were still queued or running when the server "
                   "last stopped; marked failed",
                   n);

    api::ServerOptions &server = inv.local.server;
    server.case_store = std::make_shared<pg::PgCaseStore>(db, server.data_dir);
    server.auth = auth;
    server.job_store = jobs;
    server.readiness = [db] { return db->ping(); };
    if (!cfg.redis_url.empty() || !cfg.s3_bucket.empty())
      spdlog::warn("--redis-url and --s3-* are reserved and ignored: the "
                   "server does not use Redis or S3 yet");
    return serve_web(std::move(inv.local));
  } catch (const std::exception &e) {
    spdlog::error("Could not start the ruxd server: {}", e.what());
    return 1;
  }
}

int run_admin(Invocation inv) {
  try {
    auto db = open_database(inv.config, 1);
    auto auth = std::make_shared<api::AuthService>(pg::postgres_auth_stores(db),
                                                   api::AuthOptions{});
    std::unique_ptr<pg::PgCaseStore> cases;
    if (inv.admin.kind == pg::AdminCommand::Kind::register_case) {
      if (inv.local.server.data_dir.empty())
        throw std::runtime_error("register-case needs --data-dir (the "
                                 "server's)");
      cases = std::make_unique<pg::PgCaseStore>(db, inv.local.server.data_dir);
    }
    return pg::run_admin(inv.admin, *auth, cases.get(),
                         pg::stdin_password_reader(), std::cout, std::cerr);
  } catch (const std::exception &e) {
    std::cerr << "ruxd admin: " << e.what() << '\n';
    return 1;
  }
}

} // namespace ruxd
