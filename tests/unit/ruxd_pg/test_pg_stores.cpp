// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// ruxd server mode against a real Postgres (spec 2026-10-08, phase S3):
// migrations (idempotent, concurrent, refusing a newer database), and every
// Postgres-backed store behind its interface. Each [postgres] test starts an
// ephemeral cluster (tests/support/pg_fixture.hpp) and SKIPs without initdb.

#include <catch2/catch_test_macros.hpp>

#include <api/AuthService.hpp>
#include <pg/admin.hpp>
#include <pg/database.hpp>
#include <pg/migrations.hpp>
#include <pg/stores.hpp>

#include <reusex/core/ProjectDB.hpp>

#include <pqxx/pqxx>

#include "../../support/pg_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <atomic>
#include <chrono>
#include <fstream>
#include <functional>
#include <set>
#include <sstream>
#include <thread>

namespace fs = std::filesystem;
namespace pipeline = reusex::pipeline;
using namespace ruxd;
using reusex::test_support::EphemeralPostgres;
using reusex::test_support::TempDir;
using reusex::test_support::TempPath;
using namespace std::chrono_literals;

namespace {

std::shared_ptr<pg::Database> migrated(const EphemeralPostgres &server) {
  auto db = std::make_shared<pg::Database>(server.dsn(), 4);
  db->migrate();
  return db;
}

api::AuthOptions fast_auth() {
  api::AuthOptions options;
  options.argon2.iterations = 1;
  options.argon2.memory_kib = 256;
  options.argon2.lanes = 1;
  return options;
}

int status_of(const std::function<void()> &fn) {
  try {
    fn();
  } catch (const api::HttpError &e) {
    return e.status();
  }
  return 0;
}

} // namespace

TEST_CASE("PgPendingMigrations_OrderAndUnknownVersions", "[ruxd_pg]") {
  const std::vector<pg::Migration> all{
      {2, "b", "SELECT 2"}, {1, "a", "SELECT 1"}, {3, "c", "SELECT 3"}};
  const auto todo = pg::pending_migrations(all, {2});
  REQUIRE(todo.size() == 2);
  CHECK(todo[0].version == 1);
  CHECK(todo[1].version == 3);
  CHECK(pg::pending_migrations(all, {1, 2, 3}).empty());
  // A database migrated by a newer ruxd is refused, not half-used.
  CHECK_THROWS(pg::pending_migrations(all, {4}));
  CHECK_THROWS(pg::pending_migrations({{1, "a", ""}, {1, "b", ""}}, {}));
}

TEST_CASE("PgEmbeddedMigrations_StartAtOneAndAreDense", "[ruxd_pg]") {
  const auto &all = pg::embedded_migrations();
  REQUIRE_FALSE(all.empty());
  for (std::size_t i = 0; i < all.size(); ++i) {
    CHECK(all[i].version == static_cast<int>(i) + 1);
    CHECK_FALSE(all[i].sql.empty());
  }
  CHECK(all.front().name == "initial");
}

TEST_CASE("PgMigrate_AppliesOnceAndRecordsEach", "[ruxd_pg][postgres]") {
  REUSEX_REQUIRE_POSTGRES();
  EphemeralPostgres server;
  pqxx::connection conn(server.dsn());
  const auto first = pg::migrate(conn);
  CHECK(first.size() == pg::embedded_migrations().size());
  CHECK(pg::migrate(conn).empty()); // idempotent
  CHECK(pg::applied_migrations(conn).size() == first.size());

  pqxx::nontransaction tx(conn);
  for (const char *table : {"users", "sessions", "api_tokens", "cases",
                            "case_members", "jobs", "audit_log"})
    CHECK(tx.exec("SELECT to_regclass($1) IS NOT NULL",
                  pqxx::params{std::string("public.") + table})
              .one_field()
              .as<bool>());
}

TEST_CASE("PgMigrate_ConcurrentStarts_ApplyEachMigrationOnce",
          "[ruxd_pg][postgres]") {
  REUSEX_REQUIRE_POSTGRES();
  EphemeralPostgres server;
  // Four processes' worth of servers starting at once: the advisory lock
  // serialises them, so exactly one applies the schema and none fails.
  std::atomic<int> applied{0}, failed{0};
  std::vector<std::thread> threads;
  for (int i = 0; i < 4; ++i)
    threads.emplace_back([&] {
      try {
        pqxx::connection conn(server.dsn());
        applied += static_cast<int>(pg::migrate(conn).size());
      } catch (const std::exception &) {
        ++failed;
      }
    });
  for (auto &t : threads)
    t.join();
  CHECK(failed.load() == 0);
  CHECK(applied.load() == static_cast<int>(pg::embedded_migrations().size()));
}

TEST_CASE("PgMigrate_FailingMigration_RolledBack", "[ruxd_pg][postgres]") {
  REUSEX_REQUIRE_POSTGRES();
  EphemeralPostgres server;
  pqxx::connection conn(server.dsn());
  const std::vector<pg::Migration> broken{
      {1, "ok", "CREATE TABLE a (x int);"},
      {2, "bad", "CREATE TABLE b (x int); SELECT no_such_function();"}};
  CHECK_THROWS(pg::migrate(conn, broken));
  // 1 is in; 2 left nothing behind, not even its first statement.
  CHECK(pg::applied_migrations(conn) == std::set<int>{1});
  pqxx::nontransaction tx(conn);
  CHECK_FALSE(tx.exec("SELECT to_regclass('public.b') IS NOT NULL")
                  .one_field()
                  .as<bool>());
}

TEST_CASE("PgAuthStores_LoginSessionsTokensMembersAudit",
          "[ruxd_pg][postgres]") {
  REUSEX_REQUIRE_POSTGRES();
  EphemeralPostgres server;
  auto db = migrated(server);
  TempDir data("ruxd_pg_data");
  pg::PgCaseStore cases(db, data.path);
  auto stores = pg::postgres_auth_stores(db);
  api::AuthService auth(stores, fast_auth());

  const auto anna = auth.create_user("Anna@Example.dk", "Anna", "hemmeligt1",
                                     /*is_admin=*/true);
  const auto bo = auth.create_user("bo@example.dk", "Bo", "hemmeligt2", false);
  CHECK(anna.email == "anna@example.dk");
  CHECK(anna.is_admin);
  CHECK(status_of([&] {
          auth.create_user("ANNA@example.dk", "x", "hemmeligt1", false);
        }) == 409);
  CHECK(stores.users->list().size() == 2);
  CHECK(stores.users->find_by_email(" BO@example.dk")->id == bo.id);
  CHECK(stores.users->find_by_id(bo.id)->display_name == "Bo");

  // Login: a session stored as a digest, renewed, revoked.
  const auto login = auth.login("anna@example.dk", "hemmeligt1", "127.0.0.1");
  api::PresentedCredentials c;
  c.session_cookies.push_back(login.session_token);
  CHECK(auth.authenticate(c).user.id == anna.id);
  {
    pqxx::connection conn(server.dsn());
    pqxx::nontransaction tx(conn);
    CHECK(tx.exec("SELECT count(*) FROM sessions WHERE token_hash = $1",
                  pqxx::params{login.session_token})
              .one_field()
              .as<int>() == 0);
    CHECK(tx.exec("SELECT count(*) FROM sessions WHERE token_hash = $1",
                  pqxx::params{api::sha256_hex(login.session_token)})
              .one_field()
              .as<int>() == 1);
  }
  auth.logout(auth.authenticate(c));
  CHECK_FALSE(auth.authenticate(c).authenticated());
  CHECK(status_of([&] { auth.login("bo@example.dk", "forkert1", "x"); }) ==
        401);

  // A case, membership and a case-scoped token.
  const auto kontor = cases.create("Kontor", anna.id);
  stores.members->set_role(kontor.id, bo.id, api::Role::viewer);
  CHECK(stores.members->role_of(kontor.id, bo.id) == api::Role::viewer);
  stores.members->set_role(kontor.id, bo.id, api::Role::editor);
  CHECK(stores.members->role_of(kontor.id, bo.id) == api::Role::editor);
  CHECK(stores.members->members(kontor.id).size() == 1);
  CHECK(stores.members->cases_of(bo.id) == std::set<std::string>{kontor.id});
  CHECK(status_of([&] {
          stores.members->set_role("ingen", bo.id, api::Role::viewer);
        }) == 404);

  const auto token = auth.create_api_token(bo.id, "ci", kontor.id);
  api::PresentedCredentials bearer;
  bearer.bearer = token;
  const auto as_token = auth.authenticate(bearer);
  CHECK(as_token.kind == api::PrincipalKind::api_token);
  CHECK(as_token.case_scope == kontor.id);
  CHECK(status_of([&] { auth.create_api_token(bo.id, "x", "ingen"); }) == 404);

  CHECK(stores.members->remove(kontor.id, bo.id));
  CHECK_FALSE(stores.members->remove(kontor.id, bo.id));

  // Audit rows were written (login, logout, a failure).
  pqxx::connection conn(server.dsn());
  pqxx::nontransaction tx(conn);
  CHECK(tx.exec("SELECT count(*) FROM audit_log WHERE action = 'auth.login'")
            .one_field()
            .as<int>() == 1);
  CHECK(tx.exec("SELECT count(*) FROM audit_log WHERE action = "
                "'auth.login_failed'")
            .one_field()
            .as<int>() == 1);

  // Disabling ends sessions; expired sessions are purged.
  const auto again = auth.login("anna@example.dk", "hemmeligt1", "127.0.0.2");
  auth.set_disabled(anna.id, true);
  c.session_cookies = {again.session_token};
  CHECK_FALSE(auth.authenticate(c).authenticated());
  stores.sessions->create({"h", bo.id, api::SystemClock::now() - 2h,
                           api::SystemClock::now() - 1h,
                           api::SystemClock::now() - 2h});
  CHECK(stores.sessions->purge_expired(api::SystemClock::now()) >= 1);
}

TEST_CASE("PgCaseStore_CreateAdoptRenameDelete", "[ruxd_pg][postgres]") {
  REUSEX_REQUIRE_POSTGRES();
  EphemeralPostgres server;
  auto db = migrated(server);
  TempDir data("ruxd_pg_cases");
  pg::PgCaseStore cases(db, data.path);

  const auto a = cases.create("Kontor Ø", std::nullopt);
  CHECK(a.id == "kontor-oe");
  CHECK(a.deletable);
  CHECK(fs::exists(data.path / "kontor-oe" / "project.rux"));
  const auto b = cases.create("Kontor Ø", std::nullopt);
  CHECK(b.id == "kontor-oe-2");
  CHECK(cases.list().size() == 2);

  // Adopt a staged project.
  const auto staged = cases.staging_dir() / "up.part";
  {
    reusex::ProjectDB db_file(staged, /*readOnly=*/false);
  }
  const auto c = cases.adopt("Upload", staged, std::nullopt);
  CHECK(c.id == "upload");
  CHECK_FALSE(fs::exists(staged));

  // Rename and archive.
  api::CasePatch patch;
  patch.name = "Nyt navn";
  patch.archived = true;
  const auto renamed = cases.update(a.id, patch);
  CHECK(renamed.name == "Nyt navn");
  CHECK(renamed.archived);
  CHECK(status_of([&] { cases.update("ingen", patch); }) == 404);

  // Delete: files to the trash, the row (and its members) gone.
  const auto trash = cases.move_to_trash(b.id);
  CHECK(fs::exists(trash / "project.rux"));
  CHECK_FALSE(cases.find(b.id));
  CHECK(status_of([&] { cases.move_to_trash("ingen"); }) == 404);

  // A registered path stays where it is and is not deletable.
  TempPath outside("ruxd_pg_outside", ".rux");
  {
    reusex::ProjectDB db_file(outside.path, /*readOnly=*/false);
  }
  const auto r = cases.register_path("", outside.path, std::nullopt);
  CHECK_FALSE(r.deletable);
  CHECK(r.path == fs::canonical(outside.path));
  CHECK(status_of([&] { cases.move_to_trash(r.id); }) == 409);
  // Not a project: refused.
  TempPath junk("ruxd_pg_junk", ".rux");
  {
    std::ofstream(junk.path) << "not sqlite";
  }
  CHECK(status_of([&] { cases.register_path("x", junk.path, std::nullopt); }) ==
        422);
}

TEST_CASE("PgJobStore_RoundTripRetentionAndRestart", "[ruxd_pg][postgres]") {
  REUSEX_REQUIRE_POSTGRES();
  EphemeralPostgres server;
  auto db = migrated(server);
  TempDir data("ruxd_pg_jobs");
  pg::PgCaseStore cases(db, data.path);
  const auto k = cases.create("K", std::nullopt);
  pg::PgJobStore jobs(db, /*max_terminal_jobs=*/2);

  pipeline::JobRecord r;
  r.id = "job-1";
  r.stage = pipeline::JobStage::planes;
  r.status = pipeline::JobStatus::running;
  r.parameters = R"({"a":1})";
  r.submitted_at = "2026-10-08T10:00:00Z";
  r.started_at = "2026-10-08T10:00:01Z";
  r.submitted_by = "7";
  r.result_outputs.push_back({"cloud", "planes", 42});
  jobs.save(k.id, r);
  auto back = jobs.find(k.id, "job-1");
  REQUIRE(back);
  CHECK(back->status == pipeline::JobStatus::running);
  CHECK(back->parameters == r.parameters);
  CHECK(back->submitted_by == "7");
  REQUIRE(back->result_outputs.size() == 1);
  CHECK(back->result_outputs[0].count == 42);
  // Another case cannot read it by id.
  CHECK_FALSE(jobs.find("other", "job-1"));

  // A restart fails what was in flight.
  CHECK(jobs.fail_interrupted() == 1);
  back = jobs.find(k.id, "job-1");
  CHECK(back->status == pipeline::JobStatus::failed);
  CHECK_FALSE(back->error.empty());

  // Retention: past two finished jobs, the oldest go.
  for (int i = 2; i <= 4; ++i) {
    pipeline::JobRecord t;
    t.id = "job-" + std::to_string(i);
    t.status = pipeline::JobStatus::succeeded;
    t.submitted_at = "2026-10-08T10:0" + std::to_string(i) + ":00Z";
    jobs.save(k.id, t);
  }
  const auto list = jobs.list(k.id);
  REQUIRE(list.size() == 2);
  CHECK(list[0].id == "job-4");
  CHECK(list[1].id == "job-3");

  jobs.forget(k.id);
  CHECK(jobs.list(k.id).empty());
}

TEST_CASE("PgAdmin_CreateUserPasswordFromReaderNeverArgv",
          "[ruxd_pg][postgres]") {
  REUSEX_REQUIRE_POSTGRES();
  EphemeralPostgres server;
  auto db = migrated(server);
  api::AuthService auth(pg::postgres_auth_stores(db), fast_auth());
  std::ostringstream out, err;
  int asked = 0;
  const pg::PasswordReader reader = [&](const std::string &) {
    ++asked;
    return std::string("hemmeligt-admin");
  };

  pg::AdminCommand create;
  create.kind = pg::AdminCommand::Kind::create_user;
  create.email = "root@example.dk";
  create.display_name = "Root";
  create.is_admin = true;
  CHECK(pg::run_admin(create, auth, nullptr, reader, out, err) == 0);
  CHECK(asked == 1);
  CHECK(auth.login("root@example.dk", "hemmeligt-admin", "x").user.is_admin);

  pg::AdminCommand list;
  list.kind = pg::AdminCommand::Kind::list_users;
  CHECK(pg::run_admin(list, auth, nullptr, reader, out, err) == 0);
  CHECK(out.str().find("root@example.dk") != std::string::npos);

  pg::AdminCommand token;
  token.kind = pg::AdminCommand::Kind::create_token;
  token.email = "root@example.dk";
  token.token_name = "ci";
  std::ostringstream token_out;
  CHECK(pg::run_admin(token, auth, nullptr, reader, token_out, err) == 0);
  CHECK(token_out.str().rfind("rxt_", 0) == 0);

  pg::AdminCommand disable;
  disable.kind = pg::AdminCommand::Kind::disable_user;
  disable.email = "root@example.dk";
  CHECK(pg::run_admin(disable, auth, nullptr, reader, out, err) == 0);
  CHECK(auth.stores().users->find_by_email("root@example.dk")->disabled);

  pg::AdminCommand missing;
  missing.kind = pg::AdminCommand::Kind::set_password;
  missing.email = "nobody@example.dk";
  CHECK(pg::run_admin(missing, auth, nullptr, reader, out, err) == 3);
}

TEST_CASE("PgMembers_LastOwnerCheckIsAtomic", "[ruxd_pg][postgres]") {
  REUSEX_REQUIRE_POSTGRES();
  EphemeralPostgres server;
  auto db = migrated(server);
  TempDir data("ruxd_pg_owners");
  pg::PgCaseStore cases(db, data.path);
  api::AuthService auth(pg::postgres_auth_stores(db), fast_auth());
  auto &members = *auth.stores().members;
  const auto k = cases.create("K", std::nullopt);
  const auto a = auth.create_user("a@example.dk", "A", "hemmeligt1", false);
  const auto b = auth.create_user("b@example.dk", "B", "hemmeligt1", false);
  members.set_role(k.id, a.id, api::Role::owner);
  members.set_role(k.id, b.id, api::Role::owner);
  CHECK(members.change_role(k.id, 999, api::Role::viewer) ==
        api::MemberChange::not_member);

  // Two owners demoting each other at the same moment, many times over:
  // exactly one may win each round, never both.
  for (int round = 0; round < 20; ++round) {
    std::atomic<int> done{0};
    std::thread ta([&] {
      if (members.change_role(k.id, a.id, api::Role::viewer) ==
          api::MemberChange::done)
        ++done;
    });
    std::thread tb([&] {
      if (members.remove_member(k.id, b.id) == api::MemberChange::done)
        ++done;
    });
    ta.join();
    tb.join();
    CHECK(done.load() == 1);
    std::size_t owners = 0;
    for (const auto &m : members.members(k.id))
      owners += m.role == api::Role::owner ? 1 : 0;
    CHECK(owners == 1);
    // Reset: both owners again.
    members.set_role(k.id, a.id, api::Role::owner);
    members.set_role(k.id, b.id, api::Role::owner);
  }
}

TEST_CASE("PgTokens_ListRevokeExpiryAndAuditRetention", "[ruxd_pg][postgres]") {
  REUSEX_REQUIRE_POSTGRES();
  EphemeralPostgres server;
  auto db = migrated(server);
  api::AuthService auth(pg::postgres_auth_stores(db), fast_auth());
  const auto u = auth.create_user("t@example.dk", "T", "hemmeligt1", false);
  const auto t1 =
      auth.create_api_token(u.id, "ci", std::nullopt, std::chrono::hours(24));
  auth.create_api_token(u.id, "evig", std::nullopt, std::chrono::seconds(0));
  auto list = auth.stores().tokens->list(u.id);
  REQUIRE(list.size() == 2);
  CHECK(list[0].expires_at);
  CHECK_FALSE(list[1].expires_at);
  api::PresentedCredentials c;
  c.bearer = t1;
  CHECK(auth.authenticate(c).kind == api::PrincipalKind::api_token);
  CHECK(auth.stores().tokens->list(u.id)[0].last_used_at);
  CHECK(auth.stores().tokens->revoke(list[0].id, u.id));
  CHECK_FALSE(auth.stores().tokens->revoke(list[0].id, u.id));
  CHECK(auth.authenticate(c).kind == api::PrincipalKind::anonymous);

  // Admin CLI: list and revoke.
  std::ostringstream out, err;
  pg::AdminCommand ls;
  ls.kind = pg::AdminCommand::Kind::list_tokens;
  CHECK(pg::run_admin(ls, auth, nullptr, {}, out, err) == 0);
  CHECK(out.str().find("evig") != std::string::npos);
  pg::AdminCommand rv;
  rv.kind = pg::AdminCommand::Kind::revoke_token;
  rv.token_id = list[1].id;
  CHECK(pg::run_admin(rv, auth, nullptr, {}, out, err) == 0);
  CHECK(auth.stores().tokens->list(u.id).empty());

  // Audit retention: entries older than the cut are deleted.
  auth.audit(api::superuser_principal(), std::nullopt, "test.old");
  CHECK(auth.stores().audit->prune(api::SystemClock::now() +
                                   std::chrono::hours(1)) >= 1);
  pqxx::connection conn(server.dsn());
  pqxx::nontransaction tx(conn);
  CHECK(tx.exec("SELECT count(*) FROM audit_log").one_field().as<int>() == 0);
}
