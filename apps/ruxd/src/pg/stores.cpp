// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "pg/stores.hpp"

#include <api/api.hpp>

#include <reusex/core/ProjectDB.hpp>

#include <nlohmann/json.hpp>
#include <pqxx/pqxx>
#include <spdlog/spdlog.h>

#include <chrono>
#include <system_error>

namespace ruxd::pg {

namespace fs = std::filesystem;
namespace pipeline = reusex::pipeline;
using api::HttpError;
using json = nlohmann::json;

namespace {

constexpr std::string_view kCaseFileName = "project.rux";
constexpr std::string_view kMetaDir = ".ruxd";

/// ISO-8601 UTC, as every timestamp on the wire.
constexpr const char *kIso =
    "to_char({} AT TIME ZONE 'UTC', 'YYYY-MM-DD\"T\"HH24:MI:SS\"Z\"')";

std::string iso(std::string_view column) {
  std::string out = kIso;
  out.replace(out.find("{}"), 2, column);
  return out;
}

double to_epoch(api::SystemClock::time_point tp) {
  return std::chrono::duration<double>(tp.time_since_epoch()).count();
}

api::SystemClock::time_point from_epoch(double seconds) {
  return api::SystemClock::time_point(
      std::chrono::duration_cast<api::SystemClock::duration>(
          std::chrono::duration<double>(seconds)));
}

const std::string kUserColumns =
    "id, email, display_name, is_admin, disabled, " + iso("created_at");

api::User user_of(const pqxx::row &row) {
  api::User user;
  user.id = row[0].as<std::int64_t>();
  user.email = row[1].as<std::string>();
  user.display_name = row[2].as<std::string>();
  user.is_admin = row[3].as<bool>();
  user.disabled = row[4].as<bool>();
  user.created_at = row[5].as<std::string>();
  return user;
}

void require_one(const pqxx::result &result, const char *what) {
  if (result.affected_rows() == 0)
    throw HttpError(404, std::string("no such ") + what);
}

} // namespace

// --- users
// -----------------------------------------------------------------------

PgUserStore::PgUserStore(std::shared_ptr<Database> db) : db_(std::move(db)) {}

api::User PgUserStore::create(const std::string &email,
                              const std::string &display_name,
                              const std::string &password_hash, bool is_admin) {
  const std::string key = api::normalize_email(email);
  try {
    return db_->with([&](pqxx::connection &c) {
      pqxx::work tx(c);
      const auto row =
          tx.exec("INSERT INTO users (email, display_name, password_hash, "
                  "is_admin) VALUES ($1, $2, $3, $4) RETURNING " +
                      kUserColumns,
                  pqxx::params{key, display_name.empty() ? key : display_name,
                               password_hash, is_admin})
              .one_row();
      tx.commit();
      return user_of(row);
    });
  } catch (const pqxx::unique_violation &) {
    throw HttpError(409, "a user with that email already exists");
  }
}

std::optional<api::User>
PgUserStore::find_by_email(std::string_view email) const {
  return db_->with([&](pqxx::connection &c) -> std::optional<api::User> {
    pqxx::read_transaction tx(c);
    const auto rows =
        tx.exec("SELECT " + kUserColumns + " FROM users WHERE email = $1",
                pqxx::params{api::normalize_email(email)});
    if (rows.empty())
      return std::nullopt;
    return user_of(rows[0]);
  });
}

std::optional<api::User> PgUserStore::find_by_id(std::int64_t id) const {
  return db_->with([&](pqxx::connection &c) -> std::optional<api::User> {
    pqxx::read_transaction tx(c);
    const auto rows =
        tx.exec("SELECT " + kUserColumns + " FROM users WHERE id = $1",
                pqxx::params{id});
    if (rows.empty())
      return std::nullopt;
    return user_of(rows[0]);
  });
}

std::vector<api::User> PgUserStore::list() const {
  return db_->with([&](pqxx::connection &c) {
    pqxx::read_transaction tx(c);
    std::vector<api::User> out;
    for (const auto &row :
         tx.exec("SELECT " + kUserColumns + " FROM users ORDER BY email"))
      out.push_back(user_of(row));
    return out;
  });
}

std::optional<std::string>
PgUserStore::password_hash(std::int64_t user_id) const {
  return db_->with([&](pqxx::connection &c) -> std::optional<std::string> {
    pqxx::read_transaction tx(c);
    const auto rows = tx.exec("SELECT password_hash FROM users WHERE id = $1",
                              pqxx::params{user_id});
    if (rows.empty())
      return std::nullopt;
    return rows[0][0].as<std::string>();
  });
}

void PgUserStore::set_password_hash(std::int64_t user_id,
                                    const std::string &hash) {
  db_->with([&](pqxx::connection &c) {
    pqxx::work tx(c);
    require_one(tx.exec("UPDATE users SET password_hash = $2 WHERE id = $1",
                        pqxx::params{user_id, hash}),
                "user");
    tx.commit();
  });
}

void PgUserStore::set_disabled(std::int64_t user_id, bool disabled) {
  db_->with([&](pqxx::connection &c) {
    pqxx::work tx(c);
    require_one(tx.exec("UPDATE users SET disabled = $2 WHERE id = $1",
                        pqxx::params{user_id, disabled}),
                "user");
    tx.commit();
  });
}

void PgUserStore::set_admin(std::int64_t user_id, bool is_admin) {
  db_->with([&](pqxx::connection &c) {
    pqxx::work tx(c);
    require_one(tx.exec("UPDATE users SET is_admin = $2 WHERE id = $1",
                        pqxx::params{user_id, is_admin}),
                "user");
    tx.commit();
  });
}

void PgUserStore::set_display_name(std::int64_t user_id,
                                   const std::string &name) {
  db_->with([&](pqxx::connection &c) {
    pqxx::work tx(c);
    require_one(tx.exec("UPDATE users SET display_name = $2 WHERE id = $1",
                        pqxx::params{user_id, name}),
                "user");
    tx.commit();
  });
}

// --- sessions
// --------------------------------------------------------------------

PgSessionStore::PgSessionStore(std::shared_ptr<Database> db)
    : db_(std::move(db)) {}

void PgSessionStore::create(const api::SessionRecord &s) {
  db_->with([&](pqxx::connection &c) {
    pqxx::work tx(c);
    tx.exec("INSERT INTO sessions (token_hash, user_id, created_at, "
            "expires_at, last_seen) VALUES ($1, $2, to_timestamp($3), "
            "to_timestamp($4), to_timestamp($5))",
            pqxx::params{s.token_hash, s.user_id, to_epoch(s.created_at),
                         to_epoch(s.expires_at), to_epoch(s.last_seen)});
    tx.commit();
  });
}

std::optional<api::SessionRecord>
PgSessionStore::find(std::string_view token_hash) const {
  return db_->with(
      [&](pqxx::connection &c) -> std::optional<api::SessionRecord> {
        pqxx::read_transaction tx(c);
        const auto rows =
            tx.exec("SELECT user_id, extract(epoch FROM created_at)::float8, "
                    "extract(epoch FROM expires_at)::float8, "
                    "extract(epoch FROM last_seen)::float8 FROM sessions "
                    "WHERE token_hash = $1",
                    pqxx::params{std::string(token_hash)});
        if (rows.empty())
          return std::nullopt;
        api::SessionRecord s;
        s.token_hash = std::string(token_hash);
        s.user_id = rows[0][0].as<std::int64_t>();
        s.created_at = from_epoch(rows[0][1].as<double>());
        s.expires_at = from_epoch(rows[0][2].as<double>());
        s.last_seen = from_epoch(rows[0][3].as<double>());
        return s;
      });
}

void PgSessionStore::renew(std::string_view token_hash,
                           api::SystemClock::time_point expires_at,
                           api::SystemClock::time_point last_seen) {
  db_->with([&](pqxx::connection &c) {
    pqxx::work tx(c);
    tx.exec("UPDATE sessions SET expires_at = to_timestamp($2), "
            "last_seen = to_timestamp($3) WHERE token_hash = $1",
            pqxx::params{std::string(token_hash), to_epoch(expires_at),
                         to_epoch(last_seen)});
    tx.commit();
  });
}

void PgSessionStore::revoke(std::string_view token_hash) {
  db_->with([&](pqxx::connection &c) {
    pqxx::work tx(c);
    tx.exec("DELETE FROM sessions WHERE token_hash = $1",
            pqxx::params{std::string(token_hash)});
    tx.commit();
  });
}

void PgSessionStore::revoke_user(std::int64_t user_id) {
  db_->with([&](pqxx::connection &c) {
    pqxx::work tx(c);
    tx.exec("DELETE FROM sessions WHERE user_id = $1", pqxx::params{user_id});
    tx.commit();
  });
}

std::size_t PgSessionStore::purge_expired(api::SystemClock::time_point now) {
  return db_->with([&](pqxx::connection &c) {
    pqxx::work tx(c);
    const auto r = tx.exec("DELETE FROM sessions WHERE expires_at <= "
                           "to_timestamp($1)",
                           pqxx::params{to_epoch(now)});
    tx.commit();
    return static_cast<std::size_t>(r.affected_rows());
  });
}

// --- API tokens
// ------------------------------------------------------------------

PgApiTokenStore::PgApiTokenStore(std::shared_ptr<Database> db)
    : db_(std::move(db)) {}

api::ApiTokenRecord PgApiTokenStore::create(api::ApiTokenRecord token) {
  return db_->with([&](pqxx::connection &c) {
    pqxx::work tx(c);
    std::optional<std::int64_t> case_pk;
    if (token.case_id) {
      const auto rows = tx.exec("SELECT id FROM cases WHERE slug = $1",
                                pqxx::params{*token.case_id});
      if (rows.empty())
        throw HttpError(404, "no such case '" + *token.case_id + "'");
      case_pk = rows[0][0].as<std::int64_t>();
    }
    token.id = tx.exec("INSERT INTO api_tokens (token_hash, user_id, name, "
                       "case_id, created_at) VALUES ($1, $2, $3, $4, "
                       "to_timestamp($5)) RETURNING id",
                       pqxx::params{token.token_hash, token.user_id, token.name,
                                    case_pk, to_epoch(token.created_at)})
                   .one_field()
                   .as<std::int64_t>();
    tx.commit();
    return token;
  });
}

std::optional<api::ApiTokenRecord>
PgApiTokenStore::find(std::string_view token_hash) const {
  return db_->with(
      [&](pqxx::connection &c) -> std::optional<api::ApiTokenRecord> {
        pqxx::read_transaction tx(c);
        const auto rows = tx.exec(
            "SELECT t.id, t.user_id, t.name, c.slug, "
            "extract(epoch FROM t.created_at)::float8 FROM api_tokens t "
            "LEFT JOIN cases c ON c.id = t.case_id WHERE t.token_hash = $1",
            pqxx::params{std::string(token_hash)});
        if (rows.empty())
          return std::nullopt;
        api::ApiTokenRecord t;
        t.id = rows[0][0].as<std::int64_t>();
        t.token_hash = std::string(token_hash);
        t.user_id = rows[0][1].as<std::int64_t>();
        t.name = rows[0][2].as<std::string>();
        if (!rows[0][3].is_null())
          t.case_id = rows[0][3].as<std::string>();
        t.created_at = from_epoch(rows[0][4].as<double>());
        return t;
      });
}

// --- membership
// ------------------------------------------------------------------

PgMembershipStore::PgMembershipStore(std::shared_ptr<Database> db)
    : db_(std::move(db)) {}

std::optional<api::Role>
PgMembershipStore::role_of(std::string_view case_id,
                           std::int64_t user_id) const {
  return db_->with([&](pqxx::connection &c) -> std::optional<api::Role> {
    pqxx::read_transaction tx(c);
    const auto rows = tx.exec(
        "SELECT m.role FROM case_members m JOIN cases c ON c.id = m.case_id "
        "WHERE c.slug = $1 AND m.user_id = $2",
        pqxx::params{std::string(case_id), user_id});
    if (rows.empty())
      return std::nullopt;
    return api::parse_role(rows[0][0].as<std::string>());
  });
}

std::vector<api::Member>
PgMembershipStore::members(std::string_view case_id) const {
  return db_->with([&](pqxx::connection &c) {
    pqxx::read_transaction tx(c);
    std::vector<api::Member> out;
    for (const auto &row : tx.exec(
             "SELECT u.id, u.email, u.display_name, u.is_admin, u.disabled, " +
                 iso("u.created_at") +
                 ", m.role FROM case_members m "
                 "JOIN cases c ON c.id = m.case_id "
                 "JOIN users u ON u.id = m.user_id "
                 "WHERE c.slug = $1 ORDER BY u.email",
             pqxx::params{std::string(case_id)})) {
      api::Member member;
      member.user = user_of(row);
      member.role =
          api::parse_role(row[6].as<std::string>()).value_or(api::Role::viewer);
      out.push_back(std::move(member));
    }
    return out;
  });
}

void PgMembershipStore::set_role(std::string_view case_id, std::int64_t user_id,
                                 api::Role role) {
  db_->with([&](pqxx::connection &c) {
    pqxx::work tx(c);
    const auto r = tx.exec(
        "INSERT INTO case_members (case_id, user_id, role) "
        "SELECT id, $2, $3 FROM cases WHERE slug = $1 "
        "ON CONFLICT (case_id, user_id) DO UPDATE SET role = EXCLUDED.role",
        pqxx::params{std::string(case_id), user_id,
                     std::string(api::to_string(role))});
    if (r.affected_rows() == 0)
      throw HttpError(404, "no such case '" + std::string(case_id) + "'");
    tx.commit();
  });
}

bool PgMembershipStore::remove(std::string_view case_id, std::int64_t user_id) {
  return db_->with([&](pqxx::connection &c) {
    pqxx::work tx(c);
    const auto r =
        tx.exec("DELETE FROM case_members m USING cases c "
                "WHERE c.id = m.case_id AND c.slug = $1 AND m.user_id = $2",
                pqxx::params{std::string(case_id), user_id});
    tx.commit();
    return r.affected_rows() > 0;
  });
}

std::set<std::string> PgMembershipStore::cases_of(std::int64_t user_id) const {
  return db_->with([&](pqxx::connection &c) {
    pqxx::read_transaction tx(c);
    std::set<std::string> out;
    for (const auto &row :
         tx.exec("SELECT c.slug FROM case_members m JOIN cases c ON c.id = "
                 "m.case_id WHERE m.user_id = $1",
                 pqxx::params{user_id}))
      out.insert(row[0].as<std::string>());
    return out;
  });
}

void PgMembershipStore::forget_case(std::string_view) {}

// --- audit
// -----------------------------------------------------------------------

PgAuditLog::PgAuditLog(std::shared_ptr<Database> db) : db_(std::move(db)) {}

void PgAuditLog::record(const api::AuditEntry &e) {
  db_->with([&](pqxx::connection &c) {
    pqxx::work tx(c);
    tx.exec("INSERT INTO audit_log (user_id, actor, case_id, case_slug, "
            "action, detail) VALUES ($1, $2, "
            "(SELECT id FROM cases WHERE slug = $3), $3, $4, $5)",
            pqxx::params{e.user_id, e.actor, e.case_id, e.action, e.detail});
    tx.commit();
  });
}

api::AuthStores postgres_auth_stores(const std::shared_ptr<Database> &db) {
  api::AuthStores stores;
  stores.users = std::make_shared<PgUserStore>(db);
  stores.sessions = std::make_shared<PgSessionStore>(db);
  stores.tokens = std::make_shared<PgApiTokenStore>(db);
  stores.members = std::make_shared<PgMembershipStore>(db);
  stores.audit = std::make_shared<PgAuditLog>(db);
  return stores;
}

// --- cases
// -----------------------------------------------------------------------

namespace {

const std::string kCaseColumns =
    "slug, name, storage_path, " + iso("created_at") + ", archived";

} // namespace

PgCaseStore::PgCaseStore(std::shared_ptr<Database> db, fs::path data_dir)
    : db_(std::move(db)), data_dir_(std::move(data_dir)) {
  if (data_dir_.empty())
    throw std::runtime_error("server mode needs --data-dir: where case files "
                             "are stored");
  std::error_code ec;
  fs::create_directories(data_dir_ / kMetaDir / "uploads", ec);
  fs::create_directories(data_dir_ / kMetaDir / "trash", ec);
  if (ec)
    throw std::runtime_error("cannot create the data dir '" +
                             data_dir_.string() + "': " + ec.message());
  data_dir_ = fs::canonical(data_dir_);
}

namespace {

api::CaseInfo case_of(const pqxx::row &row, const fs::path &data_dir) {
  api::CaseInfo info;
  info.id = row[0].as<std::string>();
  info.name = row[1].as<std::string>();
  info.path = row[2].as<std::string>();
  info.created_at = row[3].as<std::string>();
  info.archived = row[4].as<bool>();
  std::error_code ec;
  const auto size = fs::file_size(info.path, ec);
  info.size_bytes = ec ? 0 : size;
  // Only a case in its own directory under the data dir is the server's to
  // delete; a registered path belongs to whoever put it there.
  info.deletable = info.path.filename() == kCaseFileName &&
                   info.path.parent_path().parent_path() == data_dir;
  return info;
}

} // namespace

std::vector<api::CaseInfo> PgCaseStore::list() const {
  return db_->with([&](pqxx::connection &c) {
    pqxx::read_transaction tx(c);
    std::vector<api::CaseInfo> out;
    for (const auto &row :
         tx.exec("SELECT " + kCaseColumns + " FROM cases ORDER BY slug"))
      out.push_back(case_of(row, data_dir_));
    return out;
  });
}

std::optional<api::CaseInfo> PgCaseStore::find(std::string_view id) const {
  return db_->with([&](pqxx::connection &c) -> std::optional<api::CaseInfo> {
    pqxx::read_transaction tx(c);
    const auto rows =
        tx.exec("SELECT " + kCaseColumns + " FROM cases WHERE slug = $1",
                pqxx::params{std::string(id)});
    if (rows.empty())
      return std::nullopt;
    return case_of(rows[0], data_dir_);
  });
}

fs::path PgCaseStore::staging_dir() const {
  return data_dir_ / kMetaDir / "uploads";
}

std::pair<std::string, fs::path>
PgCaseStore::new_case_dir(const std::string &name) {
  std::set<std::string> taken;
  for (const auto &info : list())
    taken.insert(info.id);
  const std::string base = api::case_slug(name);
  constexpr std::size_t kMaxSlug = 64;
  for (int n = 1; n < 10000; ++n) {
    const std::string suffix = n == 1 ? "" : "-" + std::to_string(n);
    const std::string slug = base.substr(0, kMaxSlug - suffix.size()) + suffix;
    if (taken.count(slug) != 0)
      continue;
    const fs::path dir = data_dir_ / slug;
    std::error_code ec;
    // create_directory, not create_directories: an existing path is never
    // reused, whoever made it.
    if (fs::create_directory(dir, ec) && !ec)
      return {slug, dir};
  }
  throw HttpError(409,
                  "could not find a free directory name for '" + name + "'");
}

api::CaseInfo PgCaseStore::insert(const std::string &slug,
                                  const std::string &name, const fs::path &file,
                                  std::optional<std::int64_t> created_by) {
  return db_->with([&](pqxx::connection &c) {
    pqxx::work tx(c);
    const auto row =
        tx.exec("INSERT INTO cases (slug, name, storage_path, created_by) "
                "VALUES ($1, $2, $3, $4) RETURNING " +
                    kCaseColumns,
                pqxx::params{slug, name, file.string(), created_by})
            .one_row();
    tx.commit();
    return case_of(row, data_dir_);
  });
}

api::CaseInfo PgCaseStore::create(const std::string &name,
                                  std::optional<std::int64_t> created_by) {
  std::lock_guard<std::mutex> lock(write_mutex_);
  const auto [slug, dir] = new_case_dir(name);
  const fs::path file = dir / kCaseFileName;
  try {
    reusex::ProjectDB db(file, /*readOnly=*/false); // creates and migrates
  } catch (const std::exception &e) {
    std::error_code ec;
    fs::remove_all(dir, ec);
    throw HttpError(500,
                    std::string("could not create the project: ") + e.what());
  }
  try {
    return insert(slug, name, file, created_by);
  } catch (...) {
    std::error_code ec;
    fs::remove_all(dir, ec);
    throw;
  }
}

api::CaseInfo PgCaseStore::adopt(const std::string &name,
                                 const fs::path &staged,
                                 std::optional<std::int64_t> created_by) {
  std::lock_guard<std::mutex> lock(write_mutex_);
  const auto [slug, dir] = new_case_dir(name);
  const fs::path file = dir / kCaseFileName;
  std::error_code ec;
  fs::rename(staged, file, ec);
  if (ec) {
    fs::remove(dir, ec);
    throw std::runtime_error("could not move the upload into place: " +
                             ec.message());
  }
  try {
    reusex::ProjectDB db(file, /*readOnly=*/false); // validates and migrates
  } catch (const std::exception &e) {
    fs::remove_all(dir, ec);
    throw HttpError(422,
                    std::string("not a usable ReUseX project: ") + e.what());
  }
  try {
    return insert(slug, name, file, created_by);
  } catch (...) {
    fs::remove_all(dir, ec);
    throw;
  }
}

api::CaseInfo
PgCaseStore::register_path(const std::string &name, const fs::path &raw_file,
                           std::optional<std::int64_t> created_by) {
  std::error_code ec;
  const fs::path file = fs::weakly_canonical(fs::absolute(raw_file), ec);
  if (ec || !fs::is_regular_file(file))
    throw HttpError(404, "no such file '" + raw_file.string() + "'");
  if (const auto why = api::reusex_project_problem(file); !why.empty())
    throw HttpError(422, "not a ReUseX project: " + why);
  std::lock_guard<std::mutex> lock(write_mutex_);
  std::set<std::string> taken;
  for (const auto &info : list())
    taken.insert(info.id);
  const std::string base =
      api::case_slug(name.empty() ? file.stem().string() : name);
  std::string slug = base;
  for (int n = 2; taken.count(slug) != 0 || fs::exists(data_dir_ / slug); ++n)
    slug = base + "-" + std::to_string(n);
  return insert(slug, name.empty() ? file.stem().string() : name, file,
                created_by);
}

api::CaseInfo PgCaseStore::update(std::string_view id,
                                  const api::CasePatch &patch) {
  return db_->with([&](pqxx::connection &c) {
    pqxx::work tx(c);
    const auto rows =
        tx.exec("UPDATE cases SET name = COALESCE($2, name), "
                "archived = COALESCE($3, archived) WHERE slug = $1 "
                "RETURNING " +
                    kCaseColumns,
                pqxx::params{std::string(id), patch.name, patch.archived});
    if (rows.empty())
      throw HttpError(404, "no such case '" + std::string(id) + "'");
    tx.commit();
    return case_of(rows[0], data_dir_);
  });
}

fs::path PgCaseStore::move_to_trash(std::string_view id) {
  std::lock_guard<std::mutex> lock(write_mutex_);
  const auto info = find(id);
  if (!info)
    throw HttpError(404, "no such case '" + std::string(id) + "'");
  if (!info->deletable)
    throw HttpError(409, "case '" + info->id +
                             "' was registered from a path outside the data "
                             "dir; the server does not delete it");
  const auto now = std::chrono::system_clock::now();
  const auto stamp =
      std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch())
          .count();
  fs::path dest =
      data_dir_ / kMetaDir / "trash" / (std::to_string(stamp) + "-" + info->id);
  for (int n = 2; fs::exists(dest); ++n)
    dest = data_dir_ / kMetaDir / "trash" /
           (std::to_string(stamp) + "-" + info->id + "-" + std::to_string(n));
  const fs::path dir = info->path.parent_path();
  fs::rename(dir, dest);
  try {
    db_->with([&](pqxx::connection &c) {
      pqxx::work tx(c);
      // Cascades to members, scoped tokens and jobs; the audit log keeps
      // the slug.
      tx.exec("DELETE FROM cases WHERE slug = $1",
              pqxx::params{std::string(id)});
      tx.commit();
    });
  } catch (...) {
    std::error_code ec;
    fs::rename(dest, dir, ec); // Put the files back: the case still exists.
    throw;
  }
  spdlog::info("Case '{}' moved to {}", info->id, dest.string());
  return dest;
}

// --- jobs
// ------------------------------------------------------------------------

std::string job_record_to_json(const pipeline::JobRecord &r) {
  json outputs = json::array();
  for (const auto &a : r.result_outputs)
    outputs.push_back({{"kind", a.kind}, {"name", a.name}, {"count", a.count}});
  return json{{"id", r.id},
              {"stage", std::string(pipeline::to_string(r.stage))},
              {"status", std::string(pipeline::to_string(r.status))},
              {"parameters", r.parameters},
              {"error", r.error},
              {"submitted_at", r.submitted_at},
              {"started_at", r.started_at},
              {"finished_at", r.finished_at},
              {"submitted_by", r.submitted_by},
              {"cancel_requested", r.cancel_requested},
              {"progress_stage", static_cast<int>(r.progress_stage)},
              {"progress_current", r.progress_current},
              {"progress_total", r.progress_total},
              {"result_summary", r.result_summary},
              {"result_outputs", std::move(outputs)}}
      .dump();
}

pipeline::JobRecord job_record_from_json(const std::string &text) {
  const auto j = json::parse(text);
  pipeline::JobRecord r;
  r.id = j.value("id", "");
  r.stage = pipeline::parse_job_stage(j.value("stage", ""))
                .value_or(pipeline::JobStage::clouds);
  r.status = pipeline::parse_job_status(j.value("status", ""))
                 .value_or(pipeline::JobStatus::failed);
  r.parameters = j.value("parameters", "");
  r.error = j.value("error", "");
  r.submitted_at = j.value("submitted_at", "");
  r.started_at = j.value("started_at", "");
  r.finished_at = j.value("finished_at", "");
  r.submitted_by = j.value("submitted_by", "");
  r.cancel_requested = j.value("cancel_requested", false);
  r.progress_stage =
      static_cast<reusex::core::Stage>(j.value("progress_stage", 0));
  r.progress_current = j.value("progress_current", std::size_t{0});
  r.progress_total = j.value("progress_total", std::size_t{0});
  r.result_summary = j.value("result_summary", "");
  for (const auto &a : j.value("result_outputs", json::array()))
    r.result_outputs.push_back(pipeline::StageArtifact{
        a.value("kind", ""), a.value("name", ""), a.value("count", -1LL)});
  return r;
}

PgJobStore::PgJobStore(std::shared_ptr<Database> db,
                       std::size_t max_terminal_jobs)
    : db_(std::move(db)), max_terminal_(max_terminal_jobs) {}

void PgJobStore::save(std::string_view queue, const pipeline::JobRecord &r) {
  std::optional<std::int64_t> user;
  try {
    if (!r.submitted_by.empty())
      user = std::stoll(r.submitted_by);
  } catch (const std::exception &) {
  }
  auto maybe = [](const std::string &s) {
    return s.empty() ? std::optional<std::string>() : std::optional(s);
  };
  db_->with([&](pqxx::connection &c) {
    pqxx::work tx(c);
    const auto res = tx.exec(
        "INSERT INTO jobs (id, case_id, user_id, stage, params, state, "
        "submitted_at, started_at, finished_at, error, record) "
        // A submitter that is not (or no longer) a user is stored as NULL.
        "SELECT $1, c.id, (SELECT id FROM users WHERE id = $3), $4, $5::jsonb, "
        "$6, $7::timestamptz, "
        "$8::timestamptz, $9::timestamptz, $10, $11::jsonb "
        "FROM cases c WHERE c.slug = $2 "
        "ON CONFLICT (id) DO UPDATE SET state = EXCLUDED.state, "
        "started_at = EXCLUDED.started_at, finished_at = EXCLUDED.finished_at, "
        "error = EXCLUDED.error, record = EXCLUDED.record",
        pqxx::params{r.id, std::string(queue), user,
                     std::string(pipeline::to_string(r.stage)),
                     maybe(r.parameters),
                     std::string(pipeline::to_string(r.status)), r.submitted_at,
                     maybe(r.started_at), maybe(r.finished_at), maybe(r.error),
                     job_record_to_json(r)});
    if (res.affected_rows() == 0)
      spdlog::warn("Job {} not stored: no case '{}' in the database", r.id,
                   queue);
    if (pipeline::is_terminal(r.status))
      // Retention: past max_terminal finished jobs per case, oldest go.
      tx.exec("DELETE FROM jobs WHERE id IN (SELECT j.id FROM jobs j JOIN "
              "cases c ON c.id = j.case_id WHERE c.slug = $1 AND j.state IN "
              "('succeeded', 'failed', 'cancelled') ORDER BY j.submitted_at "
              "DESC OFFSET $2)",
              pqxx::params{std::string(queue),
                           static_cast<std::int64_t>(max_terminal_)});
    tx.commit();
  });
}

std::optional<pipeline::JobRecord> PgJobStore::find(std::string_view queue,
                                                    std::string_view id) const {
  return db_->with(
      [&](pqxx::connection &c) -> std::optional<pipeline::JobRecord> {
        pqxx::read_transaction tx(c);
        const auto rows =
            tx.exec("SELECT j.record::text FROM jobs j JOIN cases c ON c.id = "
                    "j.case_id WHERE c.slug = $1 AND j.id = $2",
                    pqxx::params{std::string(queue), std::string(id)});
        if (rows.empty())
          return std::nullopt;
        return job_record_from_json(rows[0][0].as<std::string>());
      });
}

std::vector<pipeline::JobRecord>
PgJobStore::list(std::string_view queue) const {
  return db_->with([&](pqxx::connection &c) {
    pqxx::read_transaction tx(c);
    std::vector<pipeline::JobRecord> out;
    for (const auto &row :
         tx.exec("SELECT j.record::text FROM jobs j JOIN cases c ON c.id = "
                 "j.case_id WHERE c.slug = $1 "
                 "ORDER BY j.submitted_at DESC, j.id DESC",
                 pqxx::params{std::string(queue)}))
      out.push_back(job_record_from_json(row[0].as<std::string>()));
    return out;
  });
}

void PgJobStore::forget(std::string_view queue) {
  db_->with([&](pqxx::connection &c) {
    pqxx::work tx(c);
    tx.exec("DELETE FROM jobs j USING cases c WHERE c.id = j.case_id AND "
            "c.slug = $1",
            pqxx::params{std::string(queue)});
    tx.commit();
  });
}

std::size_t PgJobStore::fail_interrupted() {
  return db_->with([&](pqxx::connection &c) {
    pqxx::work tx(c);
    const std::string reason = "the server restarted before the job finished";
    const auto r = tx.exec(
        "UPDATE jobs SET state = 'failed', error = $1, "
        "finished_at = COALESCE(finished_at, now()), "
        "record = record || jsonb_build_object('status', 'failed', 'error', "
        "$1::text, 'finished_at', " +
            iso("now()") + ") WHERE state IN ('queued', 'running')",
        pqxx::params{reason});
    tx.commit();
    return static_cast<std::size_t>(r.affected_rows());
  });
}

} // namespace ruxd::pg
