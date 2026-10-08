// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Postgres-backed stores for ruxd's server mode (spec 2026-10-08, phase S3),
// behind the same interfaces the in-memory stores implement:
//  * users, sessions, API tokens, case membership and the audit log
//    (api/auth_stores.hpp);
//  * the case catalogue (api::ICaseStore): case FILES stay in --data-dir,
//    one directory per case, and the `cases` table points at them. (Local
//    mode keeps LocalCaseStore and its `.ruxd/cases.json`; server mode never
//    reads or writes that file.)
//  * job records (reusex::pipeline::IJobStore).
//
// Tested against a real, ephemeral Postgres in tests/unit/ruxd_pg/ (tagged
// [postgres], skipped when initdb is absent).

#include "pg/database.hpp"

#include <api/auth_stores.hpp>
#include <api/cases.hpp>
#include <reusex/pipeline/JobStore.hpp>

#include <filesystem>
#include <memory>
#include <mutex>

namespace ruxd::pg {

class PgUserStore final : public api::IUserStore {
    public:
  explicit PgUserStore(std::shared_ptr<Database> db);
  api::User create(const std::string &email, const std::string &display_name,
                   const std::string &password_hash, bool is_admin) override;
  std::optional<api::User> find_by_email(std::string_view email) const override;
  std::optional<api::User> find_by_id(std::int64_t id) const override;
  std::optional<std::pair<api::User, std::string>>
  find_credentials(std::string_view email) const override;
  std::vector<api::User> list() const override;
  std::optional<std::string> password_hash(std::int64_t user_id) const override;
  void set_password_hash(std::int64_t user_id,
                         const std::string &hash) override;
  void set_disabled(std::int64_t user_id, bool disabled) override;
  void set_admin(std::int64_t user_id, bool is_admin) override;
  void set_display_name(std::int64_t user_id, const std::string &name) override;

    private:
  std::shared_ptr<Database> db_;
};

class PgSessionStore final : public api::ISessionStore {
    public:
  explicit PgSessionStore(std::shared_ptr<Database> db);
  void create(const api::SessionRecord &session) override;
  std::optional<api::SessionRecord>
  find(std::string_view token_hash) const override;
  void renew(std::string_view token_hash,
             api::SystemClock::time_point expires_at,
             api::SystemClock::time_point last_seen) override;
  void revoke(std::string_view token_hash) override;
  void revoke_user(std::int64_t user_id) override;
  std::size_t purge_expired(api::SystemClock::time_point now) override;

    private:
  std::shared_ptr<Database> db_;
};

class PgApiTokenStore final : public api::IApiTokenStore {
    public:
  explicit PgApiTokenStore(std::shared_ptr<Database> db);
  /// @throws HttpError(404) when the token's case does not exist.
  api::ApiTokenRecord create(api::ApiTokenRecord token) override;
  std::optional<api::ApiTokenRecord>
  find(std::string_view token_hash) const override;
  std::vector<api::ApiTokenRecord>
  list(std::optional<std::int64_t> user_id) const override;
  bool revoke(std::int64_t id, std::optional<std::int64_t> owner) override;
  void touch(std::int64_t id, api::SystemClock::time_point now) override;

    private:
  std::shared_ptr<Database> db_;
};

class PgMembershipStore final : public api::IMembershipStore {
    public:
  explicit PgMembershipStore(std::shared_ptr<Database> db);
  std::optional<api::Role> role_of(std::string_view case_id,
                                   std::int64_t user_id) const override;
  std::vector<api::Member> members(std::string_view case_id) const override;
  /// @throws HttpError(404) for an unknown case.
  void set_role(std::string_view case_id, std::int64_t user_id,
                api::Role role) override;
  bool remove(std::string_view case_id, std::int64_t user_id) override;
  /// One transaction that locks the case's row (SELECT … FOR UPDATE), so
  /// concurrent changes to one case's members are serialised and the
  /// last-owner check cannot race.
  api::MemberChange change_role(std::string_view case_id, std::int64_t user_id,
                                api::Role role) override;
  api::MemberChange remove_member(std::string_view case_id,
                                  std::int64_t user_id) override;
  std::set<std::string> cases_of(std::int64_t user_id) const override;
  /// A no-op: deleting the `cases` row cascades.
  void forget_case(std::string_view case_id) override;

    private:
  std::shared_ptr<Database> db_;
};

class PgAuditLog final : public api::IAuditLog {
    public:
  explicit PgAuditLog(std::shared_ptr<Database> db);
  void record(const api::AuditEntry &entry) override;
  std::size_t prune(api::SystemClock::time_point before) override;

    private:
  std::shared_ptr<Database> db_;
};

/// Every auth store over one database.
api::AuthStores postgres_auth_stores(const std::shared_ptr<Database> &db);

/// The case catalogue of server mode: rows in `cases`, files in
/// `<data_dir>/<slug>/project.rux`. Uploads stage in `<data_dir>/.ruxd/
/// uploads/`, deleted cases go to `<data_dir>/.ruxd/trash/`.
class PgCaseStore final : public api::ICaseStore {
    public:
  /// @throws std::runtime_error when @p data_dir cannot be created.
  PgCaseStore(std::shared_ptr<Database> db, std::filesystem::path data_dir);

  std::vector<api::CaseInfo> list() const override;
  std::optional<api::CaseInfo> find(std::string_view id) const override;
  bool writable() const override { return true; }
  std::filesystem::path staging_dir() const override;
  api::CaseInfo create(const std::string &name,
                       std::optional<std::int64_t> created_by) override;
  api::CaseInfo adopt(const std::string &name,
                      const std::filesystem::path &staged_file,
                      std::optional<std::int64_t> created_by) override;
  api::CaseInfo update(std::string_view id,
                       const api::CasePatch &patch) override;
  std::filesystem::path move_to_trash(std::string_view id) override;

  /// Register an existing project file as a case (`ruxd admin`): the file
  /// stays where it is and is never moved or deleted by the server (the case
  /// is not `deletable`).
  /// @throws HttpError(422) when @p file is not a ReUseX project.
  api::CaseInfo register_path(const std::string &name,
                              const std::filesystem::path &file,
                              std::optional<std::int64_t> created_by);

  const std::filesystem::path &data_dir() const noexcept { return data_dir_; }

    private:
  /// A fresh `<data_dir>/<slug>` directory and its slug.
  std::pair<std::string, std::filesystem::path>
  new_case_dir(const std::string &name);
  api::CaseInfo insert(const std::string &slug, const std::string &name,
                       const std::filesystem::path &file,
                       std::optional<std::int64_t> created_by);

  std::shared_ptr<Database> db_;
  std::filesystem::path data_dir_;
  std::mutex write_mutex_; ///< One create/adopt/delete at a time.
};

/// Job records in the `jobs` table, keyed by case slug. Retention as
/// InMemoryJobStore: past @p max_terminal_jobs finished records per case,
/// the oldest go.
class PgJobStore final : public reusex::pipeline::IJobStore {
    public:
  explicit PgJobStore(std::shared_ptr<Database> db,
                      std::size_t max_terminal_jobs = 256);

  void save(std::string_view queue,
            const reusex::pipeline::JobRecord &record) override;
  std::optional<reusex::pipeline::JobRecord>
  find(std::string_view queue, std::string_view id) const override;
  std::vector<reusex::pipeline::JobRecord>
  list(std::string_view queue) const override;
  void forget(std::string_view queue) override;

  /// Jobs a previous process left queued or running can never finish: mark
  /// them failed ("the server restarted"). Call once at startup.
  /// @return how many.
  std::size_t fail_interrupted();

    private:
  std::shared_ptr<Database> db_;
  std::size_t max_terminal_;
};

/// JobRecord <-> the `record` jsonb column (exposed for the tests).
std::string job_record_to_json(const reusex::pipeline::JobRecord &record);
reusex::pipeline::JobRecord job_record_from_json(const std::string &text);

} // namespace ruxd::pg
