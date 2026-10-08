// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Where users, sessions, API tokens, case membership and the audit log live
// (spec 2026-10-08, phase S3).
//
// Server mode backs every interface with Postgres (src/pg/, ruxd_pg_lib).
// The in-memory implementations here serve the unit tests; local mode needs
// none of them (its one implicit user owns every case).
//
// Every implementation must be thread-safe. Errors a client caused (an email
// already taken, an unknown user) are HttpError; anything else propagates.

#include "api/access.hpp"

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <vector>

namespace ruxd::api {

using SystemClock = std::chrono::system_clock;

/// Emails are compared and stored lower-cased and trimmed.
std::string normalize_email(std::string_view email);

/// A plausible email address (something@something, no spaces, at most 254
/// characters). Not RFC 5322; it only has to keep junk out of the table.
bool is_plausible_email(std::string_view email);

class IUserStore {
    public:
  virtual ~IUserStore() = default;

  /// @throws HttpError(409) when the email is taken.
  virtual User create(const std::string &email, const std::string &display_name,
                      const std::string &password_hash, bool is_admin) = 0;
  virtual std::optional<User> find_by_email(std::string_view email) const = 0;
  virtual std::optional<User> find_by_id(std::int64_t id) const = 0;
  /// Every user, by email.
  virtual std::vector<User> list() const = 0;
  /// The stored password hash, or nullopt for an unknown id.
  virtual std::optional<std::string>
  password_hash(std::int64_t user_id) const = 0;
  /// @throws HttpError(404) for an unknown id.
  virtual void set_password_hash(std::int64_t user_id,
                                 const std::string &hash) = 0;
  virtual void set_disabled(std::int64_t user_id, bool disabled) = 0;
  virtual void set_admin(std::int64_t user_id, bool is_admin) = 0;
  virtual void set_display_name(std::int64_t user_id,
                                const std::string &name) = 0;
};

/// One login session. The token itself is never stored.
struct SessionRecord {
  std::string token_hash; ///< sha256_hex of the cookie value.
  std::int64_t user_id = 0;
  SystemClock::time_point created_at;
  SystemClock::time_point expires_at;
  SystemClock::time_point last_seen;
};

class ISessionStore {
    public:
  virtual ~ISessionStore() = default;
  virtual void create(const SessionRecord &session) = 0;
  virtual std::optional<SessionRecord>
  find(std::string_view token_hash) const = 0;
  /// Slide the expiry: new expires_at and last_seen.
  virtual void renew(std::string_view token_hash,
                     SystemClock::time_point expires_at,
                     SystemClock::time_point last_seen) = 0;
  virtual void revoke(std::string_view token_hash) = 0;
  /// Every session of @p user_id (a disabled user, a changed password).
  virtual void revoke_user(std::int64_t user_id) = 0;
  /// Delete sessions that expired before @p now. @return how many.
  virtual std::size_t purge_expired(SystemClock::time_point now) = 0;
};

/// One API token (scripts, CI). Hashed like a session.
struct ApiTokenRecord {
  std::int64_t id = 0;
  std::string token_hash;
  std::int64_t user_id = 0;
  std::string name;
  /// Limited to this case id, or every case its user may see.
  std::optional<std::string> case_id;
  SystemClock::time_point created_at;
};

class IApiTokenStore {
    public:
  virtual ~IApiTokenStore() = default;
  /// @return the record with its id assigned.
  virtual ApiTokenRecord create(ApiTokenRecord token) = 0;
  virtual std::optional<ApiTokenRecord>
  find(std::string_view token_hash) const = 0;
};

/// One member of a case.
struct Member {
  User user;
  Role role = Role::viewer;
};

class IMembershipStore {
    public:
  virtual ~IMembershipStore() = default;
  virtual std::optional<Role> role_of(std::string_view case_id,
                                      std::int64_t user_id) const = 0;
  /// Every member, by email.
  virtual std::vector<Member> members(std::string_view case_id) const = 0;
  /// Add @p user_id or change their role.
  virtual void set_role(std::string_view case_id, std::int64_t user_id,
                        Role role) = 0;
  /// @return false when they were not a member.
  virtual bool remove(std::string_view case_id, std::int64_t user_id) = 0;
  /// The case ids @p user_id is a member of.
  virtual std::set<std::string> cases_of(std::int64_t user_id) const = 0;
  /// Drop every membership of a deleted case.
  virtual void forget_case(std::string_view case_id) = 0;
};

/// One audited mutation. Written, never read back by the server.
struct AuditEntry {
  std::optional<std::int64_t> user_id; ///< nullopt: superuser or local.
  std::string actor;                   ///< Email, or "superuser" / "local".
  std::optional<std::string> case_id;
  std::string action; ///< e.g. "POST /api/v1/cases/x/jobs" or "auth.login".
  std::string detail; ///< Optional context; never secrets.
};

class IAuditLog {
    public:
  virtual ~IAuditLog() = default;
  virtual void record(const AuditEntry &entry) = 0;
};

// --- in-memory implementations
// ------------------------------------------------

class InMemoryUserStore final : public IUserStore {
    public:
  User create(const std::string &email, const std::string &display_name,
              const std::string &password_hash, bool is_admin) override;
  std::optional<User> find_by_email(std::string_view email) const override;
  std::optional<User> find_by_id(std::int64_t id) const override;
  std::vector<User> list() const override;
  std::optional<std::string> password_hash(std::int64_t user_id) const override;
  void set_password_hash(std::int64_t user_id,
                         const std::string &hash) override;
  void set_disabled(std::int64_t user_id, bool disabled) override;
  void set_admin(std::int64_t user_id, bool is_admin) override;
  void set_display_name(std::int64_t user_id, const std::string &name) override;

    private:
  struct Row {
    User user;
    std::string hash;
  };
  Row &row(std::int64_t id);
  mutable std::mutex mutex_;
  std::int64_t next_id_ = 1;
  std::map<std::int64_t, Row> rows_;
};

class InMemorySessionStore final : public ISessionStore {
    public:
  void create(const SessionRecord &session) override;
  std::optional<SessionRecord> find(std::string_view token_hash) const override;
  void renew(std::string_view token_hash, SystemClock::time_point expires_at,
             SystemClock::time_point last_seen) override;
  void revoke(std::string_view token_hash) override;
  void revoke_user(std::int64_t user_id) override;
  std::size_t purge_expired(SystemClock::time_point now) override;

    private:
  mutable std::mutex mutex_;
  std::map<std::string, SessionRecord, std::less<>> sessions_;
};

class InMemoryApiTokenStore final : public IApiTokenStore {
    public:
  ApiTokenRecord create(ApiTokenRecord token) override;
  std::optional<ApiTokenRecord>
  find(std::string_view token_hash) const override;

    private:
  mutable std::mutex mutex_;
  std::int64_t next_id_ = 1;
  std::map<std::string, ApiTokenRecord, std::less<>> tokens_;
};

class InMemoryMembershipStore final : public IMembershipStore {
    public:
  /// Members need user details; @p users resolves them.
  explicit InMemoryMembershipStore(std::shared_ptr<IUserStore> users);

  std::optional<Role> role_of(std::string_view case_id,
                              std::int64_t user_id) const override;
  std::vector<Member> members(std::string_view case_id) const override;
  void set_role(std::string_view case_id, std::int64_t user_id,
                Role role) override;
  bool remove(std::string_view case_id, std::int64_t user_id) override;
  std::set<std::string> cases_of(std::int64_t user_id) const override;
  void forget_case(std::string_view case_id) override;

    private:
  std::shared_ptr<IUserStore> users_;
  mutable std::mutex mutex_;
  std::map<std::pair<std::string, std::int64_t>, Role> roles_;
};

class InMemoryAuditLog final : public IAuditLog {
    public:
  void record(const AuditEntry &entry) override;
  std::vector<AuditEntry> entries() const;

    private:
  mutable std::mutex mutex_;
  std::vector<AuditEntry> entries_;
};

/// Everything AuthService needs, in one bundle.
struct AuthStores {
  std::shared_ptr<IUserStore> users;
  std::shared_ptr<ISessionStore> sessions;
  std::shared_ptr<IApiTokenStore> tokens;
  std::shared_ptr<IMembershipStore> members;
  std::shared_ptr<IAuditLog> audit;
};

/// A fresh set of in-memory stores (tests).
AuthStores in_memory_auth_stores();

} // namespace ruxd::api
