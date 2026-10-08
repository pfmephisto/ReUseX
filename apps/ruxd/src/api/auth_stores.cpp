// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "api/auth_stores.hpp"

#include "api/api.hpp"

#include <reusex/pipeline/job_types.hpp>

#include <algorithm>
#include <cctype>
#include <utility>

namespace ruxd::api {

std::string normalize_email(std::string_view email) {
  const auto first = email.find_first_not_of(" \t\r\n");
  if (first == std::string_view::npos)
    return {};
  const auto last = email.find_last_not_of(" \t\r\n");
  std::string out(email.substr(first, last - first + 1));
  std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return out;
}

bool is_plausible_email(std::string_view email) {
  if (email.size() < 3 || email.size() > 254)
    return false;
  const auto at = email.find('@');
  if (at == std::string_view::npos || at == 0 || at + 1 == email.size() ||
      email.find('@', at + 1) != std::string_view::npos)
    return false;
  return std::none_of(email.begin(), email.end(), [](unsigned char c) {
    return std::isspace(c) != 0 || std::iscntrl(c) != 0;
  });
}

// --- users
// ---------------------------------------------------------------------

InMemoryUserStore::Row &InMemoryUserStore::row(std::int64_t id) {
  auto it = rows_.find(id);
  if (it == rows_.end())
    throw HttpError(404, "no such user");
  return it->second;
}

User InMemoryUserStore::create(const std::string &email,
                               const std::string &display_name,
                               const std::string &password_hash,
                               bool is_admin) {
  const std::string key = normalize_email(email);
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto &[id, r] : rows_)
    if (r.user.email == key)
      throw HttpError(409, "a user with that email already exists");
  User user;
  user.id = next_id_++;
  user.email = key;
  user.display_name = display_name.empty() ? key : display_name;
  user.is_admin = is_admin;
  user.created_at = reusex::pipeline::iso8601_utc_now();
  rows_[user.id] = Row{user, password_hash};
  return user;
}

std::optional<User>
InMemoryUserStore::find_by_email(std::string_view email) const {
  const std::string key = normalize_email(email);
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto &[id, r] : rows_)
    if (r.user.email == key)
      return r.user;
  return std::nullopt;
}

std::optional<User> InMemoryUserStore::find_by_id(std::int64_t id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = rows_.find(id);
  if (it == rows_.end())
    return std::nullopt;
  return it->second.user;
}

std::vector<User> InMemoryUserStore::list() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<User> out;
  for (const auto &[id, r] : rows_)
    out.push_back(r.user);
  std::sort(out.begin(), out.end(),
            [](const User &a, const User &b) { return a.email < b.email; });
  return out;
}

std::optional<std::string>
InMemoryUserStore::password_hash(std::int64_t user_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = rows_.find(user_id);
  if (it == rows_.end())
    return std::nullopt;
  return it->second.hash;
}

void InMemoryUserStore::set_password_hash(std::int64_t user_id,
                                          const std::string &hash) {
  std::lock_guard<std::mutex> lock(mutex_);
  row(user_id).hash = hash;
}

void InMemoryUserStore::set_disabled(std::int64_t user_id, bool disabled) {
  std::lock_guard<std::mutex> lock(mutex_);
  row(user_id).user.disabled = disabled;
}

void InMemoryUserStore::set_admin(std::int64_t user_id, bool is_admin) {
  std::lock_guard<std::mutex> lock(mutex_);
  row(user_id).user.is_admin = is_admin;
}

void InMemoryUserStore::set_display_name(std::int64_t user_id,
                                         const std::string &name) {
  std::lock_guard<std::mutex> lock(mutex_);
  row(user_id).user.display_name = name;
}

// --- sessions
// ------------------------------------------------------------------

void InMemorySessionStore::create(const SessionRecord &session) {
  std::lock_guard<std::mutex> lock(mutex_);
  sessions_[session.token_hash] = session;
}

std::optional<SessionRecord>
InMemorySessionStore::find(std::string_view token_hash) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = sessions_.find(token_hash);
  if (it == sessions_.end())
    return std::nullopt;
  return it->second;
}

void InMemorySessionStore::renew(std::string_view token_hash,
                                 SystemClock::time_point expires_at,
                                 SystemClock::time_point last_seen) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = sessions_.find(token_hash);
  if (it != sessions_.end()) {
    it->second.expires_at = expires_at;
    it->second.last_seen = last_seen;
  }
}

void InMemorySessionStore::revoke(std::string_view token_hash) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (auto it = sessions_.find(token_hash); it != sessions_.end())
    sessions_.erase(it);
}

void InMemorySessionStore::revoke_user(std::int64_t user_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::erase_if(sessions_,
                [&](const auto &kv) { return kv.second.user_id == user_id; });
}

std::size_t InMemorySessionStore::purge_expired(SystemClock::time_point now) {
  std::lock_guard<std::mutex> lock(mutex_);
  return std::erase_if(
      sessions_, [&](const auto &kv) { return kv.second.expires_at <= now; });
}

// --- API tokens
// ----------------------------------------------------------------

ApiTokenRecord InMemoryApiTokenStore::create(ApiTokenRecord token) {
  std::lock_guard<std::mutex> lock(mutex_);
  token.id = next_id_++;
  tokens_[token.token_hash] = token;
  return token;
}

std::optional<ApiTokenRecord>
InMemoryApiTokenStore::find(std::string_view token_hash) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = tokens_.find(token_hash);
  if (it == tokens_.end())
    return std::nullopt;
  return it->second;
}

// --- membership
// ----------------------------------------------------------------

InMemoryMembershipStore::InMemoryMembershipStore(
    std::shared_ptr<IUserStore> users)
    : users_(std::move(users)) {}

std::optional<Role>
InMemoryMembershipStore::role_of(std::string_view case_id,
                                 std::int64_t user_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = roles_.find({std::string(case_id), user_id});
  if (it == roles_.end())
    return std::nullopt;
  return it->second;
}

std::vector<Member>
InMemoryMembershipStore::members(std::string_view case_id) const {
  std::vector<std::pair<std::int64_t, Role>> rows;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto &[key, role] : roles_)
      if (key.first == case_id)
        rows.emplace_back(key.second, role);
  }
  std::vector<Member> out;
  for (const auto &[id, role] : rows)
    if (auto user = users_->find_by_id(id))
      out.push_back(Member{*user, role});
  std::sort(out.begin(), out.end(), [](const Member &a, const Member &b) {
    return a.user.email < b.user.email;
  });
  return out;
}

void InMemoryMembershipStore::set_role(std::string_view case_id,
                                       std::int64_t user_id, Role role) {
  std::lock_guard<std::mutex> lock(mutex_);
  roles_[{std::string(case_id), user_id}] = role;
}

bool InMemoryMembershipStore::remove(std::string_view case_id,
                                     std::int64_t user_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  return roles_.erase({std::string(case_id), user_id}) > 0;
}

std::set<std::string>
InMemoryMembershipStore::cases_of(std::int64_t user_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::set<std::string> out;
  for (const auto &[key, role] : roles_)
    if (key.second == user_id)
      out.insert(key.first);
  return out;
}

void InMemoryMembershipStore::forget_case(std::string_view case_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::erase_if(roles_,
                [&](const auto &kv) { return kv.first.first == case_id; });
}

// --- audit
// ---------------------------------------------------------------------

void InMemoryAuditLog::record(const AuditEntry &entry) {
  std::lock_guard<std::mutex> lock(mutex_);
  entries_.push_back(entry);
}

std::vector<AuditEntry> InMemoryAuditLog::entries() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return entries_;
}

AuthStores in_memory_auth_stores() {
  AuthStores stores;
  stores.users = std::make_shared<InMemoryUserStore>();
  stores.sessions = std::make_shared<InMemorySessionStore>();
  stores.tokens = std::make_shared<InMemoryApiTokenStore>();
  stores.members = std::make_shared<InMemoryMembershipStore>(stores.users);
  stores.audit = std::make_shared<InMemoryAuditLog>();
  return stores;
}

} // namespace ruxd::api
