// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "api/AuthService.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace ruxd::api {

namespace {
constexpr std::size_t kMaxPasswordLength = 1024;
constexpr std::size_t kMaxDisplayName = 200;
constexpr std::size_t kMaxTokenName = 200;
constexpr std::string_view kBadLogin = "wrong email or password";
} // namespace

LoginRateLimited::LoginRateLimited(std::chrono::seconds wait)
    : HttpError(429, "too many login attempts; try again in " +
                         std::to_string(wait.count()) + " s"),
      wait_(wait) {}

AuthService::AuthService(AuthStores stores, AuthOptions options, ClockFn clock)
    : stores_(std::move(stores)), options_(std::move(options)),
      clock_(std::move(clock)), limiter_(options_.rate_limit) {
  if (!stores_.users || !stores_.sessions || !stores_.tokens ||
      !stores_.members || !stores_.audit)
    throw std::invalid_argument("AuthService needs every store");
  if (!options_.superuser_token.empty() &&
      options_.superuser_token.size() < kMinSuperuserTokenLength)
    throw std::invalid_argument(
        "the superuser token (--auth-token) must be at least " +
        std::to_string(kMinSuperuserTokenLength) +
        " characters: it opens everything (try `openssl rand -hex 32`)");
  dummy_hash_ = hash_password(random_token(16), options_.argon2);
}

Principal AuthService::authenticate(const PresentedCredentials &credentials) {
  Principal out;
  const auto now = clock_();

  if (!credentials.bearer.empty()) {
    const std::string address = rate_limit_key(credentials.client_ip);
    // An address that keeps presenting wrong tokens waits, like a login.
    if (limiter_.wait_address(address).count() > 0)
      return out;
    if (!options_.superuser_token.empty() &&
        secure_equals(credentials.bearer, options_.superuser_token))
      return superuser_principal();
    const std::string hash = sha256_hex(credentials.bearer);
    if (const auto token = stores_.tokens->find(hash);
        token && (!token->expires_at || *token->expires_at > now))
      if (auto user = stores_.users->find_by_id(token->user_id);
          user && !user->disabled) {
        if (!token->last_used_at ||
            now - *token->last_used_at >= std::chrono::minutes(1))
          stores_.tokens->touch(token->id, now);
        out.kind = PrincipalKind::api_token;
        out.user = std::move(*user);
        out.case_scope = token->case_id;
        out.token_hash = hash;
        return out;
      }
    limiter_.failure_address(address);
    // A wrong Bearer does not fall through to the cookie: a script that
    // presents a token means that token.
    return out;
  }

  for (const auto cookie : credentials.session_cookies) {
    if (cookie.empty())
      continue;
    const std::string hash = sha256_hex(cookie);
    const auto session = stores_.sessions->find(hash);
    if (!session)
      continue;
    const auto hard_end = session->created_at + options_.sessions.max_lifetime;
    if (session->expires_at <= now || hard_end <= now) {
      stores_.sessions->revoke(hash);
      continue;
    }
    auto user = stores_.users->find_by_id(session->user_id);
    if (!user || user->disabled) {
      stores_.sessions->revoke(hash);
      continue;
    }
    // Sliding expiry, written at most every renew_after.
    if (now - session->last_seen >= options_.sessions.renew_after)
      stores_.sessions->renew(
          hash, std::min(now + options_.sessions.idle_ttl, hard_end), now);
    out.kind = PrincipalKind::session;
    out.user = std::move(*user);
    out.session_hash = hash;
    return out;
  }
  return out;
}

AuthService::LoginResult AuthService::login(std::string_view email,
                                            std::string_view password,
                                            std::string_view client_ip) {
  const std::string key = normalize_email(email);
  if (key.empty() || password.empty())
    throw HttpError(400, "'email' and 'password' are required");
  if (password.size() > kMaxPasswordLength)
    throw HttpError(400, "password too long");

  const std::string address = rate_limit_key(client_ip);
  if (const auto wait = limiter_.wait(address, key); wait.count() > 0) {
    spdlog::warn("Login back-off for {} from {}: {} s", key, client_ip,
                 wait.count());
    throw LoginRateLimited(wait);
  }

  // One lookup whether or not the email exists, and the same argon2id work:
  // an unknown email is verified against a dummy hash, then refused.
  const auto found = stores_.users->find_credentials(key);
  const std::optional<User> user =
      found ? std::optional<User>(found->first) : std::nullopt;
  const std::optional<std::string> stored =
      found ? std::optional<std::string>(found->second) : std::nullopt;
  const bool ok = verify_password(password, stored ? *stored : dummy_hash_) &&
                  stored.has_value();
  if (!ok || !user || user->disabled) {
    limiter_.failure(address, key);
    spdlog::info("Failed login for {} from {}", key, client_ip);
    AuditEntry entry;
    entry.user_id = user ? std::optional<std::int64_t>(user->id) : std::nullopt;
    entry.actor = key;
    entry.action = "auth.login_failed";
    entry.detail = std::string(client_ip);
    try {
      stores_.audit->record(entry);
    } catch (const std::exception &e) {
      spdlog::warn("Audit write failed: {}", e.what());
    }
    throw HttpError(401, std::string(kBadLogin));
  }

  limiter_.success(key);

  // Parameters raised since this password was set: store a fresh hash.
  if (password_needs_rehash(*stored, options_.argon2))
    stores_.users->set_password_hash(user->id,
                                     hash_password(password, options_.argon2));

  const auto now = clock_();
  LoginResult out;
  out.session_token = random_token(32);
  out.user = *user;
  out.max_age = options_.sessions.max_lifetime;
  SessionRecord session;
  session.token_hash = sha256_hex(out.session_token);
  session.user_id = user->id;
  session.created_at = now;
  session.last_seen = now;
  session.expires_at = now + std::min(options_.sessions.idle_ttl,
                                      options_.sessions.max_lifetime);
  stores_.sessions->create(session);

  Principal who;
  who.kind = PrincipalKind::session;
  who.user = *user;
  audit(who, std::nullopt, "auth.login", std::string(client_ip));
  return out;
}

void AuthService::logout(const Principal &who) {
  if (who.kind != PrincipalKind::session || who.session_hash.empty())
    return;
  stores_.sessions->revoke(who.session_hash);
  audit(who, std::nullopt, "auth.logout");
}

void AuthService::check_password_policy(std::string_view password) const {
  if (password.size() < options_.min_password_length)
    throw HttpError(400, "the password must be at least " +
                             std::to_string(options_.min_password_length) +
                             " characters");
  if (password.size() > kMaxPasswordLength)
    throw HttpError(400, "password too long");
}

User AuthService::create_user(std::string_view email,
                              std::string_view display_name,
                              std::string_view password, bool is_admin) {
  const std::string key = normalize_email(email);
  if (!is_plausible_email(key))
    throw HttpError(400, "'" + key + "' is not an email address");
  if (display_name.size() > kMaxDisplayName)
    throw HttpError(400, "display name too long");
  check_password_policy(password);
  return stores_.users->create(key, std::string(display_name),
                               hash_password(password, options_.argon2),
                               is_admin);
}

void AuthService::set_password(std::int64_t user_id,
                               std::string_view password) {
  check_password_policy(password);
  stores_.users->set_password_hash(user_id,
                                   hash_password(password, options_.argon2));
  stores_.sessions->revoke_user(user_id);
}

void AuthService::set_disabled(std::int64_t user_id, bool disabled) {
  stores_.users->set_disabled(user_id, disabled);
  if (disabled)
    stores_.sessions->revoke_user(user_id);
}

std::string
AuthService::create_api_token(std::int64_t user_id, std::string_view name,
                              std::optional<std::string> case_id,
                              std::optional<std::chrono::seconds> lifetime) {
  if (name.empty() || name.size() > kMaxTokenName)
    throw HttpError(400, "a token needs a name of 1-200 characters");
  if (!stores_.users->find_by_id(user_id))
    throw HttpError(404, "no such user");
  std::string token = std::string(kApiTokenPrefix) + random_token(32);
  ApiTokenRecord record;
  record.token_hash = sha256_hex(token);
  record.user_id = user_id;
  record.name = std::string(name);
  record.case_id = std::move(case_id);
  record.created_at = clock_();
  const auto life = lifetime.value_or(options_.default_token_lifetime);
  if (life.count() > 0)
    record.expires_at = record.created_at + life;
  stores_.tokens->create(std::move(record));
  return token;
}

std::optional<Role> AuthService::role_in(const Principal &who,
                                         std::string_view case_id) const {
  if (who.case_scope && *who.case_scope != case_id)
    return std::nullopt;
  if (who.is_admin())
    return Role::owner;
  if (const auto id = who.user_id())
    return stores_.members->role_of(case_id, *id);
  return std::nullopt;
}

void AuthService::audit(const Principal &who,
                        std::optional<std::string> case_id, std::string action,
                        std::string detail) const {
  AuditEntry entry;
  entry.user_id = who.user_id();
  entry.actor = who.user.email.empty() ? std::string(to_string(who.kind))
                                       : who.user.email;
  entry.case_id = std::move(case_id);
  entry.action = std::move(action);
  entry.detail = std::move(detail);
  try {
    stores_.audit->record(entry);
  } catch (const std::exception &e) {
    spdlog::warn("Audit write failed ({}): {}", entry.action, e.what());
  }
}

bool AuthService::still_valid(const Principal &who) const {
  const auto now = clock_();
  switch (who.kind) {
  case PrincipalKind::local:
  case PrincipalKind::superuser:
    return true;
  case PrincipalKind::anonymous:
    return false;
  case PrincipalKind::session: {
    const auto session = stores_.sessions->find(who.session_hash);
    if (!session || session->expires_at <= now ||
        session->created_at + options_.sessions.max_lifetime <= now)
      return false;
    const auto user = stores_.users->find_by_id(session->user_id);
    return user && !user->disabled;
  }
  case PrincipalKind::api_token: {
    const auto token = stores_.tokens->find(who.token_hash);
    if (!token || (token->expires_at && *token->expires_at <= now))
      return false;
    const auto user = stores_.users->find_by_id(token->user_id);
    return user && !user->disabled;
  }
  }
  return false;
}

std::size_t AuthService::prune_audit(std::chrono::seconds retention) {
  if (retention.count() <= 0)
    return 0;
  return stores_.audit->prune(clock_() - retention);
}

std::size_t AuthService::purge_expired() {
  return stores_.sessions->purge_expired(clock_());
}

} // namespace ruxd::api
