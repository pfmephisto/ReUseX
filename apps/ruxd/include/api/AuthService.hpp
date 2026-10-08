// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Authentication for ruxd's server mode (spec 2026-10-08, phase S3): who a
// request is, logging in and out, and the account operations shared by the
// HTTP API and `ruxd admin`.
//
//  * A session is a random 32-byte token in an HttpOnly cookie; only its
//    SHA-256 is stored. It expires `idle_ttl` after its last use (renewed at
//    most every `renew_after`, so a busy tab is not a write per request) and
//    `max_lifetime` after login whatever happens. Logout deletes it.
//  * An API token (`Authorization: Bearer rxt_…`) is stored the same way and
//    acts as its user, optionally limited to one case.
//  * The `--auth-token` superuser token, when set, is accepted as a Bearer
//    token and may do everything (bootstrap, operations).
//  * Logins are rate limited per client IP and email (LoginRateLimiter), and
//    an unknown email costs the same argon2id work as a wrong password, so
//    the response time does not reveal which emails exist.
//
// Framework-free; tested in tests/unit/ruxd_api/test_api_auth.cpp.

#include "api/access.hpp"
#include "api/api.hpp"
#include "api/auth_stores.hpp"
#include "api/credentials.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace ruxd::api {

struct SessionOptions {
  /// A session unused this long expires.
  std::chrono::seconds idle_ttl{std::chrono::hours(12)};
  /// A session expires this long after login, used or not.
  std::chrono::seconds max_lifetime{std::chrono::hours(24 * 14)};
  /// Slide the expiry (a write) at most this often per session.
  std::chrono::seconds renew_after{std::chrono::minutes(5)};
};

struct AuthOptions {
  Argon2Params argon2;
  SessionOptions sessions;
  LoginRateLimitOptions rate_limit;
  /// The `--auth-token` superuser token; empty = none.
  std::string superuser_token;
  /// Shortest password accepted when one is set.
  std::size_t min_password_length = 8;
  /// Lifetime of a new API token when none is given (0 = never expires).
  std::chrono::seconds default_token_lifetime{std::chrono::hours(24 * 90)};
};

/// Shortest superuser token server mode accepts: it opens everything.
inline constexpr std::size_t kMinSuperuserTokenLength = 32;

/// What a request presented. Either may be empty.
struct PresentedCredentials {
  /// The token after `Authorization: Bearer `.
  std::string_view bearer;
  /// Every value of the session cookie (a browser may send more than one).
  std::vector<std::string_view> session_cookies;
  /// The client's address (client_address()), for the wrong-token back-off.
  std::string_view client_ip;
};

/// A login refused by the rate limiter: 429, with the wait for Retry-After.
class LoginRateLimited : public HttpError {
    public:
  explicit LoginRateLimited(std::chrono::seconds wait);
  std::chrono::seconds retry_after() const noexcept { return wait_; }

    private:
  std::chrono::seconds wait_;
};

/// The prefix of every API token, so a leaked one is recognisable.
inline constexpr std::string_view kApiTokenPrefix = "rxt_";

class AuthService {
    public:
  using ClockFn = std::function<SystemClock::time_point()>;

  /// @throws std::invalid_argument when a store is missing, or the
  ///         superuser token is shorter than kMinSuperuserTokenLength.
  AuthService(AuthStores stores, AuthOptions options = {},
              ClockFn clock = SystemClock::now);

  const AuthStores &stores() const noexcept { return stores_; }
  const AuthOptions &options() const noexcept { return options_; }

  /// Who @p credentials belong to; anonymous when nothing valid was
  /// presented. A valid session is renewed (sliding expiry) as a side effect.
  /// Wrong Bearer tokens count against the client address (back-off, as for
  /// failed logins); while it waits, every Bearer token is refused unread.
  Principal authenticate(const PresentedCredentials &credentials);

  struct LoginResult {
    std::string session_token; ///< The cookie value; shown once, never stored.
    User user;
    std::chrono::seconds max_age; ///< For the cookie's Max-Age.
  };

  /// Log in. @throws LoginRateLimited (429) when rate limited,
  /// HttpError(401) for an unknown email, a wrong password or a disabled
  /// account — one message for all three — and HttpError(400) for a
  /// malformed request.
  LoginResult login(std::string_view email, std::string_view password,
                    std::string_view client_ip);

  /// End @p who's session (no-op for anything but a session).
  void logout(const Principal &who);

  // --- accounts (HTTP admin routes and `ruxd admin`) -------------------------

  /// @throws HttpError(400) for a bad email or a short password,
  ///         HttpError(409) when the email is taken.
  User create_user(std::string_view email, std::string_view display_name,
                   std::string_view password, bool is_admin);
  /// Set a new password and end every session of the user.
  void set_password(std::int64_t user_id, std::string_view password);
  /// Disable (ending every session) or re-enable a user.
  void set_disabled(std::int64_t user_id, bool disabled);

  /// Create an API token for @p user_id, expiring after @p lifetime
  /// (nullopt: the default; 0: never). @return the token — shown once.
  std::string
  create_api_token(std::int64_t user_id, std::string_view name,
                   std::optional<std::string> case_id,
                   std::optional<std::chrono::seconds> lifetime = std::nullopt);

  /// Whether @p who still stands — its session or token still valid, its
  /// user still enabled. For long-lived connections (the events socket),
  /// re-checked on the registry's sweep.
  bool still_valid(const Principal &who) const;

  /// The role @p who has in @p case_id: owner for admins, the stored role
  /// for a member, nullopt otherwise (also for a token scoped elsewhere).
  std::optional<Role> role_in(const Principal &who,
                              std::string_view case_id) const;

  /// Write an audit entry for @p who (best effort: a failure is logged).
  void audit(const Principal &who, std::optional<std::string> case_id,
             std::string action, std::string detail = {}) const;

  /// Drop expired sessions (called on the registry's sweep). @return count.
  std::size_t purge_expired();

  /// Delete audit entries older than @p retention. @return count.
  std::size_t prune_audit(std::chrono::seconds retention);

    private:
  void check_password_policy(std::string_view password) const;

  AuthStores stores_;
  AuthOptions options_;
  ClockFn clock_;
  LoginRateLimiter limiter_;
  /// A hash of nothing in particular, verified against when the email is
  /// unknown so that case costs the same as a wrong password.
  std::string dummy_hash_;
};

} // namespace ruxd::api
