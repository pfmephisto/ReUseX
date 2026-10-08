// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Who may do what (spec 2026-10-08, phase S3): case roles, the principal a
// request authenticated as, the classification of every route the server
// serves, and the login rate limiter.
//
// The rules, in one place:
//  * viewer: GET only. editor: everything in the case except deleting it and
//    managing its members. owner: everything in the case. An admin (or the
//    `--auth-token` superuser, or local mode's implicit user) may do
//    everything everywhere, including user management.
//  * Every case route checks membership. A missing case and a case the caller
//    is not a member of both answer 404, so cases cannot be enumerated.
//
// Framework-free; tested in tests/unit/ruxd_api/test_api_access.cpp.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace ruxd::api {

/// A member's role in one case. Ordered: each role can do what the ones
/// before it can.
enum class Role { viewer, editor, owner };

std::string_view to_string(Role role);
/// The role named @p name ("viewer", "editor", "owner"), or nullopt.
std::optional<Role> parse_role(std::string_view name);

/// What a request does to a case.
enum class CaseOp {
  read,           ///< GET (and HEAD): viewers may.
  write,          ///< Any other method on the case or its contents.
  delete_case,    ///< DELETE /cases/{cid}: owners only.
  manage_members, ///< Changing the member list: owners only.
};

/// The role matrix.
bool role_allows(Role role, CaseOp op);

/// How a route is protected.
struct RouteAccess {
  enum class Kind {
    public_route,  ///< Anyone: health, readiness, login, the route table.
    static_file,   ///< The frontend bundle: public in server mode (the login
                   ///< page must load), token-protected in local mode.
    authenticated, ///< Any signed-in principal (me, logout, case list, SAM3
                   ///< status, uploads).
    create_case,   ///< Creates a case (POST /cases, uploads): any signed-in
                   ///< user; not a case-scoped API token.
    admin,         ///< User management.
    case_route,    ///< Under /cases/{cid}: membership + role_allows(op).
  };
  Kind kind = Kind::authenticated;
  std::string cid; ///< case_route only.
  CaseOp op = CaseOp::read;
};

/// Classify @p method + @p path (no query). Unknown `/api/...` paths are
/// `authenticated` (the router then answers 404); paths outside `/api/` are
/// static files.
RouteAccess classify_route(std::string_view method, std::string_view path);

/// True for methods that never change anything (GET, HEAD, OPTIONS).
bool is_safe_method(std::string_view method);

/// A user account.
struct User {
  std::int64_t id = 0;
  std::string email;
  std::string display_name;
  bool is_admin = false;
  bool disabled = false;
  std::string created_at; ///< ISO-8601 UTC.
};

/// How a request authenticated.
enum class PrincipalKind {
  anonymous, ///< Nothing presented, or nothing valid.
  local,     ///< `ruxd --local`: the implicit user, owner of every case.
  superuser, ///< `--auth-token` in server mode: everything, as nobody.
  session,   ///< A login session cookie.
  api_token, ///< `Authorization: Bearer` with a stored API token.
};

std::string_view to_string(PrincipalKind kind);

/// Who a request is.
struct Principal {
  PrincipalKind kind = PrincipalKind::anonymous;
  /// The user (for local mode and the superuser, a synthetic one with id 0).
  User user;
  /// An API token limited to one case: only that case's routes (and `me`).
  std::optional<std::string> case_scope;
  /// The session's token hash, for logout and renewal (session only).
  std::string session_hash;
  /// The API token's hash (api_token only), to re-check it later.
  std::string token_hash;

  bool authenticated() const noexcept {
    return kind != PrincipalKind::anonymous;
  }
  /// May do everything everywhere (admin user, superuser, local).
  bool is_admin() const noexcept {
    return kind == PrincipalKind::local || kind == PrincipalKind::superuser ||
           (authenticated() && user.is_admin);
  }
  /// The user id for audit and membership; nullopt for local and superuser.
  std::optional<std::int64_t> user_id() const noexcept {
    if (kind == PrincipalKind::session || kind == PrincipalKind::api_token)
      return user.id;
    return std::nullopt;
  }
};

/// The synthetic user of local mode.
Principal local_principal();
/// The synthetic user behind the server-mode `--auth-token`.
Principal superuser_principal();

/// The decision for one request: 0 = allowed, else the HTTP status to refuse
/// it with and why.
struct AccessDecision {
  int status = 0;
  std::string message;
  bool allowed() const noexcept { return status == 0; }
};

/// Decide @p route for @p who, whose role in the route's case is @p role
/// (nullopt: not a member, or the case does not exist). Membership is the
/// caller's lookup; this is the pure matrix.
///  * anonymous on anything but public/static: 401;
///  * a non-member (or a scoped token on another case): 404;
///  * a member whose role does not allow the op: 403;
///  * admin routes without admin: 403 (404 would hide nothing: the route
///    table is public).
AccessDecision decide_access(const Principal &who, const RouteAccess &route,
                             std::optional<Role> role);

// --- client addresses
// ---------------------------------------------------------

/// Whether @p ip (an IPv4 or IPv6 literal) lies in @p cidr ("10.0.0.0/8",
/// "::1/128", or a bare address = a single host). Malformed input: false.
bool cidr_contains(std::string_view cidr, std::string_view ip);

/// Whether @p cidr parses as an IPv4/IPv6 network or address.
bool is_valid_cidr(std::string_view cidr);

/// The client a request comes from. @p peer is the TCP peer; when it is one
/// of @p trusted_proxies, `X-Forwarded-For` (@p forwarded_for) is honoured:
/// the right-most address that is not itself a trusted proxy. From anyone
/// else the header is ignored (it is trivially forged).
std::string client_address(std::string_view peer,
                           std::string_view forwarded_for,
                           const std::vector<std::string> &trusted_proxies);

/// The rate-limit key of an address: an IPv6 address counts as its /64 (one
/// subscriber's allocation — rotating within it must not reset the limit),
/// an IPv4 (or IPv4-mapped IPv6) address as itself.
std::string rate_limit_key(std::string_view ip);

// --- login rate limit --------------------------------------------------------

struct LoginRateLimitOptions {
  /// Failed logins an account may have before each further attempt waits.
  std::size_t free_failures_per_account = 5;
  /// The same for one client address (an IPv6 /64), any account: a spray.
  std::size_t free_failures_per_address = 20;
  /// The first wait once the free failures are used; it doubles with every
  /// further failure (exponential back-off) …
  std::chrono::seconds base_delay{1};
  /// … up to this.
  std::chrono::seconds max_delay{std::chrono::minutes(15)};
  /// Failures are forgotten after this long without another.
  std::chrono::seconds forget_after{std::chrono::minutes(30)};
  /// Keys tracked at once; the stalest are dropped past it, so a flood of
  /// distinct emails cannot grow memory without bound.
  std::size_t max_keys = 10000;
};

/// Back-off for failed logins (and wrong Bearer tokens), in memory,
/// thread-safe. Only FAILURES count; a successful login clears its account's
/// failures. Past the free failures each attempt must wait base_delay ·
/// 2^(failures − free), capped at max_delay — so a guesser is slowed to a
/// crawl, while a locked-out user is never blocked for longer than max_delay.
class LoginRateLimiter {
    public:
  using Clock = std::chrono::steady_clock;

  explicit LoginRateLimiter(LoginRateLimitOptions options = {});

  /// 0 when a login for @p email from @p address_key (rate_limit_key) may
  /// be tried at @p now, else the seconds to wait (for Retry-After).
  std::chrono::seconds wait(std::string_view address_key,
                            std::string_view email,
                            Clock::time_point now = Clock::now());
  /// Record a failed login.
  void failure(std::string_view address_key, std::string_view email,
               Clock::time_point now = Clock::now());
  /// A successful login: the account's failures are forgotten.
  void success(std::string_view email);

  /// The address-only half, for wrong Bearer tokens.
  std::chrono::seconds wait_address(std::string_view address_key,
                                    Clock::time_point now = Clock::now());
  void failure_address(std::string_view address_key,
                       Clock::time_point now = Clock::now());

  const LoginRateLimitOptions &options() const noexcept { return options_; }

    private:
  struct Failures {
    std::size_t count = 0;
    Clock::time_point last;
  };
  std::chrono::seconds wait_locked(const std::string &key, std::size_t free,
                                   Clock::time_point now);
  void fail_locked(const std::string &key, Clock::time_point now);
  void prune(Clock::time_point now);

  LoginRateLimitOptions options_;
  std::mutex mutex_;
  std::map<std::string, Failures, std::less<>> failures_;
};

} // namespace ruxd::api
