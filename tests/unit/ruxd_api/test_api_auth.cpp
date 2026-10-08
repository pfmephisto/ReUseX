// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// AuthService over the in-memory stores (ruxd server mode, spec 2026-10-08
// S3): login, sessions with sliding and absolute expiry, logout, API tokens,
// the superuser token, the login rate limit and account changes. A fake clock
// drives the expiry.

#include <catch2/catch_test_macros.hpp>

#include <api/AuthService.hpp>

#include <chrono>
#include <functional>
#include <string>

using namespace ruxd::api;
using namespace std::chrono_literals;

namespace {

struct Fixture {
  SystemClock::time_point now = SystemClock::time_point{} + 24h * 20000;
  AuthStores stores = in_memory_auth_stores();
  std::shared_ptr<AuthService> auth;

  explicit Fixture(std::string superuser = {}) {
    AuthOptions options;
    options.argon2.iterations = 1;
    options.argon2.memory_kib = 256;
    options.argon2.lanes = 1;
    options.superuser_token = std::move(superuser);
    auth =
        std::make_shared<AuthService>(stores, options, [this] { return now; });
  }

  Principal with_cookie(const std::string &token) {
    PresentedCredentials c;
    c.session_cookies.push_back(token);
    return auth->authenticate(c);
  }
  Principal with_bearer(const std::string &token) {
    PresentedCredentials c;
    c.bearer = token;
    return auth->authenticate(c);
  }
};

int status_of(const std::function<void()> &fn) {
  try {
    fn();
  } catch (const HttpError &e) {
    return e.status();
  }
  return 0;
}

} // namespace

TEST_CASE("AuthService_Login_StartsASessionStoredOnlyAsAHash",
          "[ruxd_api][auth]") {
  Fixture f;
  const auto anna =
      f.auth->create_user("Anna@Example.dk ", "Anna", "hemmeligt1", false);
  CHECK(anna.email == "anna@example.dk");
  // The stored password is an argon2id hash, never the password.
  const auto stored = f.stores.users->password_hash(anna.id);
  REQUIRE(stored);
  CHECK(stored->rfind("$argon2id$", 0) == 0);
  CHECK(stored->find("hemmeligt1") == std::string::npos);

  const auto login = f.auth->login("ANNA@example.dk", "hemmeligt1", "1.2.3.4");
  CHECK(login.user.id == anna.id);
  CHECK(login.session_token.size() == 43);
  // Only the digest is stored.
  CHECK_FALSE(f.stores.sessions->find(login.session_token));
  CHECK(f.stores.sessions->find(sha256_hex(login.session_token)));

  const auto who = f.with_cookie(login.session_token);
  CHECK(who.kind == PrincipalKind::session);
  CHECK(who.user.id == anna.id);
  CHECK(who.user_id() == anna.id);
  CHECK_FALSE(who.is_admin());

  // Garbage cookies (or several, one good) authenticate as expected.
  CHECK(f.with_cookie("nonsense").kind == PrincipalKind::anonymous);
  PresentedCredentials both;
  both.session_cookies = {"stale", login.session_token};
  CHECK(f.auth->authenticate(both).kind == PrincipalKind::session);
}

TEST_CASE("AuthService_Login_WrongUnknownOrDisabledIs401", "[ruxd_api][auth]") {
  Fixture f;
  const auto bo = f.auth->create_user("bo@example.dk", "", "hemmeligt1", false);
  CHECK(bo.display_name == "bo@example.dk");
  CHECK(status_of([&] { f.auth->login("bo@example.dk", "forkert!!", "ip"); }) ==
        401);
  CHECK(status_of([&] { f.auth->login("ingen@example.dk", "x", "ip2"); }) ==
        401);
  f.auth->set_disabled(bo.id, true);
  CHECK(status_of([&] {
          f.auth->login("bo@example.dk", "hemmeligt1", "ip3");
        }) == 401);
  CHECK(status_of([&] { f.auth->login("", "x", "ip"); }) == 400);
}

TEST_CASE("AuthService_Login_SuccessDoesNotCountAndResets",
          "[ruxd_api][auth][ratelimit]") {
  Fixture f;
  f.auth->create_user("d@example.dk", "D", "hemmeligt1", false);
  // Successful logins never count against the limit…
  for (int i = 0; i < 12; ++i)
    CHECK(f.auth->login("d@example.dk", "hemmeligt1", "9.9.9.8").user.email ==
          "d@example.dk");
  // …and one clears the failures before it.
  for (int round = 0; round < 3; ++round) {
    for (int i = 0; i < 4; ++i)
      CHECK(status_of([&] {
              f.auth->login("d@example.dk", "nej", "9.9.9.8");
            }) == 401);
    CHECK_NOTHROW(f.auth->login("d@example.dk", "hemmeligt1", "9.9.9.8"));
  }
}

TEST_CASE("AuthService_Login_RateLimitedAfterFiveAttempts",
          "[ruxd_api][auth][ratelimit]") {
  Fixture f;
  f.auth->create_user("c@example.dk", "C", "hemmeligt1", false);
  for (int i = 0; i < 5; ++i)
    CHECK(status_of([&] { f.auth->login("c@example.dk", "nej", "9.9.9.9"); }) ==
          401);
  // Even the right password is refused now, with a Retry-After.
  try {
    f.auth->login("c@example.dk", "hemmeligt1", "9.9.9.9");
    FAIL("expected 429");
  } catch (const LoginRateLimited &e) {
    CHECK(e.status() == 429);
    CHECK(e.retry_after().count() > 0);
  }
  // The back-off is per account: another address waits too (an attacker
  // rotating addresses gains nothing).
  CHECK(status_of([&] {
          f.auth->login("c@example.dk", "hemmeligt1", "2001:db8::1");
        }) == 429);
}

TEST_CASE("AuthService_Session_SlidingAndAbsoluteExpiry", "[ruxd_api][auth]") {
  Fixture f;
  f.auth->create_user("d@example.dk", "D", "hemmeligt1", false);
  const auto token =
      f.auth->login("d@example.dk", "hemmeligt1", "ip").session_token;
  const auto &opt = f.auth->options().sessions;
  const auto hash = sha256_hex(token);

  // Idle for less than idle_ttl: still valid, and the expiry slides.
  const auto first_expiry = f.stores.sessions->find(hash)->expires_at;
  f.now += opt.idle_ttl - 1h;
  CHECK(f.with_cookie(token).authenticated());
  CHECK(f.stores.sessions->find(hash)->expires_at > first_expiry);

  // Used again well within renew_after: no write (expiry unchanged).
  const auto slid = f.stores.sessions->find(hash)->expires_at;
  f.now += 1min;
  CHECK(f.with_cookie(token).authenticated());
  CHECK(f.stores.sessions->find(hash)->expires_at == slid);

  // Idle past idle_ttl: gone, and deleted.
  f.now += opt.idle_ttl + 1s;
  CHECK_FALSE(f.with_cookie(token).authenticated());
  CHECK_FALSE(f.stores.sessions->find(hash));

  // Kept busy, a session still ends max_lifetime after login.
  const auto busy =
      f.auth->login("d@example.dk", "hemmeligt1", "ip2").session_token;
  const auto start = f.now;
  while (f.now < start + opt.max_lifetime - 1h) {
    f.now += opt.idle_ttl / 2;
    if (f.now < start + opt.max_lifetime)
      CHECK(f.with_cookie(busy).authenticated());
  }
  f.now = start + opt.max_lifetime + 1s;
  CHECK_FALSE(f.with_cookie(busy).authenticated());
}

TEST_CASE("AuthService_Logout_RevokesTheSession", "[ruxd_api][auth]") {
  Fixture f;
  f.auth->create_user("e@example.dk", "E", "hemmeligt1", false);
  const auto token =
      f.auth->login("e@example.dk", "hemmeligt1", "ip").session_token;
  const auto who = f.with_cookie(token);
  REQUIRE(who.authenticated());
  f.auth->logout(who);
  CHECK_FALSE(f.with_cookie(token).authenticated());
}

TEST_CASE("AuthService_PasswordChangeOrDisable_EndsSessions",
          "[ruxd_api][auth]") {
  Fixture f;
  const auto u = f.auth->create_user("f@example.dk", "F", "hemmeligt1", false);
  const auto a =
      f.auth->login("f@example.dk", "hemmeligt1", "ip").session_token;
  f.auth->set_password(u.id, "nyt-kodeord-2");
  CHECK_FALSE(f.with_cookie(a).authenticated());
  CHECK(status_of([&] { f.auth->login("f@example.dk", "hemmeligt1", "ip"); }) ==
        401);
  const auto b =
      f.auth->login("f@example.dk", "nyt-kodeord-2", "ip").session_token;
  f.auth->set_disabled(u.id, true);
  CHECK_FALSE(f.with_cookie(b).authenticated());
  // Short passwords are refused.
  CHECK(status_of([&] { f.auth->set_password(u.id, "kort"); }) == 400);
  CHECK(status_of([&] {
          f.auth->create_user("bad", "x", "hemmeligt1", false);
        }) == 400);
  CHECK(status_of([&] {
          f.auth->create_user("f@example.dk", "x", "hemmeligt1", false);
        }) == 409);
}

TEST_CASE("AuthService_ApiTokenAndSuperuser", "[ruxd_api][auth]") {
  Fixture f("super-secret-token-0123456789abcdef");
  const auto u = f.auth->create_user("g@example.dk", "G", "hemmeligt1", false);

  const auto token = f.auth->create_api_token(u.id, "ci", std::string("k"));
  CHECK(token.rfind(std::string(kApiTokenPrefix), 0) == 0);
  const auto who = f.with_bearer(token);
  CHECK(who.kind == PrincipalKind::api_token);
  CHECK(who.user.id == u.id);
  REQUIRE(who.case_scope);
  CHECK(*who.case_scope == "k");
  // Stored as a digest only.
  CHECK_FALSE(f.stores.tokens->find(token));

  CHECK(f.with_bearer("rxt_wrong").kind == PrincipalKind::anonymous);
  CHECK(f.with_bearer("super-secret-token-0123456789abcdef").kind ==
        PrincipalKind::superuser);
  CHECK(f.with_bearer("super-secret-tokem").kind == PrincipalKind::anonymous);

  // A disabled user's tokens stop working.
  f.auth->set_disabled(u.id, true);
  CHECK(f.with_bearer(token).kind == PrincipalKind::anonymous);
}

TEST_CASE("AuthService_ShortSuperuserToken_Refused", "[ruxd_api][auth]") {
  AuthOptions options;
  options.superuser_token = "too-short";
  CHECK_THROWS_AS(AuthService(in_memory_auth_stores(), options),
                  std::invalid_argument);
}

TEST_CASE("AuthService_ApiTokens_ExpireRevokeAndList", "[ruxd_api][auth]") {
  Fixture f;
  const auto u = f.auth->create_user("t@example.dk", "T", "hemmeligt1", false);
  const auto day = f.auth->create_api_token(u.id, "kort", std::nullopt,
                                            std::chrono::hours(24));
  const auto never = f.auth->create_api_token(u.id, "evig", std::nullopt,
                                              std::chrono::seconds(0));
  const auto defaulted = f.auth->create_api_token(u.id, "std", std::nullopt);
  auto tokens = f.stores.tokens->list(u.id);
  REQUIRE(tokens.size() == 3);
  CHECK(tokens[0].expires_at);
  CHECK_FALSE(tokens[1].expires_at);
  CHECK(tokens[2].expires_at); // the default lifetime
  CHECK(f.with_bearer(day).kind == PrincipalKind::api_token);
  // Its last use is recorded.
  CHECK(f.stores.tokens->list(u.id)[0].last_used_at);

  // Expired: refused.
  f.now += std::chrono::hours(25);
  CHECK(f.with_bearer(day).kind == PrincipalKind::anonymous);
  CHECK(f.with_bearer(never).kind == PrincipalKind::api_token);

  // Revoked: refused; someone else cannot revoke it.
  const auto id = tokens[1].id;
  CHECK_FALSE(f.stores.tokens->revoke(id, u.id + 99));
  CHECK(f.stores.tokens->revoke(id, u.id));
  CHECK(f.with_bearer(never).kind == PrincipalKind::anonymous);
  (void)defaulted;
}

TEST_CASE("AuthService_WrongBearerTokens_BackOff",
          "[ruxd_api][auth][ratelimit]") {
  Fixture f;
  const auto u = f.auth->create_user("w@example.dk", "W", "hemmeligt1", false);
  const auto good = f.auth->create_api_token(u.id, "ci", std::nullopt);
  PresentedCredentials c;
  c.client_ip = "198.51.100.7";
  c.bearer = "rxt_guess";
  for (int i = 0; i < 25; ++i)
    CHECK(f.auth->authenticate(c).kind == PrincipalKind::anonymous);
  // That address now waits: even a right token is not looked at.
  c.bearer = good;
  CHECK(f.auth->authenticate(c).kind == PrincipalKind::anonymous);
  c.client_ip = "198.51.100.8";
  CHECK(f.auth->authenticate(c).kind == PrincipalKind::api_token);
}

TEST_CASE("AuthService_StillValid_FollowsSessionsTokensAndUsers",
          "[ruxd_api][auth]") {
  Fixture f;
  const auto u = f.auth->create_user("v@example.dk", "V", "hemmeligt1", false);
  const auto login = f.auth->login("v@example.dk", "hemmeligt1", "ip");
  const auto who = f.with_cookie(login.session_token);
  CHECK(f.auth->still_valid(who));
  const auto token = f.with_bearer(f.auth->create_api_token(u.id, "x", {}));
  CHECK(f.auth->still_valid(token));
  f.auth->set_disabled(u.id, true);
  CHECK_FALSE(f.auth->still_valid(who));
  CHECK_FALSE(f.auth->still_valid(token));
  CHECK(f.auth->still_valid(local_principal()));
}

TEST_CASE("AuthService_RoleIn_MembersAdminsAndScopes", "[ruxd_api][auth]") {
  Fixture f;
  const auto u = f.auth->create_user("h@example.dk", "H", "hemmeligt1", false);
  const auto admin =
      f.auth->create_user("root@example.dk", "R", "hemmeligt1", true);
  f.stores.members->set_role("k", u.id, Role::editor);

  Principal who;
  who.kind = PrincipalKind::session;
  who.user = u;
  CHECK(f.auth->role_in(who, "k") == Role::editor);
  CHECK_FALSE(f.auth->role_in(who, "other"));

  Principal root;
  root.kind = PrincipalKind::session;
  root.user = admin;
  CHECK(f.auth->role_in(root, "anything") == Role::owner);

  who.case_scope = "other";
  CHECK_FALSE(f.auth->role_in(who, "k"));

  CHECK(f.auth->role_in(local_principal(), "k") == Role::owner);
}
