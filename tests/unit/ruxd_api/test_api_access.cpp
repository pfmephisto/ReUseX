// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Who may do what (ruxd server mode, spec 2026-10-08 S3): the role matrix,
// route classification, the access decision and the login rate limiter.

#include <catch2/catch_test_macros.hpp>

#include <api/access.hpp>

#include <chrono>
#include <string>
#include <vector>

using namespace ruxd::api;
using Kind = RouteAccess::Kind;

namespace {

Principal user_principal(std::int64_t id, bool admin = false) {
  Principal p;
  p.kind = PrincipalKind::session;
  p.user.id = id;
  p.user.email = "u" + std::to_string(id) + "@example.dk";
  p.user.is_admin = admin;
  return p;
}

} // namespace

TEST_CASE("RoleMatrix_ViewerEditorOwner", "[ruxd_api][auth][roles]") {
  // viewer: GET only.
  CHECK(role_allows(Role::viewer, CaseOp::read));
  CHECK_FALSE(role_allows(Role::viewer, CaseOp::write));
  CHECK_FALSE(role_allows(Role::viewer, CaseOp::delete_case));
  CHECK_FALSE(role_allows(Role::viewer, CaseOp::manage_members));
  // editor: everything but deleting the case and managing members.
  CHECK(role_allows(Role::editor, CaseOp::read));
  CHECK(role_allows(Role::editor, CaseOp::write));
  CHECK_FALSE(role_allows(Role::editor, CaseOp::delete_case));
  CHECK_FALSE(role_allows(Role::editor, CaseOp::manage_members));
  // owner: everything.
  CHECK(role_allows(Role::owner, CaseOp::read));
  CHECK(role_allows(Role::owner, CaseOp::write));
  CHECK(role_allows(Role::owner, CaseOp::delete_case));
  CHECK(role_allows(Role::owner, CaseOp::manage_members));

  for (const auto role : {Role::viewer, Role::editor, Role::owner})
    CHECK(parse_role(to_string(role)) == role);
  CHECK_FALSE(parse_role("admin"));
}

TEST_CASE("ClassifyRoute_EveryKindOfRoute", "[ruxd_api][auth][roles]") {
  CHECK(classify_route("GET", "/").kind == Kind::static_file);
  CHECK(classify_route("GET", "/sager/x/kortlaegning").kind ==
        Kind::static_file);
  CHECK(classify_route("GET", "/assets/app.js").kind == Kind::static_file);

  CHECK(classify_route("GET", "/api/v1/health").kind == Kind::public_route);
  CHECK(classify_route("GET", "/api/v1/readyz").kind == Kind::public_route);
  CHECK(classify_route("GET", "/api/v1/livez").kind == Kind::authenticated);
  CHECK(classify_route("POST", "/api/v1/auth/login").kind ==
        Kind::public_route);
  CHECK(classify_route("GET", "/api/v1/auth/me").kind == Kind::authenticated);
  CHECK(classify_route("POST", "/api/v1/auth/logout").kind ==
        Kind::authenticated);
  CHECK(classify_route("GET", "/api/v1/models/sam3/status").kind ==
        Kind::authenticated);

  CHECK(classify_route("GET", "/api/v1/users").kind == Kind::admin);
  CHECK(classify_route("PATCH", "/api/v1/users/4").kind == Kind::admin);
  CHECK(classify_route("GET", "/api/v1/usersx").kind == Kind::authenticated);

  CHECK(classify_route("GET", "/api/v1/cases").kind == Kind::authenticated);
  CHECK(classify_route("POST", "/api/v1/cases").kind == Kind::create_case);
  CHECK(classify_route("PUT", "/api/v1/uploads/abc").kind == Kind::create_case);

  auto r = classify_route("GET", "/api/v1/cases/kontor/clouds");
  CHECK(r.kind == Kind::case_route);
  CHECK(r.cid == "kontor");
  CHECK(r.op == CaseOp::read);
  CHECK(classify_route("GET", "/api/v1/cases/kontor/events").op ==
        CaseOp::read);
  CHECK(classify_route("POST", "/api/v1/cases/kontor/jobs").op ==
        CaseOp::write);
  CHECK(classify_route("PATCH", "/api/v1/cases/kontor").op == CaseOp::write);
  CHECK(classify_route("DELETE", "/api/v1/cases/kontor").op ==
        CaseOp::delete_case);
  CHECK(classify_route("GET", "/api/v1/cases/kontor/members").op ==
        CaseOp::read);
  CHECK(classify_route("POST", "/api/v1/cases/kontor/members").op ==
        CaseOp::manage_members);
  CHECK(classify_route("DELETE", "/api/v1/cases/kontor/members/3").op ==
        CaseOp::manage_members);
  // A path that merely starts with "members" is an ordinary write.
  CHECK(classify_route("POST", "/api/v1/cases/kontor/membersx").op ==
        CaseOp::write);
}

TEST_CASE("DecideAccess_MembershipAndRoles", "[ruxd_api][auth][roles]") {
  const Principal anonymous;
  const auto viewer = user_principal(1);
  const auto admin = user_principal(2, /*admin=*/true);
  const auto read = classify_route("GET", "/api/v1/cases/k/project");
  const auto write = classify_route("POST", "/api/v1/cases/k/jobs");
  const auto remove = classify_route("DELETE", "/api/v1/cases/k");
  const auto members = classify_route("POST", "/api/v1/cases/k/members");

  // Public and static: anyone.
  CHECK(decide_access(anonymous, classify_route("GET", "/"), {}).allowed());
  CHECK(
      decide_access(anonymous, classify_route("POST", "/api/v1/auth/login"), {})
          .allowed());
  // Anything else needs a principal.
  CHECK(decide_access(anonymous, read, Role::owner).status == 401);
  CHECK(decide_access(anonymous, classify_route("GET", "/api/v1/cases"), {})
            .status == 401);

  // A non-member: 404, exactly like a case that does not exist.
  CHECK(decide_access(viewer, read, std::nullopt).status == 404);
  CHECK(decide_access(viewer, write, std::nullopt).status == 404);

  // A viewer may read, nothing else (403).
  CHECK(decide_access(viewer, read, Role::viewer).allowed());
  CHECK(decide_access(viewer, write, Role::viewer).status == 403);
  CHECK(decide_access(viewer, remove, Role::viewer).status == 403);
  CHECK(decide_access(viewer, members, Role::viewer).status == 403);
  // An editor may write, not delete or manage members.
  CHECK(decide_access(viewer, write, Role::editor).allowed());
  CHECK(decide_access(viewer, remove, Role::editor).status == 403);
  CHECK(decide_access(viewer, members, Role::editor).status == 403);
  // An owner may do all of it.
  CHECK(decide_access(viewer, remove, Role::owner).allowed());
  CHECK(decide_access(viewer, members, Role::owner).allowed());

  // An admin: every case, member or not, and user management.
  CHECK(decide_access(admin, remove, std::nullopt).allowed());
  CHECK(decide_access(admin, classify_route("GET", "/api/v1/users"), {})
            .allowed());
  CHECK(decide_access(viewer, classify_route("GET", "/api/v1/users"), {})
            .status == 403);

  // The superuser and local mode are admins.
  CHECK(decide_access(superuser_principal(), remove, {}).allowed());
  CHECK(decide_access(local_principal(), remove, {}).allowed());
}

TEST_CASE("DecideAccess_CaseScopedToken", "[ruxd_api][auth][roles]") {
  auto token = user_principal(5, /*admin=*/true);
  token.kind = PrincipalKind::api_token;
  token.case_scope = "k";
  // Its own case: as its user (an admin here).
  CHECK(decide_access(token, classify_route("POST", "/api/v1/cases/k/jobs"), {})
            .allowed());
  // Another case does not exist for it, admin or not.
  CHECK(decide_access(token, classify_route("GET", "/api/v1/cases/other/x"),
                      Role::owner)
            .status == 404);
  // It cannot create cases or manage users.
  CHECK(decide_access(token, classify_route("POST", "/api/v1/cases"), {})
            .status == 403);
  CHECK(
      decide_access(token, classify_route("GET", "/api/v1/users"), {}).status ==
      403);
}

TEST_CASE("LoginRateLimiter_AccountBackoffAfterFiveFailures",
          "[ruxd_api][auth][ratelimit]") {
  using namespace std::chrono_literals;
  LoginRateLimiter limiter;
  const auto t0 = LoginRateLimiter::Clock::time_point{} + 1h;
  const std::string a = rate_limit_key("10.0.0.1");
  for (int i = 0; i < 5; ++i) {
    CHECK(limiter.wait(a, "anna@x.dk", t0).count() == 0);
    limiter.failure(a, "anna@x.dk", t0);
  }
  // Past five failures: the account waits base_delay, doubling per failure,
  // from ANY address (a per-account bound an IPv6 rotation cannot dodge).
  CHECK(limiter.wait(a, "anna@x.dk", t0).count() == 1);
  CHECK(limiter.wait(rate_limit_key("2001:db8::7"), "ANNA@x.dk", t0).count() ==
        1);
  limiter.failure(a, "anna@x.dk", t0 + 1s);
  CHECK(limiter.wait(a, "anna@x.dk", t0 + 1s).count() == 2);
  limiter.failure(a, "anna@x.dk", t0 + 3s);
  CHECK(limiter.wait(a, "anna@x.dk", t0 + 3s).count() == 4);
  // Other accounts are not affected.
  CHECK(limiter.wait(a, "bo@x.dk", t0).count() == 0);
  // A success forgets the account's failures.
  limiter.success("anna@x.dk");
  CHECK(limiter.wait(a, "anna@x.dk", t0 + 3s).count() == 0);
}

TEST_CASE("LoginRateLimiter_BackoffIsCappedAndForgotten",
          "[ruxd_api][auth][ratelimit]") {
  using namespace std::chrono_literals;
  LoginRateLimitOptions options;
  options.max_delay = 60s;
  LoginRateLimiter limiter(options);
  const auto t0 = LoginRateLimiter::Clock::time_point{} + 1h;
  const std::string a = rate_limit_key("10.0.0.2");
  for (int i = 0; i < 40; ++i)
    limiter.failure(a, "c@x.dk", t0);
  // A user locked out by someone else waits at most max_delay, never for ever.
  CHECK(limiter.wait(a, "c@x.dk", t0).count() == 60);
  CHECK(limiter.wait(a, "c@x.dk", t0 + 61s).count() == 0);
  // Quiet for forget_after: clean slate.
  limiter.failure(a, "c@x.dk", t0 + 61s);
  CHECK(limiter.wait(a, "c@x.dk", t0 + 61s + options.forget_after).count() ==
        0);
}

TEST_CASE("LoginRateLimiter_SprayFromOneAddressOrIpv6Slash64",
          "[ruxd_api][auth][ratelimit]") {
  LoginRateLimitOptions options;
  options.free_failures_per_address = 10;
  LoginRateLimiter limiter(options);
  const auto t0 = LoginRateLimiter::Clock::time_point{} + std::chrono::hours(1);
  // Ten different addresses inside one /64, ten different accounts.
  for (int i = 0; i < 10; ++i)
    limiter.failure(rate_limit_key("2001:db8:1:2::" + std::to_string(i + 1)),
                    "user" + std::to_string(i) + "@x.dk", t0);
  // The /64 is one key: a fresh account from yet another address in it waits.
  CHECK(limiter.wait(rate_limit_key("2001:db8:1:2:ffff::9"), "fresh@x.dk", t0)
            .count() > 0);
  // Another /64 does not.
  CHECK(limiter.wait(rate_limit_key("2001:db8:1:3::1"), "fresh@x.dk", t0)
            .count() == 0);
  // Wrong bearer tokens use the address half alone.
  for (int i = 0; i < 25; ++i)
    limiter.failure_address(rate_limit_key("10.9.9.9"), t0);
  CHECK(limiter.wait_address(rate_limit_key("10.9.9.9"), t0).count() > 0);
}

TEST_CASE("LoginRateLimiter_BoundedMemory", "[ruxd_api][auth][ratelimit]") {
  LoginRateLimitOptions options;
  options.max_keys = 100;
  LoginRateLimiter limiter(options);
  const auto t0 = LoginRateLimiter::Clock::time_point{} + std::chrono::hours(1);
  for (int i = 0; i < 5000; ++i)
    limiter.failure("k" + std::to_string(i), "u" + std::to_string(i) + "@x.dk",
                    t0);
  CHECK(limiter.wait("fresh", "z@x.dk", t0).count() == 0);
}

TEST_CASE("ClientAddress_TrustedProxiesOnly", "[ruxd_api][auth][ratelimit]") {
  const std::vector<std::string> proxies{"127.0.0.1", "10.0.0.0/8"};
  // A direct client cannot claim another address.
  CHECK(client_address("203.0.113.5", "1.2.3.4", proxies) == "203.0.113.5");
  // Through our proxy: the right-most hop that is not one of our proxies.
  CHECK(client_address("127.0.0.1", "1.2.3.4", proxies) == "1.2.3.4");
  CHECK(client_address("127.0.0.1", "6.6.6.6, 1.2.3.4, 10.1.1.1", proxies) ==
        "1.2.3.4");
  CHECK(client_address("127.0.0.1", "", proxies) == "127.0.0.1");
  CHECK(client_address("127.0.0.1", "garbage", proxies) == "127.0.0.1");
  CHECK(client_address("127.0.0.1", "1.2.3.4", {}) == "127.0.0.1");

  CHECK(cidr_contains("10.0.0.0/8", "10.200.3.4"));
  CHECK_FALSE(cidr_contains("10.0.0.0/8", "11.0.0.1"));
  CHECK(cidr_contains("2001:db8::/32", "2001:db8:ff::1"));
  CHECK(cidr_contains("127.0.0.1", "::ffff:127.0.0.1"));
  CHECK_FALSE(is_valid_cidr("10.0.0.0/33"));
  CHECK_FALSE(is_valid_cidr("nope"));
  CHECK(is_valid_cidr("::1/128"));

  CHECK(rate_limit_key("2001:db8:1:2:3:4:5:6") == "v6:2001:db8:1:2::/64");
  CHECK(rate_limit_key("::ffff:192.0.2.1") == "v4:192.0.2.1");
  CHECK(rate_limit_key("192.0.2.1") == "v4:192.0.2.1");
}
