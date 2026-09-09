// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Tests for ruxd's bearer-auth enforcement: fail-closed route matching against
// Crow route rules, constant-time token comparison, and refusal to run without
// an endpoint registry. The middleware is exercised directly — no server is
// started, a crow::request/crow::response pair is enough.

#include <auth.hpp>
#include <endpoints.hpp>
#include <route_match.hpp>

#include <catch2/catch_test_macros.hpp>

#include <string>

namespace {

constexpr const char *kToken = "s3cr3t-token";

// A route table mixing public, protected, parameterized and catch-all rules.
ruxd::EndpointRegistry make_registry() {
  ruxd::EndpointRegistry reg;
  reg.add({"GET", "/health", "Public health probe", false, {}});
  reg.add({"POST", "/segment/planes", "Protected static route", true, {}});
  reg.add({"POST", "/x/<string>", "Protected parameterized route", true, {}});
  reg.add({"GET", "/blobs/<path>", "Protected catch-all route", true, {}});
  return reg;
}

// Run before_handle() for one request and report the resulting status code
// (200 = passed through to the handler, 401 = rejected).
int auth_status(const ruxd::EndpointRegistry &reg, const std::string &token,
                crow::HTTPMethod method, const std::string &url,
                const std::string &authorization = {}) {
  ruxd::BearerAuthMiddleware mw;
  mw.configure(&reg, token);

  crow::request req;
  req.method = method;
  req.url = url;
  if (!authorization.empty()) {
    req.add_header("Authorization", authorization);
  }

  crow::response res;
  ruxd::BearerAuthMiddleware::context ctx;
  mw.before_handle(req, res, ctx);
  return res.code;
}

std::string bearer(const std::string &token) { return "Bearer " + token; }

} // namespace

TEST_CASE("RoutePatternMatches_VariousUrlShapes_MatchesCorrectly",
          "[ruxd][auth]") {
  using ruxd::route_pattern_matches;

  SECTION("static rules") {
    CHECK(route_pattern_matches("/health", "/health"));
    CHECK_FALSE(route_pattern_matches("/health", "/healthz"));
    CHECK_FALSE(route_pattern_matches("/health", "/health/detail"));
    CHECK_FALSE(route_pattern_matches("/segment/planes", "/segment"));
    CHECK(route_pattern_matches("/", "/"));
  }

  SECTION("trailing and repeated slashes are normalized") {
    CHECK(route_pattern_matches("/health", "/health/"));
    CHECK(route_pattern_matches("/health/", "/health"));
    CHECK(route_pattern_matches("/segment/planes", "//segment//planes//"));
  }

  SECTION("query strings are ignored") {
    CHECK(route_pattern_matches("/health", "/health?verbose=1"));
    CHECK(route_pattern_matches("/health", "/health/?verbose=1"));
  }

  SECTION("single-segment placeholders") {
    CHECK(route_pattern_matches("/x/<string>", "/x/abc"));
    CHECK(route_pattern_matches("/x/<string>", "/x/abc/"));
    CHECK(route_pattern_matches("/projects/<int>/segment",
                                "/projects/42/segment"));
    CHECK(route_pattern_matches("/p/<double>", "/p/1.5"));
    CHECK_FALSE(route_pattern_matches("/x/<string>", "/x"));
    CHECK_FALSE(route_pattern_matches("/x/<string>", "/x/abc/def"));
    CHECK_FALSE(route_pattern_matches("/x/<string>", "/y/abc"));
  }

  SECTION("catch-all placeholder spans several segments") {
    CHECK(route_pattern_matches("/blobs/<path>", "/blobs/a"));
    CHECK(route_pattern_matches("/blobs/<path>", "/blobs/a/b/c.ply"));
    CHECK_FALSE(route_pattern_matches("/blobs/<path>", "/blobs"));
    CHECK_FALSE(route_pattern_matches("/blobs/<path>", "/blobs/"));
  }

  SECTION("literal segments are case-sensitive like Crow's router") {
    CHECK_FALSE(route_pattern_matches("/health", "/Health"));
  }
}

TEST_CASE("HttpMethodEquals_CaseVariants_IgnoresCase", "[ruxd][auth]") {
  CHECK(ruxd::http_method_equals("GET", "get"));
  CHECK(ruxd::http_method_equals("POST", "POST"));
  CHECK_FALSE(ruxd::http_method_equals("GET", "POST"));
  CHECK_FALSE(ruxd::http_method_equals("GET", "GETT"));
}

TEST_CASE("EndpointRegistry_RequiresAuth_ResolvesFailClosed", "[ruxd][auth]") {
  const ruxd::EndpointRegistry reg = make_registry();

  SECTION("declared public routes stay public") {
    CHECK_FALSE(reg.requires_auth("GET", "/health"));
    CHECK_FALSE(reg.requires_auth("GET", "/health/"));
  }

  SECTION("protected static route") {
    CHECK(reg.requires_auth("POST", "/segment/planes"));
  }

  SECTION("protected parameterized route with a concrete URL") {
    CHECK(reg.requires_auth("POST", "/x/abc"));
    CHECK(reg.requires_auth("POST", "/x/abc/"));
    CHECK(reg.requires_auth("GET", "/blobs/scan/cloud.ply"));
  }

  SECTION("unknown routes require auth") {
    CHECK(reg.requires_auth("GET", "/nope"));
    CHECK(reg.requires_auth("GET", "/"));
    CHECK(reg.requires_auth("GET", "/health/../secret"));
    CHECK(reg.requires_auth("POST", "/x/abc/def")); // too many segments
  }

  SECTION("a wrong method on a known path requires auth") {
    CHECK(reg.requires_auth("GET", "/segment/planes"));
    CHECK(reg.requires_auth("POST", "/health"));
  }

  SECTION("an empty registry protects everything") {
    const ruxd::EndpointRegistry empty;
    CHECK(empty.requires_auth("GET", "/health"));
  }

  SECTION("overlapping rules resolve to the strictest") {
    ruxd::EndpointRegistry overlapping;
    overlapping.add({"GET", "/a/<string>", "public", false, {}});
    overlapping.add({"GET", "/a/secret", "protected", true, {}});
    CHECK(overlapping.requires_auth("GET", "/a/secret"));
    CHECK_FALSE(overlapping.requires_auth("GET", "/a/other"));
  }
}

TEST_CASE("ConstantTimeEquals_VariousTokenPairs_ComparesConstantTime",
          "[ruxd][auth]") {
  CHECK(ruxd::constant_time_equals("abc", "abc"));
  CHECK_FALSE(ruxd::constant_time_equals("abc", "abd"));
  CHECK_FALSE(ruxd::constant_time_equals("abc", "Abc"));
  CHECK_FALSE(ruxd::constant_time_equals("abc", "abcd")); // shorter presented
  CHECK_FALSE(ruxd::constant_time_equals("abcd", "abc")); // longer presented
  CHECK_FALSE(ruxd::constant_time_equals("", "abc"));
  CHECK_FALSE(ruxd::constant_time_equals("abc", ""));
  CHECK(ruxd::constant_time_equals("", ""));
}

TEST_CASE("BearerAuthMiddleware_UnauthenticatedOrWrongToken_Returns401",
          "[ruxd][auth]") {
  const ruxd::EndpointRegistry reg = make_registry();

  SECTION("protected static route without a token") {
    CHECK(auth_status(reg, kToken, crow::HTTPMethod::Post, "/segment/planes") ==
          401);
  }

  SECTION("protected parameterized route without a token") {
    CHECK(auth_status(reg, kToken, crow::HTTPMethod::Post, "/x/abc") == 401);
    CHECK(auth_status(reg, kToken, crow::HTTPMethod::Post, "/x/abc/") == 401);
    CHECK(auth_status(reg, kToken, crow::HTTPMethod::Get,
                      "/blobs/scan/cloud.ply") == 401);
  }

  SECTION("unknown route without a token — fail closed") {
    CHECK(auth_status(reg, kToken, crow::HTTPMethod::Get, "/nope") == 401);
  }

  SECTION("wrong token is rejected") {
    CHECK(auth_status(reg, kToken, crow::HTTPMethod::Post, "/x/abc",
                      bearer("wrong-token")) == 401);
    CHECK(auth_status(reg, kToken, crow::HTTPMethod::Post, "/x/abc",
                      bearer(std::string(kToken) + "x")) == 401);
  }

  SECTION("malformed Authorization header is rejected") {
    CHECK(auth_status(reg, kToken, crow::HTTPMethod::Post, "/x/abc", kToken) ==
          401);
    CHECK(auth_status(reg, kToken, crow::HTTPMethod::Post, "/x/abc",
                      "Basic " + std::string(kToken)) == 401);
  }

  SECTION("correct token is accepted") {
    CHECK(auth_status(reg, kToken, crow::HTTPMethod::Post, "/x/abc",
                      bearer(kToken)) == 200);
    CHECK(auth_status(reg, kToken, crow::HTTPMethod::Post, "/segment/planes",
                      bearer(kToken)) == 200);
    CHECK(auth_status(reg, kToken, crow::HTTPMethod::Get, "/nope",
                      bearer(kToken)) == 200);
  }

  SECTION("public routes pass without a token") {
    CHECK(auth_status(reg, kToken, crow::HTTPMethod::Get, "/health") == 200);
    CHECK(auth_status(reg, kToken, crow::HTTPMethod::Get, "/health/") == 200);
  }

  SECTION("no configured token disables enforcement") {
    CHECK(auth_status(reg, "", crow::HTTPMethod::Post, "/x/abc") == 200);
    CHECK(auth_status(reg, "", crow::HTTPMethod::Get, "/nope") == 200);
  }
}

TEST_CASE("BearerAuthMiddleware_BeforeHandle_RecordsAuthenticatedState",
          "[ruxd][auth]") {
  const ruxd::EndpointRegistry reg = make_registry();

  auto authenticated = [&reg](const std::string &token,
                              const std::string &authorization) {
    ruxd::BearerAuthMiddleware mw;
    mw.configure(&reg, token);
    crow::request req;
    req.method = crow::HTTPMethod::Get;
    req.url = "/health";
    if (!authorization.empty()) {
      req.add_header("Authorization", authorization);
    }
    crow::response res;
    ruxd::BearerAuthMiddleware::context ctx;
    mw.before_handle(req, res, ctx);
    return ctx.authenticated;
  };

  CHECK(authenticated(kToken, bearer(kToken)));
  CHECK_FALSE(authenticated(kToken, bearer("wrong")));
  CHECK_FALSE(authenticated(kToken, {}));
  // Auth disabled is deliberately NOT authenticated, so privileged detail is
  // never exposed without a correct token.
  CHECK_FALSE(authenticated("", bearer(kToken)));
}

TEST_CASE("BearerAuthMiddleware_NullEndpointRegistry_Throws", "[ruxd][auth]") {
  ruxd::BearerAuthMiddleware mw;
  CHECK_THROWS_AS(mw.configure(nullptr, kToken), std::runtime_error);

  SECTION("an unconfigured middleware refuses to serve") {
    crow::request req;
    req.method = crow::HTTPMethod::Get;
    req.url = "/health";
    crow::response res;
    ruxd::BearerAuthMiddleware::context ctx;
    mw.before_handle(req, res, ctx);
    CHECK(res.code == 503);
  }
}
