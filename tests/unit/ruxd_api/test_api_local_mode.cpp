// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `ruxd --local` rules: which binds count as loopback, which project a path
// names, and how a request presents the access token.

#include <catch2/catch_test_macros.hpp>

#include <api/local_mode.hpp>

#include "../../support/temp_path.hpp"

#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace ruxd::api;
using reusex::test_support::TempDir;

namespace {
void touch(const fs::path &p) { std::ofstream(p) << ""; }
} // namespace

TEST_CASE("IsLoopbackBind_LoopbackAndRoutableHosts_Classified",
          "[ruxd_api][local]") {
  for (const char *host :
       {"127.0.0.1", "127.1.2.3", "localhost", "LOCALHOST", "::1", "[::1]"}) {
    INFO(host);
    CHECK(is_loopback_bind(host));
  }
  for (const char *host :
       {"0.0.0.0", "::", "[::]", "192.168.1.20", "10.0.0.1", "128.0.0.1",
        "127.0.0", "127.0.0.256", "127.0.0.1.5", "example.org", ""}) {
    INFO(host);
    CHECK_FALSE(is_loopback_bind(host));
  }
}

TEST_CASE("ResolveLocalProject_FileOrSingleProjectDir_ReturnsThatFile",
          "[ruxd_api][local]") {
  TempDir dir("test_api_local_mode");

  SECTION("a .rux path is served as is, even before it exists") {
    const auto p = dir.path / "new.rux";
    CHECK(resolve_local_project(p) == p);
  }
  SECTION("a directory with exactly one .rux resolves to it") {
    touch(dir.path / "scan.rux");
    touch(dir.path / "notes.txt");
    CHECK(resolve_local_project(dir.path) == dir.path / "scan.rux");
  }
}

TEST_CASE("ResolveLocalProject_AmbiguousOrWrongTarget_Throws",
          "[ruxd_api][local]") {
  TempDir dir("test_api_local_mode");

  SECTION("empty directory") {
    CHECK_THROWS_AS(resolve_local_project(dir.path), std::runtime_error);
  }
  SECTION("several projects are refused, not guessed between") {
    touch(dir.path / "a.rux");
    touch(dir.path / "b.rux");
    CHECK_THROWS_AS(resolve_local_project(dir.path), std::runtime_error);
  }
  SECTION("a non-.rux file") {
    touch(dir.path / "x.db");
    CHECK_THROWS_AS(resolve_local_project(dir.path / "x.db"),
                    std::runtime_error);
  }
  SECTION("empty path") {
    CHECK_THROWS_AS(resolve_local_project({}), std::runtime_error);
  }
}

TEST_CASE("TokenCookieName_IsPerPort", "[ruxd_api][local]") {
  CHECK(token_cookie_name(8420) == "ruxd_token_8420");
  CHECK(token_cookie_name(8451) != token_cookie_name(8420));
}

TEST_CASE("CheckToken_AnyMatchingCredential_Accepted", "[ruxd_api][local]") {
  const std::string c = "ruxd_token_8420";
  auto ok = [&](std::string_view auth, std::string_view cookie,
                std::string_view query) {
    return check_token(auth, cookie, query, c, "right");
  };
  CHECK(ok("Bearer right", "", "").ok);
  CHECK(ok("bearer  right ", "", "").ok);
  CHECK(ok("", "a=1; ruxd_token_8420=right; b=2", "").ok);
  CHECK(ok("", "", "right").ok);

  // A stale cookie must not shadow a correct header or query token.
  CHECK(ok("Bearer right", "ruxd_token_8420=stale", "").ok);
  const auto via_query = ok("", "ruxd_token_8420=stale", "right");
  CHECK(via_query.ok);
  CHECK(via_query.via_query); // the response overwrites the stale cookie
  // ...and of two same-named cookies, either may be the right one.
  CHECK(ok("", "ruxd_token_8420=stale; ruxd_token_8420=right", "").ok);

  // Only the header/cookie matched: no cookie to (re)set.
  CHECK_FALSE(ok("Bearer right", "", "").via_query);

  // Wrong everywhere, another port's cookie, or no expected token at all.
  CHECK_FALSE(ok("Bearer nope", "ruxd_token_8420=nope", "nope").ok);
  CHECK_FALSE(ok("", "ruxd_token_9000=right", "").ok);
  CHECK_FALSE(ok("Basic right", "ruxd_token=right", "").ok);
  CHECK_FALSE(check_token("Bearer x", "", "x", c, "").ok);
}

TEST_CASE("StripQueryParam_RemovesOnlyThatKey", "[ruxd_api][local]") {
  CHECK(strip_query_param("/?token=abc", "token") == "/");
  CHECK(strip_query_param("/kortlaegning?token=abc&x=1", "token") ==
        "/kortlaegning?x=1");
  CHECK(strip_query_param("/a?x=1&token=abc&y=2", "token") == "/a?x=1&y=2");
  CHECK(strip_query_param("/a?tokens=1&token", "token") == "/a?tokens=1");
  CHECK(strip_query_param("/a", "token") == "/a");
}

TEST_CASE("RedactQueryStrings_HidesEveryQueryValue", "[ruxd_api][local]") {
  CHECK(redact_query_strings("Response: 0x1 /?token=s3cret 303 0") ==
        "Response: 0x1 /?<redacted> 303 0");
  CHECK(redact_query_strings("GET /a?x=1&token=s3 HTTP") ==
        "GET /a?<redacted> HTTP");
  CHECK(redact_query_strings("no query here") == "no query here");
}

TEST_CASE("OriginMatchesHost_SameOriginOnly", "[ruxd_api][local]") {
  CHECK(origin_matches_host("http://192.168.1.20:8420", "192.168.1.20:8420"));
  CHECK(origin_matches_host("https://Box.lan:8420", "box.lan:8420"));
  CHECK_FALSE(
      origin_matches_host("http://192.168.1.20:9999", "192.168.1.20:8420"));
  CHECK_FALSE(origin_matches_host("http://evil.example", "192.168.1.20:8420"));
  CHECK_FALSE(origin_matches_host("http://x", ""));
}

TEST_CASE("HostAllowed_LoopbackWildcardAndSpecificBinds", "[ruxd_api][local]") {
  const std::vector<std::string> none;
  SECTION("loopback bind accepts only loopback names (DNS rebinding)") {
    for (const char *h : {"localhost:8420", "127.0.0.1:8420", "[::1]:8420",
                          "LOCALHOST", "127.0.0.1"}) {
      INFO(h);
      CHECK(host_allowed(h, "127.0.0.1", none));
    }
    for (const char *h :
         {"evil.example:8420", "192.168.1.20:8420", "", "127.0.0.2:8420"}) {
      INFO(h);
      CHECK_FALSE(host_allowed(h, "127.0.0.1", none));
    }
  }
  SECTION("wildcard bind accepts any non-empty host (a token is required)") {
    CHECK(host_allowed("192.168.1.20:8420", "0.0.0.0", none));
    CHECK(host_allowed("box.lan", "::", none));
    CHECK_FALSE(host_allowed("", "0.0.0.0", none));
  }
  SECTION("specific bind: itself or an --allow-origin host") {
    const std::vector<std::string> origins{"http://box.lan:8420"};
    CHECK(host_allowed("192.168.1.20:8420", "192.168.1.20", origins));
    CHECK(host_allowed("box.lan:8420", "192.168.1.20", origins));
    CHECK_FALSE(host_allowed("evil.example:8420", "192.168.1.20", origins));
    CHECK_FALSE(host_allowed("localhost:8420", "192.168.1.20", origins));
  }
}

TEST_CASE("TokenMatches_ExactOnly_EmptyExpectedNeverMatches",
          "[ruxd_api][local]") {
  CHECK(token_matches("s3cret", "s3cret"));
  CHECK_FALSE(token_matches("s3cre", "s3cret"));
  CHECK_FALSE(token_matches("s3cretX", "s3cret"));
  CHECK_FALSE(token_matches("S3cret", "s3cret"));
  CHECK_FALSE(token_matches("", ""));
  CHECK_FALSE(token_matches("anything", ""));
}
