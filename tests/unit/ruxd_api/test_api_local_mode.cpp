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

TEST_CASE("PresentedToken_HeaderCookieQuery_FirstPresentWins",
          "[ruxd_api][local]") {
  CHECK(presented_token("Bearer abc", "ruxd_token=def", "ghi") == "abc");
  CHECK(presented_token("bearer  abc ", "", "") == "abc");
  CHECK(presented_token("", "a=1; ruxd_token=def; b=2", "ghi") == "def");
  CHECK(presented_token("Basic xyz", "other=1", "ghi") == "ghi");
  CHECK(presented_token("", "", "").empty());
  CHECK(presented_token("Bearer ", "ruxd_tokenx=1", "").empty());
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
