// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Case figures and renders read WITHOUT opening a case (review of S2, I3).

#include <catch2/catch_test_macros.hpp>

#include <api/case_meta.hpp>

#include <reusex/core/ProjectDB.hpp>

#include "../../support/temp_path.hpp"

#include <thread>

namespace fs = std::filesystem;
using namespace ruxd::api;
using reusex::test_support::TempDir;

TEST_CASE("ReadCaseSummary_Project_RecordAndSurvey", "[ruxd_api][cases]") {
  TempDir dir("test_api_case_meta");
  const auto file = dir.path / "kontor.rux";
  {
    reusex::ProjectDB db(file, /*readOnly=*/false);
  }
  const auto summary = read_case_summary(file);
  REQUIRE(summary.is_object());
  CHECK(summary.contains("project"));
  REQUIRE(summary["survey"].is_object());
  CHECK(summary["survey"].contains("counts"));
  // A file that is not there yet has nothing to say.
  CHECK(read_case_summary(dir.path / "missing.rux").is_null());
}

TEST_CASE("CaseSummaryCache_RereadsOnlyWhenTheFileChanges",
          "[ruxd_api][cases]") {
  TempDir dir("test_api_case_meta");
  CaseInfo info;
  info.id = "k";
  info.path = dir.path / "k.rux";
  {
    reusex::ProjectDB db(info.path, /*readOnly=*/false);
  }
  CaseSummaryCache cache;
  const auto first = cache.get(info);
  const auto stamp = file_stamp(info.path);
  CHECK(cache.get(info) == first);
  CHECK(file_stamp(info.path) == stamp); // reading changed nothing
}

TEST_CASE("RenderCache_HitsUntilTheProjectChanges", "[ruxd_api][cases]") {
  TempDir dir("test_api_case_meta");
  const auto file = dir.path / "k.rux";
  {
    reusex::ProjectDB db(file, /*readOnly=*/false);
  }
  RenderCache cache(/*budget_bytes=*/100);
  Blob blob{"image/png", std::vector<uint8_t>(40, 7)};
  CHECK_FALSE(cache.get(file, "?view=plan"));
  cache.put(file, "?view=plan", blob);
  REQUIRE(cache.get(file, "?view=plan"));
  CHECK(cache.get(file, "?view=plan")->data.size() == 40);
  CHECK_FALSE(cache.get(file, "?view=top"));

  // Over the byte budget: least recently used out first.
  cache.put(file, "?a", blob);
  cache.put(file, "?b", blob);
  CHECK(cache.bytes() <= 100);
  CHECK_FALSE(cache.get(file, "?view=plan"));

  // The project changes: the cached render is stale.
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  {
    reusex::ProjectDB db(file, /*readOnly=*/false);
    reusex::ProjectDB::ProjectMetadata meta;
    meta.id = "p1";
    meta.name = "Changed";
    db.update_project_metadata(meta);
  }
  CHECK_FALSE(cache.get(file, "?b"));
}
