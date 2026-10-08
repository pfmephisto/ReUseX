// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Cases (ruxd multi-case spec, phase S2): ids from file names, the local case
// store, chunked uploads and their limits, and path-traversal safety.

#include <catch2/catch_test_macros.hpp>

#include <api/api.hpp>
#include <api/cases.hpp>

#include <reusex/core/ProjectDB.hpp>

#include "../../support/temp_path.hpp"

#include <fstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace ruxd::api;
using reusex::test_support::TempDir;

namespace {

/// A real, migrated (empty) project at @p path.
void make_project(const fs::path &path) {
  fs::create_directories(path.parent_path());
  reusex::ProjectDB db(path, /*readOnly=*/false);
}

void write_text(const fs::path &path, const std::string &text) {
  fs::create_directories(path.parent_path());
  std::ofstream(path, std::ios::binary) << text;
}

std::string read_text(const fs::path &path) {
  std::ifstream in(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), {}};
}

int status_of(const auto &fn) {
  try {
    fn();
  } catch (const HttpError &e) {
    return e.status();
  }
  return 0;
}

std::vector<std::string> ids_of(const std::vector<CaseInfo> &cases) {
  std::vector<std::string> ids;
  for (const auto &c : cases)
    ids.push_back(c.id);
  return ids;
}

} // namespace

// ===========================================================================
// Ids
// ===========================================================================

TEST_CASE("CaseSlug_FileNames_BecomeUrlSafeIds", "[ruxd_api][cases]") {
  CHECK(case_slug("NewOffice") == "newoffice");
  CHECK(case_slug("office_corridor") == "office-corridor");
  CHECK(case_slug("Kontor 2. sal") == "kontor-2-sal");
  CHECK(case_slug("Bøgevej Ålborg æble") == "boegevej-aalborg-aeble");
  CHECK(case_slug("  --scan.v2--  ") == "scan-v2");
  CHECK(case_slug("../../etc/passwd") == "etc-passwd");
  CHECK(case_slug("") == "sag");
  CHECK(case_slug("???") == "sag");
  CHECK(case_slug(std::string(100, 'a')).size() == 64);
}

TEST_CASE("AssignCaseIds_Collisions_GetStableSuffixes", "[ruxd_api][cases]") {
  const auto ids = assign_case_ids({"Scan", "scan", "SCAN", "other"});
  CHECK(ids == std::vector<std::string>{"scan", "scan-2", "scan-3", "other"});
  // Deterministic: the same input always gives the same ids.
  CHECK(assign_case_ids({"Scan", "scan", "SCAN", "other"}) == ids);
}

TEST_CASE("CaseIdOfEventsUrl_OnlyTheEventsRoute", "[ruxd_api][cases]") {
  CHECK(case_id_of_events_url("/api/v1/cases/abc/events") ==
        std::optional<std::string>("abc"));
  CHECK_FALSE(case_id_of_events_url("/api/v1/cases//events"));
  CHECK_FALSE(case_id_of_events_url("/api/v1/cases/a/b/events"));
  CHECK_FALSE(case_id_of_events_url("/api/v1/events"));
  CHECK_FALSE(case_id_of_events_url("/api/v1/cases/abc/jobs"));
}

TEST_CASE("CaseBodies_Validated", "[ruxd_api][cases]") {
  CHECK(parse_case_create(R"({"name":"  Ny sag  "})") == "Ny sag");
  CHECK(status_of([] { parse_case_create(R"({"name":"  "})"); }) == 400);
  CHECK(status_of([] { parse_case_create(R"({})"); }) == 400);
  CHECK(status_of([] { parse_case_create("nope"); }) == 400);
  CHECK(status_of([] { parse_case_create(R"({"name":"a\u0001b"})"); }) == 400);

  const auto patch = parse_case_patch(R"({"name":"X","archived":true})");
  CHECK(patch.name == std::optional<std::string>("X"));
  CHECK(patch.archived == std::optional<bool>(true));
  CHECK(status_of([] { parse_case_patch(R"({"path":"/etc"})"); }) == 400);
  CHECK(status_of([] { parse_case_patch(R"({"archived":"yes"})"); }) == 400);

  CHECK(parse_upload_request(R"({"name":"x","size":10})").size == 10);
  CHECK(status_of([] { parse_upload_request(R"({"name":"x","size":0})"); }) ==
        400);
  CHECK(status_of([] { parse_upload_request(R"({"name":"x","size":-4})"); }) ==
        400);
  CHECK(parse_upload_offset("42") == 42);
  CHECK(status_of([] { parse_upload_offset(nullptr); }) == 400);
  CHECK(status_of([] { parse_upload_offset("-1"); }) == 400);
  CHECK(status_of([] { parse_upload_offset("1e3"); }) == 400);
}

// ===========================================================================
// LocalCaseStore
// ===========================================================================

TEST_CASE("LocalCaseStore_Directory_EveryRuxIsACase", "[ruxd_api][cases]") {
  TempDir dir("test_api_cases");
  make_project(dir.path / "NewOffice.rux");
  make_project(dir.path / "office_corridor.rux");
  make_project(dir.path / "Office Corridor.rux"); // collides with the above
  write_text(dir.path / "notes.txt", "not a case");
  make_project(dir.path / ".hidden.rux");
  make_project(dir.path / "mappe" / "project.rux"); // a case directory

  LocalCaseStore store(dir.path, {});
  const auto cases = store.list();
  CHECK(ids_of(cases) == std::vector<std::string>{"mappe", "newoffice",
                                                  "office-corridor",
                                                  "office-corridor-2"});
  // Sorted by path, so "Office Corridor.rux" (capital O) takes the bare id.
  const auto first = store.find("office-corridor");
  REQUIRE(first);
  CHECK(first->path.filename() == "Office Corridor.rux");
  CHECK(first->name == "Office Corridor");
  CHECK(first->size_bytes > 0);
  CHECK_FALSE(first->created_at.empty());
  CHECK(store.writable()); // the directory is its own data dir
  CHECK(first->deletable);

  // The same directory always yields the same ids.
  LocalCaseStore again(dir.path, {});
  CHECK(ids_of(again.list()) == ids_of(cases));
}

TEST_CASE("LocalCaseStore_LoneFile_IsOneReadOnlyCase", "[ruxd_api][cases]") {
  TempDir dir("test_api_cases");
  const auto file = dir.path / "Scan.rux";
  make_project(file);
  make_project(dir.path / "sibling.rux"); // not served

  LocalCaseStore store(file, {});
  const auto cases = store.list();
  REQUIRE(cases.size() == 1);
  CHECK(cases[0].id == "scan");
  CHECK_FALSE(store.writable());
  CHECK_FALSE(cases[0].deletable);
  CHECK(store.staging_dir().empty());
  CHECK(status_of([&] { store.create("Ny"); }) == 409);
  CHECK(status_of([&] { store.move_to_trash("scan"); }) == 409);

  // Renaming still works, in memory.
  CHECK(store.update("scan", {std::string("Mit kontor"), {}}).name ==
        "Mit kontor");
  CHECK(status_of([&] { store.update("nope", {}); }) == 404);
}

TEST_CASE("LocalCaseStore_CreateRenameDelete_RoundTrip", "[ruxd_api][cases]") {
  TempDir dir("test_api_cases");
  make_project(dir.path / "kontor.rux");
  LocalCaseStore store(dir.path, {});

  // A new case is a directory named by a free slug, holding a real project.
  const auto created = store.create("Kontor");
  CHECK(created.id == "kontor-2"); // "kontor" is taken by kontor.rux
  CHECK(created.name == "Kontor");
  CHECK(created.path ==
        fs::weakly_canonical(dir.path) / "kontor-2" / "project.rux");
  CHECK(fs::exists(created.path));
  CHECK(store.find("kontor-2"));

  // Rename and archive persist across a restart (stored in .ruxd/cases.json).
  store.update("kontor-2", {std::string("Kontor, 2. sal"), true});
  {
    LocalCaseStore reopened(dir.path, {});
    const auto info = reopened.find("kontor-2");
    REQUIRE(info);
    CHECK(info->name == "Kontor, 2. sal");
    CHECK(info->archived);
  }

  // Delete moves the files into the trash; nothing is removed.
  const auto trashed = store.move_to_trash("kontor-2");
  CHECK_FALSE(store.find("kontor-2"));
  CHECK(fs::exists(trashed / "project.rux"));
  CHECK(trashed.parent_path().filename() == "trash");

  const auto flat = store.move_to_trash("kontor");
  CHECK_FALSE(fs::exists(dir.path / "kontor.rux"));
  CHECK(fs::exists(flat / "kontor.rux"));
  CHECK(store.list().empty());
  CHECK(status_of([&] { store.move_to_trash("kontor"); }) == 404);
}

TEST_CASE("LocalCaseStore_TraversalAndSymlinks_NeverEscape",
          "[ruxd_api][cases]") {
  TempDir outside("test_api_cases_outside");
  make_project(outside.path / "secret" / "project.rux");
  TempDir dir("test_api_cases");
  make_project(dir.path / "real.rux");
  // A symlinked directory is not followed.
  fs::create_directory_symlink(outside.path / "secret", dir.path / "linked");

  LocalCaseStore store(dir.path, {});
  CHECK(ids_of(store.list()) == std::vector<std::string>{"real"});
  for (const char *id : {"../secret", "..", "/etc/passwd", "linked",
                         "real/../../x", "%2e%2e", ""}) {
    INFO(id);
    CHECK_FALSE(store.find(id));
    CHECK(status_of([&] { store.move_to_trash(id); }) == 404);
  }

  // A hostile name only ever becomes a slug under the data dir.
  const auto created = store.create("../../../../tmp/evil");
  CHECK(created.id == "tmp-evil");
  CHECK(created.path.parent_path().parent_path() ==
        fs::weakly_canonical(dir.path));
  CHECK_FALSE(fs::exists(outside.path / "tmp"));
}

TEST_CASE("LocalCaseStore_DataDir_SeparateFromServedFile",
          "[ruxd_api][cases]") {
  TempDir dir("test_api_cases");
  TempDir data("test_api_cases_data");
  const auto file = dir.path / "scan.rux";
  make_project(file);
  LocalCaseStore store(file, data.path);
  CHECK(store.writable());

  const auto created = store.create("Ny sag");
  CHECK(ids_of(store.list()) == std::vector<std::string>{"ny-sag", "scan"});
  CHECK(created.deletable);
  // The served file lives outside the data dir: listed, never deleted here.
  CHECK(status_of([&] { store.move_to_trash("scan"); }) == 409);
  CHECK(fs::exists(file));
}

// ===========================================================================
// Uploads
// ===========================================================================

TEST_CASE("UploadManager_ChunkedUpload_AdoptedAsCase", "[ruxd_api][upload]") {
  TempDir dir("test_api_cases");
  LocalCaseStore store(dir.path, {});
  UploadManager uploads(store.staging_dir(), {});

  // Build a real project elsewhere and upload its bytes in three chunks.
  TempDir src("test_api_cases_src");
  make_project(src.path / "source.rux");
  const std::string bytes = read_text(src.path / "source.rux");
  REQUIRE(bytes.size() > 100);

  auto session = uploads.begin("Uploadet sag", bytes.size());
  CHECK(session.id.size() == 32);
  const std::size_t third = bytes.size() / 3;
  uploads.append(session.id, 0, std::string_view(bytes).substr(0, third));
  // A resumed client asks where it got to.
  CHECK(uploads.status(session.id).received == third);
  uploads.append(session.id, third,
                 std::string_view(bytes).substr(third, third));
  uploads.append(session.id, 2 * third,
                 std::string_view(bytes).substr(2 * third));

  auto [done, staged] = uploads.finish(session.id);
  CHECK(done.received == bytes.size());
  CHECK(has_sqlite_header(staged));
  const auto info = store.adopt(done.name, staged);
  CHECK(info.id == "uploadet-sag");
  CHECK(read_text(info.path).size() >= bytes.size());
  CHECK_FALSE(fs::exists(staged));
  CHECK(status_of([&] { uploads.status(session.id); }) == 404);
}

TEST_CASE("UploadManager_Limits_Enforced", "[ruxd_api][upload]") {
  TempDir dir("test_api_cases");
  UploadLimits limits;
  limits.max_bytes = 100;
  limits.max_chunk_bytes = 10;
  limits.max_sessions = 2;
  UploadManager uploads(dir.path / "staging", limits);

  CHECK(status_of([&] { uploads.begin("x", 101); }) == 413);
  CHECK(status_of([&] { uploads.begin("x", 0); }) == 400);
  CHECK(status_of([&] { uploads.begin("", 10); }) == 400);

  const auto s = uploads.begin("x", 15);
  // Chunk too large for one request.
  CHECK(status_of([&] { uploads.append(s.id, 0, std::string(11, 'a')); }) ==
        413);
  // Out of order: the offset must continue the upload.
  CHECK(status_of([&] { uploads.append(s.id, 5, "abc"); }) == 409);
  uploads.append(s.id, 0, std::string(10, 'a'));
  CHECK(status_of([&] { uploads.append(s.id, 0, "a"); }) == 409);
  // Past the declared size.
  CHECK(status_of([&] { uploads.append(s.id, 10, std::string(6, 'a')); }) ==
        413);
  // Incomplete uploads cannot be finished.
  CHECK(status_of([&] { uploads.finish(s.id); }) == 409);

  // Session cap.
  uploads.begin("y", 5);
  CHECK(status_of([&] { uploads.begin("z", 5); }) == 429);

  // Unknown and malformed ids (never joined onto a path).
  for (const char *id :
       {"nope", "../../etc/passwd", "0123456789abcdef0123456789abcdeg", ""}) {
    INFO(id);
    CHECK(status_of([&] { uploads.append(id, 0, "a"); }) == 404);
    CHECK(status_of([&] { uploads.status(id); }) == 404);
  }
  uploads.abort("../../etc/passwd"); // ignored, no throw
}

TEST_CASE("UploadManager_IdleUploads_Expire", "[ruxd_api][upload]") {
  TempDir dir("test_api_cases");
  UploadLimits limits;
  limits.idle_ttl = std::chrono::seconds(60);
  UploadManager uploads(dir.path, limits);
  const auto s = uploads.begin("x", 5);
  CHECK(uploads.expire(UploadManager::Clock::now()) == 0);
  CHECK(uploads.expire(UploadManager::Clock::now() + std::chrono::minutes(2)) ==
        1);
  CHECK(status_of([&] { uploads.status(s.id); }) == 404);
  CHECK_FALSE(fs::exists(dir.path / (s.id + ".part")));
}

TEST_CASE("HasSqliteHeader_RejectsOtherFiles", "[ruxd_api][upload]") {
  TempDir dir("test_api_cases");
  write_text(dir.path / "fake.rux", "this is not sqlite at all, sorry");
  CHECK_FALSE(has_sqlite_header(dir.path / "fake.rux"));
  CHECK_FALSE(has_sqlite_header(dir.path / "missing.rux"));
  make_project(dir.path / "real.rux");
  CHECK(has_sqlite_header(dir.path / "real.rux"));
}
