// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Cases: the server-level collection of projects (spec 2026-10-08, phase S2).
//
// A *case* (Danish UI: "sag") is one `.rux` project file the server serves
// under `/api/v1/cases/{cid}/...`. "Project" is taken: `/api/v1/projects`
// already means the building records *inside* a `.rux` file.
//
// This header is framework-free so the rules — how a file name becomes an id,
// where a new case is stored, what an upload may do — are unit-tested in the
// light binary (tests/unit/ruxd_api/test_api_cases.cpp).
//
// STORAGE AND TRAVERSAL SAFETY. Nothing a client sends is ever joined onto a
// filesystem path:
//  * a case id is only ever *looked up* in the list a directory scan produced;
//  * a new case's directory name is a server-generated slug ([a-z0-9-]) that
//    is checked not to exist and created with create_directory, never
//    create_directories, so it cannot reuse or climb out of anything;
//  * an upload id is a server-generated 32-hex-digit token, checked against
//    that shape before it names a staging file;
//  * symlinked directories are never followed during a scan.

#include <nlohmann/json.hpp>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace ruxd::api {

/// One case as the server knows it.
struct CaseInfo {
  /// Stable, URL-safe id ([a-z0-9-], 1–64 chars), derived from the file or
  /// directory name (case_slug). Unique within the server.
  std::string id;
  /// Display name: the stored one when the case was named or renamed, else
  /// the file stem (or directory name).
  std::string name;
  /// The `.rux` file. Never sent to a client (the contract discloses no
  /// server paths); only its file name is.
  std::filesystem::path path;
  /// ISO-8601 UTC creation time (stored at creation, else the file's mtime).
  std::string created_at;
  bool archived = false;
  /// Size of the `.rux` file (without its WAL), 0 when unreadable.
  std::uintmax_t size_bytes = 0;
  /// True when the server may delete this case (it lives in the data dir).
  bool deletable = false;
};

/// The URL-safe id for a file stem or directory name: lower case, æ/ø/å and
/// a few other Latin letters folded to ASCII, every other run of characters
/// to one '-', trimmed, at most 64 characters. Never empty: a name with no
/// usable character becomes "sag".
std::string case_slug(std::string_view name);

/// Ids for @p names (already in a stable order): case_slug of each, with
/// "-2", "-3", … appended to the second and later holders of the same slug.
/// Deterministic, so the same directory always yields the same ids.
std::vector<std::string> assign_case_ids(const std::vector<std::string> &names);

/// A sparse update of a case's metadata (`PATCH /api/v1/cases/{cid}`).
struct CasePatch {
  std::optional<std::string> name;
  std::optional<bool> archived;
};

/// Parse and validate a PATCH body. @throws HttpError(400).
CasePatch parse_case_patch(std::string_view body);

/// Parse `POST /api/v1/cases` (`{"name": "..."}`, name required, trimmed,
/// 1–200 characters). @throws HttpError(400).
std::string parse_case_create(std::string_view body);

/// The catalogue of cases a server serves.
class ICaseStore {
    public:
  virtual ~ICaseStore() = default;

  /// Every case, sorted by id.
  virtual std::vector<CaseInfo> list() const = 0;
  /// One case, or nullopt.
  virtual std::optional<CaseInfo> find(std::string_view id) const = 0;

  /// Whether cases can be created, uploaded and deleted (a data dir exists).
  virtual bool writable() const = 0;
  /// Where uploads are staged; empty when not writable. Same filesystem as
  /// the cases, so adopting an upload is a rename.
  virtual std::filesystem::path staging_dir() const = 0;

  /// Create a new, empty case: a fresh, migrated project file. @p created_by
  /// is the user who asked (server mode records it; nullopt = nobody in
  /// particular).
  /// @throws HttpError(409) when not writable.
  virtual CaseInfo create(const std::string &name,
                          std::optional<std::int64_t> created_by) = 0;
  /// Create a case from a complete `.rux` at @p staged_file, which is moved
  /// into place and then opened (and migrated) once to prove it is a project.
  /// @throws HttpError(409) when not writable, HttpError(422) when the file
  ///         is not a usable project (the case is then removed again).
  virtual CaseInfo adopt(const std::string &name,
                         const std::filesystem::path &staged_file,
                         std::optional<std::int64_t> created_by) = 0;
  /// Apply @p patch. @throws HttpError(404) for an unknown id.
  virtual CaseInfo update(std::string_view id, const CasePatch &patch) = 0;
  /// Move the case's files (the `.rux`, its `-wal`/`-shm`, and its case
  /// directory when it has one) into the trash dir. The caller must have
  /// closed the project first. @return where it went.
  /// @throws HttpError(404) unknown, HttpError(409) not deletable.
  virtual std::filesystem::path move_to_trash(std::string_view id) = 0;
};

/// The store behind `ruxd --local <file.rux | dir>` (and, with a data dir,
/// the server's own storage).
///
/// Cases are found in two places:
///  * the served target: a `.rux` file, or every `.rux` file directly inside
///    a directory (flat, the way people keep projects);
///  * the data dir: one directory per case, `<data-dir>/<id>/project.rux`,
///    which is where created and uploaded cases go.
/// When the target is a directory and no data dir is given, the target is the
/// data dir. A lone file with no data dir is read-only: nothing can be
/// created or deleted.
///
/// Names and archive flags live in `<data-dir>/.ruxd/cases.json`, keyed by
/// the case file's path relative to the data dir (or in memory when there is
/// none). Trash goes to `<data-dir>/.ruxd/trash/`, uploads are staged in
/// `<data-dir>/.ruxd/uploads/`. Entries starting with '.' are never cases.
class LocalCaseStore final : public ICaseStore {
    public:
  /// A `.rux` @p target that does not exist yet is listed anyway; it is
  /// created when first opened, like any project command would.
  /// @throws std::runtime_error when @p target is neither a directory nor a
  ///         `.rux` path, or the data dir cannot be created.
  LocalCaseStore(std::filesystem::path target, std::filesystem::path data_dir);
  ~LocalCaseStore() override;

  std::vector<CaseInfo> list() const override;
  std::optional<CaseInfo> find(std::string_view id) const override;
  bool writable() const override;
  std::filesystem::path staging_dir() const override;
  /// @p created_by is not recorded: local mode has one implicit user.
  CaseInfo create(const std::string &name,
                  std::optional<std::int64_t> created_by = {}) override;
  CaseInfo adopt(const std::string &name,
                 const std::filesystem::path &staged_file,
                 std::optional<std::int64_t> created_by = {}) override;
  CaseInfo update(std::string_view id, const CasePatch &patch) override;
  std::filesystem::path move_to_trash(std::string_view id) override;

  /// The data dir, empty when there is none.
  const std::filesystem::path &data_dir() const noexcept;

    private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

// --- uploads ---------------------------------------------------------------

/// Limits on `.rux` uploads. Crow buffers each request body in memory, so an
/// upload is a sequence of bounded chunks appended to a staging file rather
/// than one request: memory stays at one chunk whatever the file size.
struct UploadLimits {
  /// Largest file accepted. A real scan is a few GB.
  std::uint64_t max_bytes = std::uint64_t{32} << 30; // 32 GiB
  /// Largest single chunk (`PUT /api/v1/uploads/{id}` body).
  std::uint64_t max_chunk_bytes = std::uint64_t{64} << 20; // 64 MiB
  /// Uploads in progress at once, server-wide.
  std::size_t max_sessions = 8;
  /// An unfinished upload untouched this long is discarded (checked on every
  /// begin and on the case registry's sweep).
  std::chrono::seconds idle_ttl{std::chrono::hours(6)};
  /// Free space an upload must leave on the data dir's filesystem, beyond its
  /// own size and that of every upload in progress: adopting a project runs a
  /// migration, which needs room for the WAL.
  std::uint64_t free_space_margin = std::uint64_t{1} << 30; // 1 GiB
};

/// One upload in progress.
struct UploadSession {
  std::string id;   ///< 32 lower-case hex digits.
  std::string name; ///< Name for the case it will become.
  std::uint64_t size = 0;
  std::uint64_t received = 0;
};

/// Chunked `.rux` uploads into a staging dir. Thread-safe; each upload has its
/// own lock, so a disk write of one never holds up another.
///
/// The protocol is resumable (`status()` says where to continue, and a chunk
/// is written at its offset after cutting the file back to what was
/// acknowledged), though today's frontend starts over instead of resuming.
class UploadManager {
    public:
  using Clock = std::chrono::steady_clock;

  UploadManager(std::filesystem::path staging_dir, UploadLimits limits = {});
  ~UploadManager();

  UploadManager(const UploadManager &) = delete;
  UploadManager &operator=(const UploadManager &) = delete;

  const UploadLimits &limits() const noexcept;

  /// Start an upload of @p size bytes. Clears stale staging files first.
  /// @throws HttpError(400) bad name/size, HttpError(413) over max_bytes,
  ///         HttpError(429) too many uploads in progress, HttpError(507) not
  ///         enough free space for it and the uploads already in progress.
  UploadSession begin(const std::string &name, std::uint64_t size);

  /// Append @p bytes at @p offset, which must equal what has been received.
  /// @throws HttpError(404) unknown id, HttpError(409) wrong offset (the
  ///         message carries the expected one), HttpError(413) chunk or total
  ///         too large.
  UploadSession append(std::string_view id, std::uint64_t offset,
                       std::string_view bytes);

  /// The current state of an upload. @throws HttpError(404).
  UploadSession status(std::string_view id) const;

  /// End a complete upload: returns the staged file (the caller validates and
  /// adopts it, or deletes it) and forgets the session.
  /// @throws HttpError(404) unknown, HttpError(409) incomplete.
  std::pair<UploadSession, std::filesystem::path> finish(std::string_view id);

  /// Abandon an upload and delete its staging file. Unknown ids are ignored.
  void abort(std::string_view id);

  /// Drop uploads idle longer than idle_ttl. @return how many.
  std::size_t expire(Clock::time_point now = Clock::now());

    private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

/// True when @p file starts with the SQLite 3 header ("SQLite format 3\0").
bool has_sqlite_header(const std::filesystem::path &file);

/// Why @p file is not a ReUseX project, or "" when it is one: it must be a
/// SQLite database (header) that ReUseX wrote — a `schema_version` table with
/// a version of at least 1, or the pre-versioning `material_passports`
/// table. Read-only; nothing is migrated. Any other SQLite file is refused:
/// adopting it would add ReUseX's tables to someone else's database.
///
/// Not a full integrity check: `PRAGMA quick_check` reads the whole file
/// (minutes for a multi-GB scan) and belongs with the multi-user server's
/// untrusted-upload handling (spec phase S3).
std::string reusex_project_problem(const std::filesystem::path &file);

// --- wire shapes (docs/gui/openapi.yaml: Case, CaseList, Upload) -----------

/// One case: id, name, file_name, created_at, archived, size_bytes,
/// deletable and whether it is @p open right now. Never the server path.
nlohmann::json case_json(const CaseInfo &info, bool open);

/// `GET /api/v1/cases`: `{"cases": [...], "writable": bool, "upload":
/// {"max_bytes", "chunk_bytes"}}`. Not paged: a server's case list is short,
/// and the frontend shows all of it.
nlohmann::json cases_list_json(nlohmann::json cases, bool writable,
                               const UploadLimits &limits);

/// A validated `POST /api/v1/uploads` body.
struct UploadRequest {
  std::string name;
  std::uint64_t size = 0;
};
/// @throws HttpError(400) on a malformed body.
UploadRequest parse_upload_request(std::string_view body);

/// The `offset` query parameter of a chunk. @throws HttpError(400).
std::uint64_t parse_upload_offset(const char *raw);

/// An upload's state, plus the chunk size the client should use.
nlohmann::json upload_json(const UploadSession &session,
                           const UploadLimits &limits);

/// The case id in `/api/v1/cases/<cid>/events`, or nullopt.
std::optional<std::string> case_id_of_events_url(std::string_view url);

} // namespace ruxd::api
