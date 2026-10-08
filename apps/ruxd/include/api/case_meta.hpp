// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// What ruxd can say about a case WITHOUT opening it (review of S2, I3).
//
// Opening a case (ProjectRegistry) costs a WAL anchor, a job queue and a slot
// under the open-case cap; the case list must not pay that per card. So:
//  * CaseSummaryCache reads a card's figures — the building record, the
//    survey counts and fractions — over one short read-only connection, and
//    keeps them until the project file (or its WAL) changes;
//  * RenderCache keeps rendered images (the cards' plan thumbnails, the
//    Kortlægning evidence renders) by the same file stamp and the request,
//    within a byte budget.
//
// Framework-free; tested in tests/unit/ruxd_api/test_api_case_meta.cpp.

#include "api/api.hpp"
#include "api/cases.hpp"

#include <nlohmann/json.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace ruxd::api {

/// Changes whenever the project's content may have: the mtime and size of the
/// `.rux` and of its `-wal` (a write lands in the WAL first and touches the
/// main file only at a checkpoint).
struct FileStamp {
  std::int64_t main_mtime = 0;
  std::uintmax_t main_size = 0;
  std::int64_t wal_mtime = 0;
  std::uintmax_t wal_size = 0;
  bool operator==(const FileStamp &) const = default;
};

FileStamp file_stamp(const std::filesystem::path &project);

/// A card's figures, for `Case.summary` in `GET /api/v1/cases`:
/// `{"project": ProjectInfo|null, "survey": SurveySummary|null,
///   "fractions": SurveyFractions|null}` — the same shapes as
/// `GET /cases/{cid}/projects`, `/survey/summary` and `/survey/fractions`.
/// A project on an older schema reports only what it can (no migration
/// happens on a read-only probe); one that cannot be read at all gives null.
nlohmann::json read_case_summary(const std::filesystem::path &project);

/// read_case_summary() cached by FileStamp. Thread-safe.
class CaseSummaryCache {
    public:
  nlohmann::json get(const CaseInfo &info);
  /// Forget one case (it was deleted). Unknown paths are ignored.
  void forget(const std::filesystem::path &project);

    private:
  struct Entry {
    FileStamp stamp;
    nlohmann::json summary;
  };
  std::mutex mutex_;
  std::map<std::filesystem::path, Entry> entries_;
};

/// Rendered images by (project, its FileStamp, request), least recently used
/// out first once over @p budget_bytes. Thread-safe.
class RenderCache {
    public:
  explicit RenderCache(std::size_t budget_bytes = std::size_t{64} << 20);

  std::optional<Blob> get(const std::filesystem::path &project,
                          const std::string &request);
  /// Store @p blob under @p stamp — the file stamp taken BEFORE the project
  /// was read for it, so a write that commits during the render leaves the
  /// entry stale rather than caching the old image under the new stamp
  /// (review N2).
  void put(const std::filesystem::path &project, const std::string &request,
           const FileStamp &stamp, Blob blob);
  std::size_t bytes() const;

    private:
  struct Item {
    std::string key;
    FileStamp stamp;
    Blob blob;
  };
  std::size_t budget_;
  std::size_t bytes_ = 0;
  mutable std::mutex mutex_;
  std::list<Item> lru_; ///< Most recently used first.
  std::map<std::string, std::list<Item>::iterator> index_;
};

} // namespace ruxd::api
