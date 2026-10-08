// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "api/case_meta.hpp"

#include "api/survey.hpp"

#include <reusex/core/ProjectDB.hpp>

#include <spdlog/spdlog.h>

#include <system_error>
#include <utility>

namespace ruxd::api {
namespace fs = std::filesystem;
using json = nlohmann::json;

namespace {

std::pair<std::int64_t, std::uintmax_t> stat_of(const fs::path &file) {
  std::error_code ec;
  const auto time = fs::last_write_time(file, ec);
  if (ec)
    return {0, 0};
  const auto size = fs::file_size(file, ec);
  return {time.time_since_epoch().count(), ec ? 0 : size};
}

} // namespace

FileStamp file_stamp(const fs::path &project) {
  FileStamp stamp;
  std::tie(stamp.main_mtime, stamp.main_size) = stat_of(project);
  std::tie(stamp.wal_mtime, stamp.wal_size) =
      stat_of(project.string() + "-wal");
  return stamp;
}

json read_case_summary(const fs::path &project) {
  std::error_code ec;
  if (!fs::exists(project, ec))
    return nullptr; // Not created yet: nothing to say.
  try {
    reusex::ProjectDB db(project, /*readOnly=*/true);
    json out{{"project", nullptr}, {"survey", nullptr}, {"fractions", nullptr}};
    const auto records = projects_json(db, Params{});
    if (records.contains("projects") && !records["projects"].empty())
      out["project"] = records["projects"][0];
    // A read-only probe never migrates, and the survey tables are recent: an
    // older project shows its record only, until someone opens it.
    if (db.schema_version() >= reusex::ProjectDB::latest_schema_version()) {
      out["survey"] = survey_summary_json(db);
      try {
        out["fractions"] = survey_fractions_json(db);
      } catch (const std::exception &e) {
        spdlog::debug("No fractions for {}: {}", project.filename().string(),
                      e.what());
      }
    }
    return out;
  } catch (const std::exception &e) {
    spdlog::debug("Could not read the card of {}: {}",
                  project.filename().string(), e.what());
    return nullptr;
  }
}

json CaseSummaryCache::get(const CaseInfo &info) {
  const FileStamp stamp = file_stamp(info.path);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = entries_.find(info.path);
    if (it != entries_.end() && it->second.stamp == stamp)
      return it->second.summary;
  }
  json summary = read_case_summary(info.path); // Outside the lock.
  // A failed read (a lock held past the busy timeout, say) is not cached.
  if (!summary.is_null()) {
    std::lock_guard<std::mutex> lock(mutex_);
    entries_[info.path] = Entry{stamp, summary};
  }
  return summary;
}

void CaseSummaryCache::forget(const fs::path &project) {
  std::lock_guard<std::mutex> lock(mutex_);
  entries_.erase(project);
}

RenderCache::RenderCache(std::size_t budget_bytes) : budget_(budget_bytes) {}

std::optional<Blob> RenderCache::get(const fs::path &project,
                                     const std::string &request) {
  const std::string key = project.string() + '\n' + request;
  const FileStamp stamp = file_stamp(project);
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = index_.find(key);
  if (it == index_.end())
    return std::nullopt;
  if (!(it->second->stamp == stamp)) { // The project changed: stale.
    bytes_ -= it->second->blob.data.size();
    lru_.erase(it->second);
    index_.erase(it);
    return std::nullopt;
  }
  lru_.splice(lru_.begin(), lru_, it->second);
  return it->second->blob;
}

void RenderCache::put(const fs::path &project, const std::string &request,
                      Blob blob) {
  const std::size_t size = blob.data.size();
  if (size > budget_)
    return;
  const std::string key = project.string() + '\n' + request;
  const FileStamp stamp = file_stamp(project);
  std::lock_guard<std::mutex> lock(mutex_);
  if (auto it = index_.find(key); it != index_.end()) {
    bytes_ -= it->second->blob.data.size();
    lru_.erase(it->second);
    index_.erase(it);
  }
  lru_.push_front(Item{key, stamp, std::move(blob)});
  index_[key] = lru_.begin();
  bytes_ += size;
  while (bytes_ > budget_ && !lru_.empty()) {
    bytes_ -= lru_.back().blob.data.size();
    index_.erase(lru_.back().key);
    lru_.pop_back();
  }
}

std::size_t RenderCache::bytes() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return bytes_;
}

} // namespace ruxd::api
