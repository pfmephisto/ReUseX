// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "gui/photo_cache.hpp"

#include <reusex/core/ProjectDB.hpp>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <optional>

namespace rux::gui {

std::string photo_revision(const reusex::ProjectDB &db,
                           const std::string &cloud, std::uint64_t generation) {
  std::string rev = cloud + "|g" + std::to_string(generation);
  const auto log = db.pipeline_log(1); // newest first
  if (!log.empty())
    rev +=
        "|log" + std::to_string(log.front().id) + "@" + log.front().finished_at;
  rev += "|frames" + std::to_string(db.sensor_frame_ids().size());
  return rev;
}

namespace {
bool covers(const InstancePhotos &photos,
            const std::set<std::uint32_t> &wanted) {
  return std::all_of(wanted.begin(), wanted.end(),
                     [&](std::uint32_t id) { return photos.count(id) != 0; });
}
} // namespace

InstancePhotos PhotoEvidenceCache::get(const reusex::ProjectDB &db,
                                       const std::string &cloud,
                                       const std::set<std::uint32_t> &wanted) {
  std::uint64_t generation = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    generation = generation_;
  }
  const auto revision = photo_revision(db, cloud, generation);
  // A copy under the lock: another thread may replace the entry right after.
  auto lookup = [&]() -> std::optional<InstancePhotos> {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = entries_.find(cloud);
    if (it != entries_.end() && it->second.revision == revision &&
        covers(it->second.photos, wanted))
      return it->second.photos;
    return std::nullopt;
  };
  if (auto hit = lookup())
    return std::move(*hit);

  std::lock_guard<std::mutex> compute(compute_mutex_);
  if (auto hit = lookup()) // another caller computed it meanwhile
    return std::move(*hit);

  const auto start = std::chrono::steady_clock::now();
  auto photos = reusex::core::instance_photos(db, cloud, query_);
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - start)
                      .count();
  spdlog::info("Photo evidence for '{}' computed: {} instance(s) in {} ms",
               cloud, photos.size(), ms);

  std::lock_guard<std::mutex> lock(mutex_);
  ++computations_;
  entries_[cloud] = Entry{revision, photos};
  return photos;
}

void PhotoEvidenceCache::invalidate() {
  std::lock_guard<std::mutex> lock(mutex_);
  ++generation_;
  entries_.clear();
}

std::size_t PhotoEvidenceCache::computations() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return computations_;
}

} // namespace rux::gui
