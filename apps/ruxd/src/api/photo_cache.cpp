// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "api/photo_cache.hpp"

#include <reusex/core/ProjectDB.hpp>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <optional>

namespace ruxd::api {

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

std::uint64_t PhotoEvidenceCache::generation() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return generation_;
}

std::shared_ptr<const PhotoEvidenceCache::Entry>
PhotoEvidenceCache::current(const std::string &cloud,
                            const std::string &revision) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = entries_.find(cloud);
  if (it == entries_.end() || it->second->revision != revision)
    return nullptr;
  return it->second;
}

InstancePhotos PhotoEvidenceCache::get(const reusex::ProjectDB &db,
                                       const std::string &cloud,
                                       const std::set<std::uint32_t> &wanted,
                                       const std::atomic<bool> *cancel) {
  const auto revision = photo_revision(db, cloud, generation());
  auto covers = [&](const Entry &entry) {
    return std::all_of(wanted.begin(), wanted.end(), [&](std::uint32_t id) {
      return entry.evidence.count(id) != 0 || entry.absent.count(id) != 0;
    });
  };
  auto photos_of = [](const Entry &entry) {
    InstancePhotos out;
    for (const auto &[id, evidence] : entry.evidence)
      out.emplace(id, evidence.photos());
    return out;
  };

  if (auto hit = current(cloud, revision); hit && covers(*hit))
    return photos_of(*hit);

  std::lock_guard<std::mutex> compute(compute_mutex_);
  // Another caller may have computed it while this one waited.
  if (auto hit = current(cloud, revision); hit && covers(*hit))
    return photos_of(*hit);

  const auto start = std::chrono::steady_clock::now();
  auto entry = std::make_shared<Entry>();
  entry->revision = revision;
  entry->evidence =
      reusex::core::instance_evidence(db, cloud, query_, "cloud", cancel);
  for (const auto id : wanted)
    if (entry->evidence.count(id) == 0)
      entry->absent.insert(id);
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - start)
                      .count();
  spdlog::info("Photo evidence for '{}' computed: {} instance(s) in {} ms",
               cloud, entry->evidence.size(), ms);

  std::lock_guard<std::mutex> lock(mutex_);
  ++computations_;
  entries_[cloud] = entry;
  return photos_of(*entry);
}

std::optional<reusex::core::InstanceEvidence>
PhotoEvidenceCache::peek(const reusex::ProjectDB &db, const std::string &cloud,
                         std::uint32_t instance_id) {
  const auto hit = current(cloud, photo_revision(db, cloud, generation()));
  if (!hit)
    return std::nullopt;
  const auto it = hit->evidence.find(instance_id);
  if (it == hit->evidence.end())
    return std::nullopt;
  return it->second;
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

} // namespace ruxd::api
