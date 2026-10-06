// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// In-memory cache of the occlusion-aware photo evidence behind
// GET /survey/photos. Computing it decodes the depth image of every frame that
// sees any instance — seconds on a real project (3.5 s on a 3876-frame scan) —
// while its inputs (instance labels, frame poses, depth) change only when a
// stage runs or labels are edited. Survey edits never change it: the cache
// holds per-INSTANCE results and parts are mapped onto them per request.

#include <reusex/core/instance_evidence.hpp>

#include <cstdint>
#include <map>
#include <mutex>
#include <set>
#include <string>

namespace reusex {
class ProjectDB;
}

namespace rux::gui {

using InstancePhotos = std::map<std::uint32_t, reusex::core::PartPhotos>;

/// What the cached evidence of @p cloud depends on, cheaply: the server's
/// geometry generation, the newest pipeline-log entry (id + finish time, so a
/// stage run from the CLI against the same file invalidates too) and the
/// sensor-frame count. Not a content hash: an edit that bypasses both the
/// server and the pipeline log is not seen.
std::string photo_revision(const reusex::ProjectDB &db,
                           const std::string &cloud, std::uint64_t generation);

class PhotoEvidenceCache {
    public:
  explicit PhotoEvidenceCache(reusex::core::PhotoQuery query = {})
      : query_(query) {}

  /// Per-instance photos of @p cloud. Served from the cache when the
  /// revision matches and every id in @p wanted is present (an instance
  /// created since — e.g. from a segmentation — forces a recompute);
  /// otherwise computed with core::instance_photos and stored. Concurrent
  /// misses compute once: the second caller waits and then hits.
  /// @throws as core::instance_photos.
  InstancePhotos get(const reusex::ProjectDB &db, const std::string &cloud,
                     const std::set<std::uint32_t> &wanted = {});

  /// Forget everything: call after the server itself changes geometry
  /// (label edits, a finished pipeline job).
  void invalidate();

  /// How many times get() computed rather than hit (for tests and logs).
  std::size_t computations() const;

    private:
  struct Entry {
    std::string revision;
    InstancePhotos photos;
  };
  reusex::core::PhotoQuery query_;
  mutable std::mutex mutex_; ///< guards entries_, generation_, computations_
  std::mutex compute_mutex_; ///< serialises computations
  std::map<std::string, Entry> entries_;
  std::uint64_t generation_ = 0;
  std::size_t computations_ = 0;
};

} // namespace rux::gui
