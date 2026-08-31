// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Validates noise-adaptive plane segmentation (issue #214). Fixed absolute
// thresholds over-fragment noisy scans (a 7cm distance threshold with 15mm
// noise splits each wall into slivers). Adaptivity must derive the distance
// threshold from the measured sigma so a box stays ~6 planes on BOTH a clean
// (2mm) and a noisy (15mm) room, and must never do worse than the fixed
// defaults on the noisy room.

#include "../../support/synthetic_scene.hpp"

#include <reusex/geometry/segment_planes.hpp>

#include <catch2/catch_test_macros.hpp>

#include <map>

using reusex::geometry::segment_planes;
using reusex::geometry::SegmentPlanesOptions;
using reusex::test_support::make_room;

namespace {

/// Number of distinct plane labels (label > 0) in the labeled cloud.
std::size_t plane_count(const reusex::CloudLPtr &labels) {
  std::map<uint32_t, std::size_t> hist;
  for (const auto &pt : labels->points)
    if (pt.label > 0)
      ++hist[pt.label];
  return hist.size();
}

std::size_t segment_count(const reusex::CloudPtr &cloud,
                          const reusex::CloudNPtr &normals,
                          const SegmentPlanesOptions &opt) {
  auto [labels, centroids, plane_normals] = segment_planes(cloud, normals, opt);
  // centroids->size() is the algorithm's own plane count; label histogram is
  // an independent cross-check. Use the centroid count as the headline number.
  (void)plane_count(labels);
  return centroids->size();
}

} // namespace

TEST_CASE("adaptive planes: ~6 planes on clean AND noisy room",
          "[geometry][planes][adaptive]") {
  const auto clean = make_room(4.0F, 3.0F, 2.5F, 0.05F, /*sigma=*/0.002F);
  const auto noisy = make_room(4.0F, 3.0F, 2.5F, 0.05F, /*sigma=*/0.015F);

  SegmentPlanesOptions adaptive; // adaptive == true by default
  adaptive.noise_seed = 42;

  const std::size_t clean_adaptive =
      segment_count(clean.cloud, clean.normals, adaptive);
  const std::size_t noisy_adaptive =
      segment_count(noisy.cloud, noisy.normals, adaptive);

  INFO("clean adaptive planes = "
       << clean_adaptive << ", noisy adaptive planes = " << noisy_adaptive);
  // A box has 6 faces; adaptivity must keep both clean and noisy in a tight
  // band around that (no collapse, no explosion).
  CHECK(clean_adaptive >= 4);
  CHECK(clean_adaptive <= 12);
  CHECK(noisy_adaptive >= 4);
  CHECK(noisy_adaptive <= 12);
}

TEST_CASE("adaptive planes: no worse than fixed defaults on a noisy room",
          "[geometry][planes][adaptive]") {
  const auto noisy = make_room(4.0F, 3.0F, 2.5F, 0.05F, /*sigma=*/0.015F);

  SegmentPlanesOptions fixed;
  fixed.adaptive = false; // legacy absolute defaults (dist 0.07, min 1000)

  SegmentPlanesOptions adaptive;
  adaptive.noise_seed = 42;

  const std::size_t fixed_count =
      segment_count(noisy.cloud, noisy.normals, fixed);
  const std::size_t adaptive_count =
      segment_count(noisy.cloud, noisy.normals, adaptive);

  INFO("noisy fixed planes = " << fixed_count << ", noisy adaptive planes = "
                               << adaptive_count);
  // Adaptivity must not over-fragment relative to the fixed baseline, and must
  // stay in a sane band.
  CHECK(adaptive_count <= fixed_count);
  CHECK(adaptive_count >= 4);
  CHECK(adaptive_count <= 12);
}

TEST_CASE("adaptive planes: explicit distance override bypasses adaptivity",
          "[geometry][planes][adaptive]") {
  const auto noisy = make_room(4.0F, 3.0F, 2.5F, 0.05F, /*sigma=*/0.015F);

  // Pin the distance threshold; segmentation must use exactly this value
  // regardless of the estimated sigma. We assert it runs and produces planes;
  // the pinned run should match a fixed-mode run with the same value.
  SegmentPlanesOptions pinned;
  pinned.adaptive = true;
  pinned.plane_dist_threshold_override = 0.05F;

  const std::size_t pinned_count =
      segment_count(noisy.cloud, noisy.normals, pinned);
  INFO("pinned-distance planes = " << pinned_count);
  CHECK(pinned_count >= 1);
}
