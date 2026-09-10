// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "gui/point_lod.hpp"

#include <reusex/core/logging.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace rux::gui {
namespace {

/// Widest stored record this file handles (`PointXYZRGB` and `Normal` are both
/// 16 bytes). Kept inline in an entry so the selection needs no second read.
constexpr size_t kMaxRecord = 16;

/// How far below the estimated answer the grid starts, in dyadic levels.
///
/// The starting edge is *estimated* (see `estimate_edge`) but deliberately
/// biased fine, because the two directions are not symmetric: starting too
/// fine costs a rebuild or two, while starting too coarse would silently
/// return a handful of points for a budget of hundreds of thousands, with
/// nothing in the answer to say so. Four levels is a 16× margin on the edge —
/// comfortably more than the estimate's error in practice, and only ~2
/// rebuilds to walk back up.
constexpr int kInitialUndershootLevels = 4;

/// Fallback edge when the first window is a single point (or degenerate), so
/// there is no diagonal to scale from.
constexpr double kDegenerateVoxel = 1e-6;

/// A voxel address. `int64` per axis rather than a packed key: a very fine
/// initial grid over a georeferenced scan overflows a 21-bit-per-axis packing,
/// and a silently wrapped key would merge two distant voxels.
struct VoxelKey {
  int64_t x = 0;
  int64_t y = 0;
  int64_t z = 0;

  bool operator==(const VoxelKey &) const = default;
};

/// A parent key, `levels` doublings up.
///
/// This is the whole reason the grid is dyadic. For a real coordinate `c` and
/// edge `s`, `floor(floor(c/s) / 2^n) == floor(c / (s * 2^n))` exactly, so a
/// coarser grid's key is the finer key shifted right — no coordinate is
/// re-read and no rounding is re-done. C++20 defines `>>` on a signed integer
/// as an arithmetic shift, which is floor division for negatives too; that
/// matters because half a georeferenced scan can sit at negative coordinates.
VoxelKey parent_of(const VoxelKey &key, int levels) {
  return {key.x >> levels, key.y >> levels, key.z >> levels};
}

struct VoxelKeyHash {
  size_t operator()(const VoxelKey &key) const noexcept {
    // splitmix64 finalizer per axis, folded. The keys of neighbouring voxels
    // differ by 1 in one axis, so a cheap shift-and-xor combiner would pile
    // a scan's worth of surface points into a handful of buckets.
    auto mix = [](uint64_t value) {
      value += 0x9e3779b97f4a7c15ULL;
      value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
      value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
      return value ^ (value >> 31);
    };
    uint64_t hash = mix(static_cast<uint64_t>(key.x));
    hash = mix(hash ^ static_cast<uint64_t>(key.y));
    hash = mix(hash ^ static_cast<uint64_t>(key.z));
    return static_cast<size_t>(hash);
  }
};

/// One kept point: its voxel at the current level, its storage index, and its
/// record bytes.
struct Entry {
  VoxelKey key;
  uint64_t index = 0;
  std::array<uint8_t, kMaxRecord> record{};
};

float read_f32(const uint8_t *at) {
  float value = 0.0F;
  std::memcpy(&value, at, sizeof(value));
  return value;
}

bool finite_position(const uint8_t *record) {
  return std::isfinite(read_f32(record)) &&
         std::isfinite(read_f32(record + 4)) &&
         std::isfinite(read_f32(record + 8));
}

int64_t voxel_coordinate(float value, double edge) {
  return static_cast<int64_t>(std::floor(static_cast<double>(value) / edge));
}

/// A first guess at the voxel edge that keeps `budget` of `total` points.
///
/// Points in a scan lie on surfaces, so a window of @p window_points spanning
/// a box of diagonal @p diagonal has a mean spacing of roughly
/// `diagonal / sqrt(window_points)`. Keeping one point per voxel out of every
/// `total / budget` then wants an edge about `sqrt(total / budget)` times that
/// spacing.
///
/// It is only ever a starting point — the loop that uses it measures the real
/// answer and corrects. What it buys is starting two rebuilds away instead of
/// fifteen: a fixed "very fine" edge is wrong by whatever the scan's scale
/// happens to be, and every factor of two costs a full pass over the kept set.
double estimate_edge(double diagonal, uint64_t window_points, uint64_t total,
                     uint64_t budget) {
  if (diagonal <= 0.0 || window_points == 0)
    return kDegenerateVoxel;
  const double spacing =
      diagonal / std::sqrt(static_cast<double>(window_points));
  const double thinning =
      std::sqrt(static_cast<double>(total) / static_cast<double>(budget));
  const double edge = spacing * thinning /
                      std::exp2(static_cast<double>(kInitialUndershootLevels));
  return edge > 0.0 ? edge : kDegenerateVoxel;
}

/// What one streaming selection pass produced.
struct Pass {
  std::vector<Entry> kept; ///< Ascending by point index.
  double edge = 0.0;       ///< Voxel edge the pass finished at.
  uint64_t skipped_non_finite = 0;
};

/**
 * One streaming pass: walk every stored record once, keeping the
 * lowest-indexed point of each occupied voxel, coarsening whenever the kept
 * set would exceed @p budget.
 *
 * @param start_edge Voxel edge to begin at, or 0 to estimate one from the
 *                   first window's density.
 */
Pass select_pass(const reusex::ProjectDB &db, std::string_view name,
                 uint64_t total, size_t step, uint64_t budget,
                 double start_edge) {
  Pass pass;
  pass.kept.reserve(static_cast<size_t>(budget) + 1);
  std::unordered_set<VoxelKey, VoxelKeyHash> occupied;
  occupied.reserve(pass.kept.capacity());

  double base_edge = start_edge;
  double edge = start_edge;
  // Doublings applied so far, so the reported edge is `base * 2^level` exactly
  // rather than a product accumulated one multiplication at a time.
  int level = 0;

  const auto rebuild = [&](int levels) {
    level += levels;
    edge = base_edge * std::exp2(static_cast<double>(level));
    occupied.clear();
    std::vector<Entry> merged;
    merged.reserve(pass.kept.size());
    // `kept` is ascending by index, and the first entry to claim a parent
    // voxel therefore carries the lowest index in it. That is the same
    // tie-break the insert loop uses, so a coarsened result is identical to
    // one selected at that level from scratch.
    for (auto &entry : pass.kept) {
      entry.key = parent_of(entry.key, levels);
      if (occupied.insert(entry.key).second)
        merged.push_back(entry);
    }
    pass.kept.swap(merged);
  };

  const auto coarsen = [&] {
    // Occupied voxels of a scan scale roughly as the inverse *square* of the
    // edge (points lie on surfaces, not through volumes), so the jump that
    // lands near the budget is about the square root of the overshoot. Below
    // an overshoot of 4× that says "one level", which is also the smallest
    // step the dyadic merge can take.
    //
    // **The loop never takes a bigger step than the ratio calls for.**
    // Jumping ahead — to the edge `estimate_edge()` predicted, or by
    // escalating when a rebuild fails to pay for itself — converges in fewer
    // passes over the kept set and was tried; both overshoot the coarsest
    // level that still fits, and a dyadic level is a ~4× step, so overshooting
    // answers a 200 000-point budget with 45 000 points when 183 000 were
    // available. Stopping at the first level inside the budget is what makes
    // the answer the best this lattice can offer; moving the lattice itself is
    // the refinement pass's job, not this loop's.
    while (pass.kept.size() > budget && level < 62) {
      const double ratio =
          static_cast<double>(pass.kept.size()) / static_cast<double>(budget);
      const int levels =
          static_cast<int>(std::ceil(std::log2(std::sqrt(ratio))));
      rebuild(std::clamp(levels, 1, 62 - level));
    }
  };

  for (uint64_t start = 0; start < total; start += kLodStreamChunk) {
    const auto window = db.point_cloud_page(name, start, kLodStreamChunk);
    if (window.count == 0)
      break;

    const uint8_t *records = window.data.data();

    if (base_edge == 0.0) {
      // Scale the starting edge off the first window rather than assuming
      // metres: nothing in the store says what unit a cloud is in.
      float lo[3] = {std::numeric_limits<float>::infinity(),
                     std::numeric_limits<float>::infinity(),
                     std::numeric_limits<float>::infinity()};
      float hi[3] = {-std::numeric_limits<float>::infinity(),
                     -std::numeric_limits<float>::infinity(),
                     -std::numeric_limits<float>::infinity()};
      for (uint64_t i = 0; i < window.count; ++i) {
        const uint8_t *record = records + i * step;
        if (!finite_position(record))
          continue;
        for (int axis = 0; axis < 3; ++axis) {
          const float value = read_f32(record + 4 * axis);
          lo[axis] = std::min(lo[axis], value);
          hi[axis] = std::max(hi[axis], value);
        }
      }
      double diagonal = 0.0;
      for (int axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(lo[axis]) || !std::isfinite(hi[axis])) {
          diagonal = 0.0;
          break;
        }
        const double span = static_cast<double>(hi[axis]) - lo[axis];
        diagonal += span * span;
      }
      base_edge =
          estimate_edge(std::sqrt(diagonal), window.count, total, budget);
      edge = base_edge;
    }

    for (uint64_t i = 0; i < window.count; ++i) {
      const uint8_t *record = records + i * step;
      if (!finite_position(record)) {
        ++pass.skipped_non_finite;
        continue;
      }
      const VoxelKey key{voxel_coordinate(read_f32(record), edge),
                         voxel_coordinate(read_f32(record + 4), edge),
                         voxel_coordinate(read_f32(record + 8), edge)};
      if (!occupied.insert(key).second)
        continue; // The voxel already holds a lower-indexed point.

      Entry entry;
      entry.key = key;
      entry.index = window.offset + i;
      std::memcpy(entry.record.data(), record, step);
      pass.kept.push_back(entry);

      if (pass.kept.size() > budget)
        coarsen();
    }
  }

  pass.edge = edge;
  return pass;
}

} // namespace

bool lod_supports(std::string_view point_type) {
  return point_type == "PointXYZRGB" || point_type == "PointXYZ";
}

LodSelection voxel_lod(const reusex::ProjectDB &db, std::string_view name,
                       uint64_t max_points) {
  if (max_points == 0)
    throw std::runtime_error("voxel LOD needs a budget of at least 1 point");

  const std::string type = db.point_cloud_type(name);
  if (!lod_supports(type))
    throw std::runtime_error("cloud '" + std::string(name) + "' is a '" + type +
                             "' cloud and carries no positions to voxelise");

  LodSelection result;

  // A zero-length read is the cheap way to learn `total` and `point_step`
  // without touching a blob.
  const auto probe = db.point_cloud_page(name, 0, 0);
  result.page.point_type = probe.point_type;
  result.page.point_step = probe.point_step;
  result.page.offset = 0;
  result.page.total = probe.total;

  if (probe.point_step == 0 || probe.point_step > kMaxRecord)
    throw std::runtime_error("cloud '" + std::string(name) +
                             "' has an unusable point_step of " +
                             std::to_string(probe.point_step));

  const size_t step = probe.point_step;

  // Under budget: there is nothing to choose between, and subsampling a cloud
  // that already fits would throw away points for no reason. The caller must
  // then serve it as an ordinary complete page (`subsampled` stays false).
  if (probe.total <= max_points) {
    auto whole = db.point_cloud_page(name, 0, probe.total);
    result.indices.resize(static_cast<size_t>(whole.count));
    for (size_t i = 0; i < result.indices.size(); ++i)
      result.indices[i] = i;
    result.page = std::move(whole);
    return result;
  }

  auto pass = select_pass(db, name, probe.total, step, max_points, 0.0);

  // ---- one measured refinement pass ----
  //
  // A dyadic grid can only offer counts a factor of ~4 apart, and where those
  // land relative to the budget is luck: measured on a 1.2 M-point office
  // scan, the same code answered a 100 000-point budget with 97 807 points and
  // a 200 000-point one with 45 720. Handing back 23 % of what was asked for
  // is not a subsample, it is a different picture of the room.
  //
  // The first pass measures what the second needs. Having selected `n` points
  // at edge `e`, the surface model that drives `coarsen()` inverts: the edge
  // that would have filled the budget is `e * sqrt(n / budget)`. Starting the
  // second pass *there* moves the whole lattice rather than stepping along the
  // old one, so it can land where no level of the first pass could.
  //
  // It aims at 80 % of the budget rather than at all of it, and that margin is
  // load-bearing. The `n ∝ e^-2` model is only approximate, so aiming exactly
  // at the budget lands over it about half the time — and a second pass that
  // overflows coarsens back onto a lattice no better than the first one, which
  // is the whole cost with none of the benefit. Undershooting by a fifth is
  // cheap; overshooting wastes the pass.
  //
  // Bounded at exactly two passes, and only paid when the first one wasted a
  // quarter of the budget. `coarsen()` still runs inside the second pass, so a
  // model that guessed too fine is corrected rather than allowed to blow the
  // bound — which is also why the better of the two answers is taken rather
  // than the newer one.
  if (!pass.kept.empty() && pass.kept.size() * 4 < max_points * 3) {
    const double aim = 0.8 * static_cast<double>(max_points);
    const double refined =
        pass.edge * std::sqrt(static_cast<double>(pass.kept.size()) / aim);
    if (refined > 0.0 && refined < pass.edge) {
      auto second =
          select_pass(db, name, probe.total, step, max_points, refined);
      if (second.kept.size() > pass.kept.size())
        pass = std::move(second);
    }
  }

  auto &kept = pass.kept;
  result.skipped_non_finite = pass.skipped_non_finite;

  if (result.skipped_non_finite > 0)
    reusex::warn("voxel LOD on '{}': skipped {} of {} points with a "
                 "non-finite coordinate",
                 name, result.skipped_non_finite, probe.total);

  // Storage order, which is what every other page on this endpoint promises
  // and what makes a sibling gather a single forward pass.
  std::sort(kept.begin(), kept.end(),
            [](const Entry &a, const Entry &b) { return a.index < b.index; });

  result.subsampled = true;
  result.voxel_size = pass.edge;
  result.page.count = kept.size();
  result.page.data.resize(kept.size() * step);
  result.indices.resize(kept.size());
  for (size_t i = 0; i < kept.size(); ++i) {
    std::memcpy(result.page.data.data() + i * step, kept[i].record.data(),
                step);
    result.indices[i] = kept[i].index;
  }

  if (kept.empty())
    reusex::warn("voxel LOD on '{}' selected no points from {} stored", name,
                 probe.total);

  return result;
}

reusex::ProjectDB::CloudPage
gather_points(const reusex::ProjectDB &db, std::string_view name,
              const std::vector<uint64_t> &indices) {
  const auto probe = db.point_cloud_page(name, 0, 0);

  reusex::ProjectDB::CloudPage out;
  out.point_type = probe.point_type;
  out.point_step = probe.point_step;
  out.offset = 0;
  out.total = probe.total;
  out.count = indices.size();

  if (indices.empty())
    return out;
  if (indices.back() >= probe.total)
    throw std::runtime_error(
        "cloud '" + std::string(name) + "' holds " +
        std::to_string(probe.total) + " points, so index " +
        std::to_string(indices.back()) +
        " is out of range — it is not index-aligned with the LOD source");

  const size_t step = probe.point_step;
  out.data.resize(indices.size() * step);

  // One forward pass. `indices` is ascending, so a cursor into it walks in
  // lockstep with the windows and never seeks backwards; the alternative — a
  // page read per index — would open a blob a quarter of a million times.
  size_t cursor = 0;
  for (uint64_t start = 0; start < probe.total && cursor < indices.size();
       start += kLodStreamChunk) {
    if (indices[cursor] >= start + kLodStreamChunk)
      continue; // No wanted index falls in this window; skip the read entirely.

    const auto window = db.point_cloud_page(name, start, kLodStreamChunk);
    if (window.count == 0)
      break;
    const uint64_t end = window.offset + window.count;
    while (cursor < indices.size() && indices[cursor] < end) {
      const uint64_t local = indices[cursor] - window.offset;
      std::memcpy(out.data.data() + cursor * step,
                  window.data.data() + local * step, step);
      ++cursor;
    }
  }

  if (cursor != indices.size())
    throw std::runtime_error("cloud '" + std::string(name) + "' yielded only " +
                             std::to_string(cursor) + " of " +
                             std::to_string(indices.size()) +
                             " requested points");

  return out;
}

} // namespace rux::gui
