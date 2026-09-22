// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "gui/point_lod.hpp"

#include <reusex/core/logging.hpp>
#include <reusex/geometry/morton.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

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

  // Bit-reversed Morton storage: the cloud was sorted by reverse_bits30(code)
  // so the coarsest-level octant bits are most-significant in the sort key.
  // Any prefix therefore visits all spatial octants before refining any single
  // one — it is a spatially stratified uniform sample, not a corner.  Skip
  // the two-pass voxel algorithm and return a contiguous prefix page.
  //
  // voxel_size stays 0.0 — the same sentinel the under-budget path uses for
  // "no voxel grid was applied". subsampled==true tells the caller to advertise
  // LOD on the wire.
  if (db.point_cloud_storage_order(name) == "morton_10bit_bitrev") {
    auto page = db.point_cloud_page(name, 0, max_points);
    result.page = std::move(page);
    result.indices.resize(static_cast<size_t>(result.page.count));
    for (size_t i = 0; i < result.indices.size(); ++i)
      result.indices[i] = static_cast<uint64_t>(i);
    result.subsampled = true;
    // voxel_size = 0.0 (default): prefix read, no voxel grid.
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

namespace {

// 30-bit bit-reversed Morton sort key for (x,y,z) quantised in [lo, hi].
// Uses the shared helpers from <reusex/geometry/morton.hpp> so the computation
// stays identical to reconstruct.cpp's sort and can never silently diverge.
uint32_t morton_sort_key(float x, float y, float z, const float lo[3],
                         const float range[3]) {
  namespace gm = reusex::geometry;
  constexpr uint32_t kMax10 = 1023u;
  const auto clamp01 = [](float v) {
    return v < 0.f ? 0.f : (v > 1.f ? 1.f : v);
  };
  const uint32_t xi =
      static_cast<uint32_t>(clamp01((x - lo[0]) / range[0]) * kMax10);
  const uint32_t yi =
      static_cast<uint32_t>(clamp01((y - lo[1]) / range[1]) * kMax10);
  const uint32_t zi =
      static_cast<uint32_t>(clamp01((z - lo[2]) / range[2]) * kMax10);
  return gm::morton_reverse_bits30(gm::morton_expand3(xi) |
                                   (gm::morton_expand3(yi) << 1u) |
                                   (gm::morton_expand3(zi) << 2u));
}

} // namespace

std::vector<uint8_t> compute_tile_index(const reusex::ProjectDB &db,
                                        std::string_view name,
                                        uint32_t tile_bits) {
  const auto probe = db.point_cloud_page(name, 0, 0);
  const uint64_t total = probe.total;
  const size_t step = probe.point_step;

  if (step == 0 || step > kMaxRecord || total == 0)
    return {};

  const uint32_t K = 1u << tile_bits;
  if (total < K)
    return {}; // cloud too small for K distinct tiles

  // Pass 1: compute cloud bbox (needed for sort_key normalization).
  float lo[3] = {std::numeric_limits<float>::infinity(),
                 std::numeric_limits<float>::infinity(),
                 std::numeric_limits<float>::infinity()};
  float hi[3] = {-std::numeric_limits<float>::infinity(),
                 -std::numeric_limits<float>::infinity(),
                 -std::numeric_limits<float>::infinity()};
  for (uint64_t start = 0; start < total; start += kLodStreamChunk) {
    const auto window = db.point_cloud_page(name, start, kLodStreamChunk);
    if (window.count == 0)
      break;
    for (uint64_t i = 0; i < window.count; ++i) {
      const uint8_t *rec = window.data.data() + i * step;
      if (!finite_position(rec))
        continue;
      for (int ax = 0; ax < 3; ++ax) {
        const float v = read_f32(rec + 4 * ax);
        if (v < lo[ax])
          lo[ax] = v;
        if (v > hi[ax])
          hi[ax] = v;
      }
    }
  }
  // Avoid division by zero for degenerate clouds.
  float rng[3];
  for (int ax = 0; ax < 3; ++ax)
    rng[ax] = (hi[ax] - lo[ax]) > 1e-9f ? (hi[ax] - lo[ax]) : 1e-9f;

  // Pass 2: assign each point to tile (sort_key & (K-1)), update tight AABB.
  // The bottom tile_bits bits of the 30-bit bit-reversed Morton sort_key are
  // the TOP tile_bits bits of the plain Morton code — the coarsest spatial
  // octant bits. Points sharing these bits occupy one spatial cell.
  std::vector<TileInfo> tiles(K);
  for (auto &t : tiles) {
    t.min[0] = t.min[1] = t.min[2] = std::numeric_limits<float>::infinity();
    t.max[0] = t.max[1] = t.max[2] = -std::numeric_limits<float>::infinity();
    t.count = 0;
  }

  const uint32_t mask = K - 1;
  for (uint64_t start = 0; start < total; start += kLodStreamChunk) {
    const auto window = db.point_cloud_page(name, start, kLodStreamChunk);
    if (window.count == 0)
      break;

    for (uint64_t i = 0; i < window.count; ++i) {
      const uint8_t *rec = window.data.data() + i * step;
      if (!finite_position(rec))
        continue;

      const float x = read_f32(rec);
      const float y = read_f32(rec + 4);
      const float z = read_f32(rec + 8);
      const uint32_t sk = morton_sort_key(x, y, z, lo, rng);
      const uint32_t tile = sk & mask;
      TileInfo &t = tiles[tile];
      t.count += 1;
      if (x < t.min[0])
        t.min[0] = x;
      if (y < t.min[1])
        t.min[1] = y;
      if (z < t.min[2])
        t.min[2] = z;
      if (x > t.max[0])
        t.max[0] = x;
      if (y > t.max[1])
        t.max[1] = y;
      if (z > t.max[2])
        t.max[2] = z;
    }
  }

  // Empty tiles: degenerate bbox at origin.
  for (auto &t : tiles) {
    if (t.count == 0) {
      t.min[0] = t.min[1] = t.min[2] = 0.0f;
      t.max[0] = t.max[1] = t.max[2] = 0.0f;
    }
  }

  // Serialize: fixed-size header (v2 includes bbox) + per-tile data.
  TileIndexHeader hdr{};
  hdr.magic = kTileIndexMagic;
  hdr.version = 2u;
  hdr.tile_bits = tile_bits;
  hdr.point_count =
      static_cast<uint32_t>(total > UINT32_MAX ? UINT32_MAX : total);
  for (int ax = 0; ax < 3; ++ax) {
    hdr.lo[ax] = lo[ax];
    hdr.hi[ax] = hi[ax];
  }
  const size_t blob_size = sizeof(TileIndexHeader) + K * sizeof(TileInfo);
  std::vector<uint8_t> blob(blob_size);
  std::memcpy(blob.data(), &hdr, sizeof(hdr));
  std::memcpy(blob.data() + sizeof(hdr), tiles.data(), K * sizeof(TileInfo));
  return blob;
}

std::pair<TileIndexHeader, std::vector<TileInfo>>
parse_tile_index(const std::vector<uint8_t> &blob) {
  if (blob.size() < sizeof(TileIndexHeader))
    throw std::runtime_error("tile index blob too short");

  TileIndexHeader hdr;
  std::memcpy(&hdr, blob.data(), sizeof(hdr));
  if (hdr.magic != kTileIndexMagic)
    throw std::runtime_error("tile index blob has wrong magic");
  if (hdr.version != 2u)
    throw std::runtime_error(
        "unsupported tile index version " + std::to_string(hdr.version) +
        " (expected 2; re-run rux create clouds to rebuild)");
  if (hdr.tile_bits == 0 || hdr.tile_bits > 20)
    throw std::runtime_error("tile_bits out of range");

  const uint32_t K = 1u << hdr.tile_bits;
  const size_t expected = sizeof(TileIndexHeader) + K * sizeof(TileInfo);
  if (blob.size() != expected)
    throw std::runtime_error("tile index blob size mismatch");

  std::vector<TileInfo> tiles(K);
  std::memcpy(tiles.data(), blob.data() + sizeof(hdr), K * sizeof(TileInfo));
  return {hdr, std::move(tiles)};
}

reusex::ProjectDB::CloudPage gather_tile_points(const reusex::ProjectDB &db,
                                                std::string_view name,
                                                const TileIndexHeader &hdr,
                                                uint32_t tile_id, uint64_t skip,
                                                uint64_t limit) {
  const uint32_t K = 1u << hdr.tile_bits;
  if (tile_id >= K)
    throw std::runtime_error("tile_id " + std::to_string(tile_id) +
                             " >= K=" + std::to_string(K));

  const auto probe = db.point_cloud_page(name, 0, 0);
  const size_t step = probe.point_step;
  if (step == 0 || step > kMaxRecord)
    throw std::runtime_error("unusable point_step for cloud '" +
                             std::string(name) + "'");

  // Precompute range clamped to avoid division by zero.
  float rng[3];
  for (int ax = 0; ax < 3; ++ax)
    rng[ax] =
        (hdr.hi[ax] - hdr.lo[ax]) > 1e-9f ? (hdr.hi[ax] - hdr.lo[ax]) : 1e-9f;

  const uint32_t mask = K - 1;
  reusex::ProjectDB::CloudPage out;
  out.point_type = probe.point_type;
  out.point_step = step;
  out.offset = 0;
  out.total = probe.total;
  out.count = 0;

  uint64_t skipped = 0;
  // O(N) scan: collect points whose sort_key & mask == tile_id, applying
  // skip/limit. Within a tile, storage order is ascending sort_key, which is
  // a bit-reversed sub-octant ordering — so a prefix is a stratified sample.
  for (uint64_t start = 0; start < probe.total; start += kLodStreamChunk) {
    const auto window = db.point_cloud_page(name, start, kLodStreamChunk);
    if (window.count == 0)
      break;
    for (uint64_t i = 0; i < window.count; ++i) {
      const uint8_t *rec = window.data.data() + i * step;
      if (!finite_position(rec))
        continue;
      const float x = read_f32(rec);
      const float y = read_f32(rec + 4);
      const float z = read_f32(rec + 8);
      if ((morton_sort_key(x, y, z, hdr.lo, rng) & mask) != tile_id)
        continue;
      if (skipped < skip) {
        ++skipped;
        continue;
      }
      out.data.insert(out.data.end(), rec, rec + step);
      ++out.count;
      if (limit > 0 && out.count >= limit)
        return out;
    }
  }
  return out;
}

std::vector<uint64_t> gather_tile_indices(const reusex::ProjectDB &db,
                                          std::string_view name,
                                          const TileIndexHeader &hdr,
                                          uint32_t tile_id, uint64_t skip,
                                          uint64_t limit) {
  const uint32_t K = 1u << hdr.tile_bits;
  if (tile_id >= K)
    throw std::runtime_error("tile_id " + std::to_string(tile_id) +
                             " >= K=" + std::to_string(K));

  const auto probe = db.point_cloud_page(name, 0, 0);
  const size_t step = probe.point_step;
  if (step == 0 || step > kMaxRecord)
    throw std::runtime_error("unusable point_step for cloud '" +
                             std::string(name) + "'");

  float rng[3];
  for (int ax = 0; ax < 3; ++ax)
    rng[ax] =
        (hdr.hi[ax] - hdr.lo[ax]) > 1e-9f ? (hdr.hi[ax] - hdr.lo[ax]) : 1e-9f;

  const uint32_t mask = K - 1;
  std::vector<uint64_t> indices;
  uint64_t skipped = 0;
  for (uint64_t start = 0; start < probe.total; start += kLodStreamChunk) {
    const auto window = db.point_cloud_page(name, start, kLodStreamChunk);
    if (window.count == 0)
      break;
    for (uint64_t i = 0; i < window.count; ++i) {
      const uint8_t *rec = window.data.data() + i * step;
      if (!finite_position(rec))
        continue;
      const float x = read_f32(rec);
      const float y = read_f32(rec + 4);
      const float z = read_f32(rec + 8);
      if ((morton_sort_key(x, y, z, hdr.lo, rng) & mask) != tile_id)
        continue;
      if (skipped < skip) {
        ++skipped;
        continue;
      }
      indices.push_back(start + i);
      if (limit > 0 && static_cast<uint64_t>(indices.size()) >= limit)
        return indices;
    }
  }
  return indices; // ascending by construction — forward iteration
}

} // namespace rux::gui
