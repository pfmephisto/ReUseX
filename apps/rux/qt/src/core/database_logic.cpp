// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/database_logic.hpp>
#include <set>
#include <tuple>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <iterator>
#include <numeric>

namespace rux::qt {

// ----------------------------------------------------------- frame pair --

void FramePair::reset(std::vector<int> ids) {
  std::sort(ids.begin(), ids.end());
  ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
  ids_ = std::move(ids);
  if (ids_.empty()) {
    a_ = b_ = -1;
    return;
  }
  a_ = 0;
  b_ = ids_.size() > 1 ? 1 : 0;
}

int FramePair::a_id() const { return a_ < 0 ? -1 : ids_[a_]; }
int FramePair::b_id() const { return b_ < 0 ? -1 : ids_[b_]; }

bool FramePair::set(int &slot, int index, int size) {
  if (index < 0 || index >= size || index == slot)
    return false;
  slot = index;
  return true;
}

bool FramePair::step_a(int delta) {
  if (empty())
    return false;
  return set(a_, std::clamp(a_ + delta, 0, size() - 1), size());
}

bool FramePair::step_b(int delta) {
  if (empty())
    return false;
  return set(b_, std::clamp(b_ + delta, 0, size() - 1), size());
}

bool FramePair::set_a_index(int index) { return set(a_, index, size()); }
bool FramePair::set_b_index(int index) { return set(b_, index, size()); }

bool FramePair::set_a_id(int id) {
  const int i = index_of(id);
  return i >= 0 && (set(a_, i, size()) || a_ == i);
}

bool FramePair::set_b_id(int id) {
  const int i = index_of(id);
  return i >= 0 && (set(b_, i, size()) || b_ == i);
}

int FramePair::index_of(int id) const {
  const auto it = std::lower_bound(ids_.begin(), ids_.end(), id);
  if (it == ids_.end() || *it != id)
    return -1;
  return static_cast<int>(it - ids_.begin());
}

int FramePair::nearest_index(int id) const {
  if (ids_.empty())
    return -1;
  const auto it = std::lower_bound(ids_.begin(), ids_.end(), id);
  if (it == ids_.begin())
    return 0;
  if (it == ids_.end())
    return size() - 1;
  const auto prev = std::prev(it);
  // Ties go to the lower id.
  return static_cast<int>((id - *prev <= *it - id ? prev : it) - ids_.begin());
}

BrowserKey browser_key(ArrowKey key, bool shift, bool text_focus) {
  if (text_focus)
    return BrowserKey::none;
  switch (key) {
  case ArrowKey::left:
    return shift ? BrowserKey::b_prev : BrowserKey::a_prev;
  case ArrowKey::right:
    return shift ? BrowserKey::b_next : BrowserKey::a_next;
  case ArrowKey::other:
    break;
  }
  return BrowserKey::none;
}

int browser_step(bool ctrl) { return ctrl ? 10 : 1; }

// ------------------------------------------------------- pending edits --

bool is_edge_type(std::string_view type) {
  return type == "odometry" || type == "loop_closure" || type == "panorama";
}

std::string edge_type_da(std::string_view type) {
  if (type == "odometry")
    return "Odometri";
  if (type == "loop_closure")
    return "Løkkelukning";
  if (type == "panorama")
    return "Panorama";
  return std::string(type);
}

double weight_from_icp_fitness(double rms_m) {
  constexpr double kFloorM = 0.01;
  const double sigma =
      std::isfinite(rms_m) ? std::max(rms_m, kFloorM) : kFloorM;
  return 1.0 / (sigma * sigma);
}

void PendingEdgeEdits::set_base(std::vector<EdgeRecord> edges) {
  base_ = std::move(edges);
  discard();
}

bool PendingEdgeEdits::removed(const EdgeKey &k) const {
  return std::find(removes_.begin(), removes_.end(), k) != removes_.end();
}

bool PendingEdgeEdits::in_base(const EdgeKey &k) const {
  return std::any_of(base_.begin(), base_.end(),
                     [&](const EdgeRecord &e) { return e.key == k; });
}

PendingEdgeEdits::Result PendingEdgeEdits::add(const EdgeRecord &edge) {
  const EdgeKey &k = edge.key;
  if (k.from == k.to || !is_edge_type(k.type) || !std::isfinite(edge.weight) ||
      edge.weight <= 0.0)
    return Result::invalid;
  if (removed(k)) {
    removes_.erase(std::find(removes_.begin(), removes_.end(), k));
    return Result::restored;
  }
  if (in_base(k) ||
      std::any_of(adds_.begin(), adds_.end(),
                  [&](const EdgeRecord &e) { return e.key == k; }))
    return Result::duplicate;
  adds_.push_back(edge);
  return Result::added;
}

PendingEdgeEdits::Result PendingEdgeEdits::remove(const EdgeKey &key) {
  const auto staged =
      std::find_if(adds_.begin(), adds_.end(),
                   [&](const EdgeRecord &e) { return e.key == key; });
  if (staged != adds_.end()) {
    adds_.erase(staged);
    return Result::unstaged;
  }
  if (!in_base(key) || removed(key))
    return Result::not_found;
  removes_.push_back(key);
  return Result::removed;
}

void PendingEdgeEdits::discard() {
  adds_.clear();
  removes_.clear();
}

int PendingEdgeEdits::count() const {
  return static_cast<int>(adds_.size() + removes_.size());
}

std::vector<PendingEdgeEdits::Op> PendingEdgeEdits::ops() const {
  std::vector<Op> out;
  for (const EdgeKey &k : removes_) {
    EdgeRecord r;
    r.key = k;
    for (const auto &b : base_)
      if (b.key == k) {
        r = b;
        break;
      }
    out.push_back({Op::Kind::remove, r});
  }
  for (const auto &a : adds_)
    out.push_back({Op::Kind::add, a});
  return out;
}

std::vector<EdgeView> PendingEdgeEdits::between(int a, int b) const {
  std::vector<EdgeView> out;
  auto match = [&](const EdgeKey &k) {
    return (k.from == a && k.to == b) || (k.from == b && k.to == a);
  };
  std::vector<EdgeKey> seen;
  for (const auto &e : base_) {
    if (!match(e.key) ||
        std::find(seen.begin(), seen.end(), e.key) != seen.end())
      continue; // duplicated rows of one key show once
    seen.push_back(e.key);
    out.push_back({e, false, removed(e.key), e.key.from != a});
  }
  for (const auto &e : adds_)
    if (match(e.key))
      out.push_back({e, true, false, e.key.from != a});
  return out;
}

std::map<int, int> PendingEdgeEdits::degrees() const {
  using Key = std::tuple<int, int, std::string>;
  std::set<Key> removed_keys;
  for (const auto &k : removes_)
    removed_keys.emplace(k.from, k.to, k.type);
  std::set<Key> seen;
  std::map<int, int> out;
  for (const auto &e : base_) {
    Key k{e.key.from, e.key.to, e.key.type};
    if (removed_keys.count(k) || !seen.insert(k).second)
      continue;
    ++out[e.key.from];
    if (e.key.to != e.key.from)
      ++out[e.key.to];
  }
  for (const auto &e : adds_) {
    ++out[e.key.from];
    if (e.key.to != e.key.from)
      ++out[e.key.to];
  }
  return out;
}

int PendingEdgeEdits::degree(int frame) const {
  int n = 0;
  std::vector<EdgeKey> seen;
  for (const auto &e : base_)
    if ((e.key.from == frame || e.key.to == frame) && !removed(e.key) &&
        std::find(seen.begin(), seen.end(), e.key) == seen.end()) {
      seen.push_back(e.key);
      ++n;
    }
  for (const auto &e : adds_)
    if (e.key.from == frame || e.key.to == frame)
      ++n;
  return n;
}

void PendingEdgeEdits::commit_succeeded() {
  base_.erase(
      std::remove_if(base_.begin(), base_.end(),
                     [&](const EdgeRecord &e) { return removed(e.key); }),
      base_.end());
  base_.insert(base_.end(), adds_.begin(), adds_.end());
  discard();
}

void PendingEdgeEdits::commit_saved(const std::vector<Op> &saved) {
  // What is pending now, as ops; the saved snapshot is applied to the base.
  const std::vector<Op> now = ops();
  for (const Op &op : saved) {
    if (op.kind == Op::Kind::remove)
      base_.erase(std::remove_if(base_.begin(), base_.end(),
                                 [&](const EdgeRecord &e) {
                                   return e.key == op.edge.key;
                                 }),
                  base_.end());
    else
      base_.push_back(op.edge);
  }
  auto in = [](const std::vector<Op> &v, const Op &op) {
    return std::any_of(v.begin(), v.end(), [&](const Op &o) {
      return o.kind == op.kind && o.edge.key == op.edge.key;
    });
  };
  adds_.clear();
  removes_.clear();
  // Pending ops that were not part of the save stay pending.
  for (const Op &op : now)
    if (!in(saved, op))
      (void)(op.kind == Op::Kind::add ? add(op.edge) : remove(op.edge.key));
  // A saved op the user undid during the save (staged add removed, staged
  // removal restored) must now be undone against the new base.
  for (const Op &op : saved)
    if (!in(now, op))
      (void)(op.kind == Op::Kind::add ? remove(op.edge.key) : add(op.edge));
}

// --------------------------------------------------------------- paging --

std::int64_t Paging::next_count() const {
  return std::max<std::int64_t>(0, std::min(page, total - loaded));
}

std::int64_t Paging::count_to_reach(std::int64_t row) const {
  if (row < loaded || row < 0 || page <= 0)
    return 0;
  const std::int64_t target = std::min(total, row + 1);
  const std::int64_t need = target - loaded;
  const std::int64_t pages = (need + page - 1) / page;
  return std::min(pages * page, total - loaded);
}

// ----------------------------------------------------------- formatting --

std::string format_decimal_da(double v, int decimals) {
  char buf[64];
  std::snprintf(buf, sizeof buf, "%.*f", std::max(0, decimals), v);
  std::string s(buf);
  if (s.size() > 1 && s[0] == '-' &&
      s.find_first_not_of("-0.") == std::string::npos)
    s.erase(0, 1); // no "-0,00"
  for (char &c : s)
    if (c == '.')
      c = ',';
  return s;
}

std::string format_bytes_da(std::uint64_t bytes) {
  if (bytes < 1000)
    return std::to_string(bytes) + " B";
  static const char *units[] = {"kB", "MB", "GB", "TB"};
  double v = static_cast<double>(bytes);
  int u = -1;
  while (v >= 999.95 && u < 3) {
    v /= 1000.0;
    ++u;
  }
  // GB and up keep two decimals: a project's size is read in hundreds of MB.
  return format_decimal_da(v, u >= 2 ? 2 : 1) + " " + units[u];
}

std::string sniff_blob(std::string_view h) {
  auto starts = [&](std::string_view p) {
    return h.size() >= p.size() && h.substr(0, p.size()) == p;
  };
  if (starts(std::string_view("\x89PNG\r\n\x1a\n", 8)))
    return "PNG";
  if (starts("\xff\xd8\xff"))
    return "JPEG";
  if (starts("ply\n") || starts("ply\r\n"))
    return "PLY";
  if (starts("%PDF"))
    return "PDF";
  if (starts("RIFF") && h.size() >= 12 && h.substr(8, 4) == "WEBP")
    return "WebP";
  if (starts("\x1f\x8b"))
    return "GZIP";
  if (starts("PK\x03\x04"))
    return "ZIP";
  // JSON only when the whole head is text: binary data (float chunks)
  // starts with '{' or '[' bytes often enough.
  const bool text = std::all_of(h.begin(), h.end(), [](char ch) {
    const auto u = static_cast<unsigned char>(ch);
    return u >= 0x20 || u == '\t' || u == '\r' || u == '\n';
  });
  const auto first = h.find_first_not_of(" \t\r\n");
  if (text && first != std::string_view::npos &&
      (h[first] == '{' || h[first] == '['))
    return "JSON";
  return {};
}

std::string blob_column_meaning(std::string_view table,
                                std::string_view column) {
  struct Known {
    std::string_view table, column, meaning;
  };
  static const Known known[] = {
      {"sensor_frames", "color", "Farvebillede"},
      {"sensor_frames", "depth", "Dybde"},
      {"sensor_frames", "confidence", "Konfidens"},
      {"sensor_frames", "transform", "Pose 4×4"},
      {"segmentation_images", "label_image", "Mærkatbillede"},
      {"glass_confidence_images", "confidence", "Glas-konfidens"},
      {"panoramic_images", "image_data", "Panorama"},
      {"panoramic_images", "pose", "Pose 4×4"},
      {"panorama_segmentation", "label_image", "Mærkatbillede"},
      {"point_cloud_data", "data", "Punktdata"},
      {"point_clouds", "tile_index", "Fliseindeks"},
      {"meshes", "data", "Mesh"},
      {"mesh_texture_data", "data", "Tekstur"},
      {"gaussian_splat_data", "data", "Splat-data"},
      {"material_thumbnails", "data", "Miniature"},
      {"report_pdfs", "pdf_blob", "Rapport"},
      {"building_components", "geometry", "Geometri"},
  };
  for (const auto &k : known)
    if (k.table == table && k.column == column)
      return std::string(k.meaning);
  return {};
}

std::string describe_blob(std::string_view table, std::string_view column,
                          std::string_view head, std::uint64_t size) {
  std::string out = blob_column_meaning(table, column);
  const std::string format = sniff_blob(head);
  auto add = [&](const std::string &part) {
    if (part.empty())
      return;
    if (!out.empty())
      out += " · ";
    out += part;
  };
  if (out.empty() && column == "transform" && size == 128)
    out = "Pose 4×4";
  add(format);
  add(format_bytes_da(size));
  return out;
}

PoseSummary summarize_pose(const std::array<double, 16> &m) {
  PoseSummary s;
  s.t = {m[3], m[7], m[11]};
  // R = Rz(yaw) * Ry(pitch) * Rx(roll), the ZYX convention.
  const double r00 = m[0], r10 = m[4], r20 = m[8], r21 = m[9], r22 = m[10];
  constexpr double kDeg = 180.0 / 3.14159265358979323846;
  s.pitch_deg = std::asin(std::clamp(-r20, -1.0, 1.0)) * kDeg;
  s.yaw_deg = std::atan2(r10, r00) * kDeg;
  s.roll_deg = std::atan2(r21, r22) * kDeg;
  const double trace = m[0] + m[5] + m[10];
  s.angle_deg = std::acos(std::clamp((trace - 1.0) / 2.0, -1.0, 1.0)) * kDeg;
  return s;
}

PoseDelta pose_delta(const std::array<double, 16> &a,
                     const std::array<double, 16> &b) {
  PoseDelta d;
  const double dx = b[3] - a[3], dy = b[7] - a[7], dz = b[11] - a[11];
  d.distance_m = std::sqrt(dx * dx + dy * dy + dz * dz);
  // trace(Ra^T Rb) = sum_ij Ra_ij Rb_ij over the 3x3 blocks.
  double trace = 0;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      trace += a[i * 4 + j] * b[i * 4 + j];
  constexpr double kDeg = 180.0 / 3.14159265358979323846;
  d.angle_deg = std::acos(std::clamp((trace - 1.0) / 2.0, -1.0, 1.0)) * kDeg;
  return d;
}

// --------------------------------------------------------- write errors --

WriteErrorKind classify_write_error(std::string_view what) {
  auto has = [&](std::string_view s) {
    return what.find(s) != std::string_view::npos;
  };
  if (has("locked") || has("busy"))
    return WriteErrorKind::locked;
  if (has("readonly") || has("read-only") || has("read only"))
    return WriteErrorKind::read_only;
  return WriteErrorKind::other;
}

std::string write_error_da(WriteErrorKind kind) {
  switch (kind) {
  case WriteErrorKind::read_only:
    return "Projektet er åbnet skrivebeskyttet, så ændringerne kan ikke "
           "gemmes. Åbn det med skriveadgang for at gemme.";
  case WriteErrorKind::locked:
    return "Projektet er låst af en anden proces, der skriver til det. "
           "Ændringerne er ikke gemt — prøv igen, når den er færdig.";
  case WriteErrorKind::other:
    break;
  }
  return "Ændringerne kunne ikke gemmes. De venter stadig; se detaljerne.";
}

} // namespace rux::qt
