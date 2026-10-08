// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/workspace_logic.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <utility>

namespace rux::qt {

namespace {

std::string lower(std::string_view s) {
  std::string out(s);
  for (char &c : out)
    if (c >= 'A' && c <= 'Z')
      c = static_cast<char>(c - 'A' + 'a');
  return out;
}

bool contains_ci(std::string_view hay, const std::string &needle_lower) {
  return lower(hay).find(needle_lower) != std::string::npos;
}

} // namespace

bool log_row_matches(const LogRow &row, const LogFilter &filter) {
  if (!filter.stage.empty() && row.stage != filter.stage)
    return false;
  switch (filter.status) {
  case LogStatusFilter::all:
    break;
  case LogStatusFilter::success:
    if (row.status != "success")
      return false;
    break;
  case LogStatusFilter::failed:
    if (row.status != "failed")
      return false;
    break;
  case LogStatusFilter::cancelled:
    if (row.status != "cancelled")
      return false;
    break;
  case LogStatusFilter::unfinished:
    if (!(row.status == "running" && !row.finished))
      return false;
    break;
  }
  if (!filter.text.empty()) {
    const std::string needle = lower(filter.text);
    if (!contains_ci(row.stage, needle) && !contains_ci(row.error, needle) &&
        !contains_ci(row.parameters, needle))
      return false;
  }
  return true;
}

std::vector<std::string> log_stages(const std::vector<LogRow> &rows) {
  std::set<std::string> s;
  for (const auto &r : rows)
    s.insert(r.stage);
  return {s.begin(), s.end()};
}

std::string log_status_label(const std::string &status, bool finished) {
  if (status == "success")
    return "Gennemført";
  if (status == "failed")
    return "Fejlet";
  if (status == "cancelled")
    return "Annulleret";
  if (status == "running")
    // A "running" row with no finished_at is a process that died mid-run; one
    // WITH finished_at set is not supposed to happen (log_pipeline_end always
    // writes status and finished_at together) but is handled defensively.
    return finished ? "Kørte" : "Afbrudt";
  return status;
}

std::string log_status_tone_key(const std::string &status, bool finished) {
  if (status == "success")
    return "good";
  if (status == "failed")
    return "crit";
  if (status == "cancelled")
    return "wait";
  if (status == "running" && !finished)
    return "wait";
  return "outline";
}

double segment_distance(double px, double py, double ax, double ay, double bx,
                        double by) {
  const double dx = bx - ax, dy = by - ay;
  const double len2 = dx * dx + dy * dy;
  double t = len2 > 0.0 ? ((px - ax) * dx + (py - ay) * dy) / len2 : 0.0;
  t = std::clamp(t, 0.0, 1.0);
  const double cx = ax + t * dx - px, cy = ay + t * dy - py;
  return std::sqrt(cx * cx + cy * cy);
}

int nearest_node(const std::vector<GraphNode> &nodes, double x, double y,
                 double radius) {
  int best = -1;
  double best_d = radius;
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    const double d = std::hypot(nodes[i].x - x, nodes[i].y - y);
    if (d <= best_d && (best < 0 || d < best_d)) {
      best = static_cast<int>(i);
      best_d = d;
    }
  }
  return best;
}

int nearest_edge(const std::vector<GraphNode> &nodes,
                 const std::vector<GraphEdge> &edges, double x, double y,
                 double radius) {
  std::map<int, std::size_t> by_id;
  for (std::size_t i = 0; i < nodes.size(); ++i)
    by_id.emplace(nodes[i].id, i);
  int best = -1;
  double best_d = radius;
  for (std::size_t i = 0; i < edges.size(); ++i) {
    const auto a = by_id.find(edges[i].from);
    const auto b = by_id.find(edges[i].to);
    if (a == by_id.end() || b == by_id.end())
      continue;
    const GraphNode &na = nodes[a->second];
    const GraphNode &nb = nodes[b->second];
    const double d = segment_distance(x, y, na.x, na.y, nb.x, nb.y);
    if (d <= best_d && (best < 0 || d < best_d)) {
      best = static_cast<int>(i);
      best_d = d;
    }
  }
  return best;
}

// ------------------------------------------------------------- Log tap --

namespace {

struct Tap {
  std::mutex mutex;
  std::size_t next = 1;
  // Copy-on-write, so publish_log calls listeners with no lock held.
  std::shared_ptr<const std::vector<std::pair<std::size_t, LogListener>>>
      listeners = std::make_shared<
          const std::vector<std::pair<std::size_t, LogListener>>>();
};

Tap &tap() {
  static Tap *t = new Tap; // never destroyed: logging may outlive statics
  return *t;
}

} // namespace

std::size_t add_log_listener(LogListener listener) {
  Tap &t = tap();
  std::lock_guard lock(t.mutex);
  auto next =
      std::make_shared<std::vector<std::pair<std::size_t, LogListener>>>(
          *t.listeners);
  const std::size_t token = t.next++;
  next->emplace_back(token, std::move(listener));
  t.listeners = std::move(next);
  return token;
}

void remove_log_listener(std::size_t token) {
  Tap &t = tap();
  std::lock_guard lock(t.mutex);
  auto next =
      std::make_shared<std::vector<std::pair<std::size_t, LogListener>>>(
          *t.listeners);
  next->erase(
      std::remove_if(next->begin(), next->end(),
                     [token](const auto &p) { return p.first == token; }),
      next->end());
  t.listeners = std::move(next);
}

void publish_log(int level, std::string_view message) {
  Tap &t = tap();
  std::shared_ptr<const std::vector<std::pair<std::size_t, LogListener>>>
      snapshot;
  {
    std::lock_guard lock(t.mutex);
    snapshot = t.listeners;
  }
  for (const auto &[token, fn] : *snapshot)
    fn(level, message);
}

bool log_tail_accepts(int level, bool on_job_thread) {
  if (level < 2) // trace, debug: never (info and up only)
    return false;
  if (level < 3) // info: only the job's own thread
    return on_job_thread;
  return true; // warn and above: any thread
}

} // namespace rux::qt
