// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Qt-free logic of the Q3 workspaces (3D, Posegraf, Pipeline, Log), so the
// light test binary covers it:
//
//  - the Log workspace's filter over pipeline_log rows;
//  - the Posegraf's hit testing in scene units;
//  - a process-wide log tap the Pipeline workspace tails a running job with.

#include <cstddef>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

namespace rux::qt {

// ---------------------------------------------------------------- Log --

/// The fields of one pipeline_log row a filter looks at.
struct LogRow {
  std::string stage;
  std::string status; ///< "running" | "success" | "failed"
  bool finished = false;
  std::string error;
  std::string parameters;
};

enum class LogStatusFilter { all, success, failed, unfinished };

struct LogFilter {
  std::string stage; ///< exact stage name; empty = every stage
  LogStatusFilter status = LogStatusFilter::all;
  /// Case-insensitive (ASCII) substring of stage, error or parameters.
  std::string text;
};

bool log_row_matches(const LogRow &row, const LogFilter &filter);

/// The distinct stage names of @p rows, sorted.
std::vector<std::string> log_stages(const std::vector<LogRow> &rows);

// ------------------------------------------------------------ Posegraf --

struct GraphNode {
  int id = 0;
  double x = 0.0;
  double y = 0.0;
};

struct GraphEdge {
  int from = 0;
  int to = 0;
};

/// Index of the node nearest (@p x, @p y) within @p radius, or -1.
/// Ties go to the lower index (the earlier frame).
int nearest_node(const std::vector<GraphNode> &nodes, double x, double y,
                 double radius);

/// Index of the edge whose segment passes nearest (@p x, @p y) within
/// @p radius, or -1. @p nodes is looked up by id; an edge with an unknown
/// endpoint is skipped.
int nearest_edge(const std::vector<GraphNode> &nodes,
                 const std::vector<GraphEdge> &edges, double x, double y,
                 double radius);

/// Distance from (@p px, @p py) to the segment a-b.
double segment_distance(double px, double py, double ax, double ay, double bx,
                        double by);

// ------------------------------------------------------------- Log tap --

/// Receives every library log line (level as reusex::core::LogLevel's int).
/// Called on the logging thread with no lock held: post, do not block.
using LogListener = std::function<void(int level, std::string_view message)>;

/// Register @p listener; returns a token for remove_log_listener().
std::size_t add_log_listener(LogListener listener);
void remove_log_listener(std::size_t token);
/// Hand @p message to every listener (rux's log handler and the gallery's
/// call this).
void publish_log(int level, std::string_view message);

} // namespace rux::qt
