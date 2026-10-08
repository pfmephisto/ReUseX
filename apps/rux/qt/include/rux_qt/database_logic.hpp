// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The Qt-free half of the Database workspace (Stream Q, Q2): the A/B frame
// pair and its keyboard, the pending pose-graph edits that "Gem ændringer"
// writes in one transaction, the table viewer's paging, and the small
// formatters (sizes, blob kinds, poses, edge types, write errors) the widgets
// show. Standard library only, unit-tested in the light binary.

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace rux::qt {

// ----------------------------------------------------------- frame pair --

/// The two frames being compared, as indices into the sorted frame ids. A is
/// the frame being browsed; B is the one it is compared with. Both are always
/// valid when there are frames; with one frame, A and B are the same.
class FramePair {
    public:
  /// Sorted, de-duplicated frame ids. A goes to the first frame and B to the
  /// second (or the first, with only one).
  void reset(std::vector<int> ids);

  bool empty() const { return ids_.empty(); }
  int size() const { return static_cast<int>(ids_.size()); }
  const std::vector<int> &ids() const { return ids_; }

  int a_index() const { return a_; }
  int b_index() const { return b_; }
  /// -1 when empty.
  int a_id() const;
  int b_id() const;

  /// Move by @p delta frames, clamped to the ends. True if it moved.
  bool step_a(int delta);
  bool step_b(int delta);
  bool set_a_index(int index);
  bool set_b_index(int index);
  /// Select by frame id; false (and no change) when the id is not a frame.
  bool set_a_id(int id);
  bool set_b_id(int id);

  /// Index of frame @p id, or -1.
  int index_of(int id) const;
  /// The index whose id is nearest @p id (ties go to the lower id); -1 when
  /// empty. For jumping to "the first frame of scan N" or a typed id.
  int nearest_index(int id) const;

    private:
  static bool set(int &slot, int index, int size);
  std::vector<int> ids_;
  int a_ = -1;
  int b_ = -1;
};

/// What a key press in the frame browser does.
enum class BrowserKey { none, a_prev, a_next, b_prev, b_next };
enum class ArrowKey { left, right, other };

/// ←/→ move A, Shift+←/→ move B. Nothing while a text field or spin box has
/// the focus — its own cursor keys win.
BrowserKey browser_key(ArrowKey key, bool shift, bool text_focus);

/// How far a browser key moves: 1, or 10 with Ctrl (Page-like jumps).
int browser_step(bool ctrl);

// ------------------------------------------------------- pending edits --

/// One pose-graph edge as the editor sees it.
struct EdgeKey {
  int from = 0;
  int to = 0;
  std::string type; ///< "odometry" | "loop_closure" | "panorama"
  bool operator==(const EdgeKey &o) const {
    return from == o.from && to == o.to && type == o.type;
  }
};

struct EdgeRecord {
  EdgeKey key;
  double residual = 0.0;
  double weight = 1.0; ///< NaN when the optimizer stored none
};

/// An edge between the current A and B, with its pending state.
struct EdgeView {
  EdgeRecord edge;
  bool pending_add = false;    ///< staged, not in the file yet
  bool pending_delete = false; ///< in the file, staged for deletion
  bool reversed = false;       ///< stored B -> A rather than A -> B
};

/// Pose-graph edits staged in memory until "Gem ændringer" (RTABMap's
/// linksAdded_/linksRemoved_). The base is what the file holds; ops() is
/// what a save must do, deletions first, in staging order.
class PendingEdgeEdits {
    public:
  enum class Result {
    added,     ///< a new edge is staged
    restored,  ///< it re-adds an edge staged for deletion: no-op now
    removed,   ///< an edge in the file is staged for deletion
    unstaged,  ///< it removes a staged addition: no-op now
    duplicate, ///< that edge already exists (or is already staged)
    not_found, ///< nothing to remove
    invalid,   ///< from == to, an unknown type, or a weight <= 0
  };

  struct Op {
    enum class Kind { remove, add };
    Kind kind;
    EdgeRecord edge;
  };

  /// The edges as stored. Clears every pending edit.
  void set_base(std::vector<EdgeRecord> edges);
  const std::vector<EdgeRecord> &base() const { return base_; }

  Result add(const EdgeRecord &edge);
  /// Stage the removal of every stored row with this key (the store deletes
  /// by from/to/type), or drop a staged addition.
  Result remove(const EdgeKey &key);
  void discard();

  bool empty() const { return adds_.empty() && removes_.empty(); }
  /// Number of staged edits (additions + removals).
  int count() const;
  std::vector<Op> ops() const;

  /// Every edge between frames @p a and @p b, either direction: stored ones
  /// (flagged when staged for deletion) and staged additions.
  std::vector<EdgeView> between(int a, int b) const;
  /// Effective edge count of a frame (stored minus deleted plus added).
  int degree(int frame) const;

  /// After a successful save: fold the ops into the base, clear pending.
  void commit_succeeded();

    private:
  bool removed(const EdgeKey &k) const;
  bool in_base(const EdgeKey &k) const;
  std::vector<EdgeRecord> base_;
  std::vector<EdgeRecord> adds_;
  std::vector<EdgeKey> removes_;
};

/// True for the three stored edge types.
bool is_edge_type(std::string_view type);
/// Danish name of an edge type ("Odometri", "Løkkelukning", "Panorama").
std::string edge_type_da(std::string_view type);

/// A weight for a manual edge from an ICP fit: the information weight the
/// store uses (1 / sigma^2) with sigma the RMS fitness, floored at 1 cm so a
/// perfect synthetic fit cannot claim infinite certainty.
double weight_from_icp_fitness(double rms_m);

// --------------------------------------------------------------- paging --

/// Lazy paging of a table model (QAbstractTableModel::fetchMore).
struct Paging {
  std::int64_t total = 0;  ///< rows in the table
  std::int64_t loaded = 0; ///< rows fetched so far
  std::int64_t page = 200; ///< rows per fetch

  bool can_fetch_more() const { return loaded < total; }
  /// Rows the next fetch asks for (0 when done).
  std::int64_t next_count() const;
  /// Rows to fetch so that row @p row is loaded (0 if it already is).
  std::int64_t count_to_reach(std::int64_t row) const;
};

// ----------------------------------------------------------- formatting --

/// A byte size the Danish way: "834 B", "33,4 kB", "1,2 MB", "3,40 GB".
std::string format_bytes_da(std::uint64_t bytes);

/// What a blob is, from its first bytes: "PNG", "JPEG", "PLY", "PDF", "WebP",
/// "GZIP", "ZIP", "JSON"… or empty when unknown.
std::string sniff_blob(std::string_view head);

/// What a known blob column holds, in Danish ("Farvebillede", "Pose 4×4"),
/// or empty for an unknown column.
std::string blob_column_meaning(std::string_view table,
                                std::string_view column);

/// The text a table cell shows for a blob: meaning, format and size, e.g.
/// "Farvebillede · JPEG · 172,7 kB". A 128-byte `transform` is "Pose 4×4".
std::string describe_blob(std::string_view table, std::string_view column,
                          std::string_view head, std::uint64_t size);

/// Translation and rotation of a row-major 4x4 pose.
struct PoseSummary {
  std::array<double, 3> t{}; ///< translation (m)
  double yaw_deg = 0;        ///< about z
  double pitch_deg = 0;      ///< about y
  double roll_deg = 0;       ///< about x
  double angle_deg = 0;      ///< total rotation angle
};
PoseSummary summarize_pose(const std::array<double, 16> &m);

/// How far apart two poses are: translation distance (m) and the rotation
/// angle between them (degrees).
struct PoseDelta {
  double distance_m = 0;
  double angle_deg = 0;
};
PoseDelta pose_delta(const std::array<double, 16> &a,
                     const std::array<double, 16> &b);

/// A decimal number with a Danish comma and @p decimals digits.
std::string format_decimal_da(double v, int decimals);

// --------------------------------------------------------- write errors --

enum class WriteErrorKind { read_only, locked, other };

/// Map a ProjectDB / sqlite exception from a save to a kind.
WriteErrorKind classify_write_error(std::string_view what);
/// The Danish message shown in the pending-edits banner for @p kind.
std::string write_error_da(WriteErrorKind kind);

} // namespace rux::qt
