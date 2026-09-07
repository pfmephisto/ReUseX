// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// License-clean bridge for learned loop-closure matchers (issue #221 / #225
// P2).
//
// The wide-baseline loop-closure front-end (LoopClosure.cpp) ships a
// commercial-safe ORB matcher, but the strongest wide-baseline correspondences
// come from learned matchers (EfficientLoFTR / LightGlue+ALIKED / MapAnything)
// or pointmap foundation models (MASt3R). Two obstacles keep those out of the
// C++ binary directly: (a) some are painful to link into GPL C++ (Python-only
// research code), and (b) several carry non-commercial licences (MASt3R,
// MapAnything's 13-dataset checkpoint) that must not enter a commercial
// product.
//
// This loader dissolves both problems by moving the producer OUT of process:
// an external tool (Python) writes a JSON file of relative-pose constraints,
// and this GPL C++ consumes it purely as DATA. A non-commercial model can then
// serve as an offline accuracy-CEILING oracle without ever linking into the
// shipped binary, while a commercial-safe matcher writes the identical file for
// the production path.
//
// Trust boundary: the file is UNTRUSTED INPUT from another process, so this
// loader validates the numeric payload (SE(3)-ness, finiteness, positive
// sigmas) and not just the JSON shape — see load_loop_edges' doc block. The
// accepted edges land in the very same GncOptimizer graph as the
// internally-detected ORB edges, and optimize_sensor_poses() applies the
// seed-disagreement gate and (when `--loop-no-pcm` is not given) PCM over the
// UNION of both sources, so GNC / PCM / seed gating cover the file path too.
// The consistency filtering happens THERE, not here: PCM needs the seed poses
// and the internal edges, neither of which this function sees.

#include "slam/LoopClosure.hpp"

#include "core/logging.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace reusex::geometry {

namespace {
constexpr const char *kSchema = "reusex.loop_edges.v1";

/// Orthonormality tolerance on the rotation block. Loose enough (1e-3) to
/// accept a rotation that has been round-tripped through float or printed with
/// ~7 significant digits by a Python producer, tight enough to catch a matrix
/// that is not a rotation at all (a scale, a shear, a pointmap-derived
/// similarity transform).
constexpr double kRotTolerance = 1e-3;
/// Bottom row must be exactly [0,0,0,1] up to print precision; anything else
/// means the producer wrote a general projective matrix, not an SE(3) element.
constexpr double kBottomRowTolerance = 1e-6;

/// Is @p T a usable SE(3) element? Rejects non-finite entries, a
/// non-orthonormal 3x3 block, a REFLECTION (det < 0 — mirror-image geometry,
/// which drags the solve to a wrong basin rather than merely adding noise), and
/// a bottom row that is not [0,0,0,1].
bool is_valid_se3(const Eigen::Matrix4d &T) {
  if (!T.allFinite())
    return false;
  const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  if ((R.transpose() * R - Eigen::Matrix3d::Identity()).norm() > kRotTolerance)
    return false;
  if (R.determinant() < 0.0)
    return false;
  const Eigen::Vector4d bottom = T.row(3).transpose();
  return (bottom - Eigen::Vector4d(0.0, 0.0, 0.0, 1.0)).norm() <=
         kBottomRowTolerance;
}

/// A noise-model std must be finite and strictly positive. Zero is the
/// dangerous case, not just an odd one: gtsam turns a zero sigma into a
/// Constrained (hard-equality) model, so a single `"sigma_trans": 0` line would
/// weld two poses together and override every other factor in the graph.
bool is_valid_sigma(double s) { return std::isfinite(s) && s > 0.0; }
} // namespace

std::vector<LoopEdge> load_loop_edges(const std::string &path,
                                      const std::vector<int> &node_ids,
                                      LoopClosureResult *out_result) {
  std::ifstream in(path);
  if (!in)
    throw std::runtime_error("load_loop_edges: cannot open edge file '" + path +
                             "'");

  nlohmann::json j;
  try {
    in >> j;
  } catch (const std::exception &e) {
    throw std::runtime_error("load_loop_edges: '" + path +
                             "' is not valid JSON: " + e.what());
  }

  // Schema check — a wrong file must fail loudly, not run an unconstrained
  // optimization (docs/STANDARDS.md §5).
  if (!j.contains("edges") || !j["edges"].is_array())
    throw std::runtime_error("load_loop_edges: '" + path +
                             "' has no 'edges' array (expected schema '" +
                             kSchema + "')");
  if (j.contains("schema") && j["schema"].is_string() &&
      j["schema"].get<std::string>() != kSchema)
    core::warn("load_loop_edges: '{}' declares schema '{}', expected '{}' — "
               "parsing anyway",
               path, j["schema"].get<std::string>(), kSchema);

  const std::string producer =
      (j.contains("producer") && j["producer"].is_string())
          ? j["producer"].get<std::string>()
          : std::string("<unknown>");

  // node id -> frame index. If a producer emitted the same node id twice
  // (should not happen) the first wins; that mirrors the optimizer's frame
  // ordering, which is unique per node id by construction.
  std::unordered_map<int, int> id_to_index;
  id_to_index.reserve(node_ids.size() * 2);
  for (int k = 0; k < static_cast<int>(node_ids.size()); ++k)
    id_to_index.emplace(node_ids[k], k);

  std::vector<LoopEdge> edges;
  edges.reserve(j["edges"].size());
  std::set<std::pair<int, int>> seen; // dedup on frame-index pair (min,max)

  int skipped_unknown = 0, skipped_self = 0, skipped_dup = 0, skipped_bad = 0;
  int total_inliers = 0;

  for (const auto &je : j["edges"]) {
    if (!je.contains("node_i") || !je.contains("node_j") ||
        !je.contains("T_ij") || !je["T_ij"].is_array() ||
        je["T_ij"].size() != 16) {
      ++skipped_bad;
      continue;
    }

    // A wrong-typed field (e.g. "node_i": "7", "sigma_trans": null) makes
    // nlohmann throw json::type_error. That is a malformed EDGE, not a
    // malformed FILE: count it and keep reading, so one bad producer line
    // cannot abort an otherwise usable edge set. Structural problems (no
    // 'edges' array, invalid JSON) still throw above.
    try {
      const int ni = je["node_i"].get<int>();
      const int nj = je["node_j"].get<int>();

      auto it_i = id_to_index.find(ni);
      auto it_j = id_to_index.find(nj);
      if (it_i == id_to_index.end() || it_j == id_to_index.end()) {
        ++skipped_unknown;
        continue;
      }

      LoopEdge e;
      e.i = it_i->second;
      e.j = it_j->second;
      if (e.i == e.j) {
        ++skipped_self;
        continue;
      }

      // Row-major 4x4 into an Eigen::Matrix4d (Eigen default is column-major,
      // so fill element-wise rather than memcpy).
      for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
          e.T_ij(r, c) = je["T_ij"][r * 4 + c].get<double>();

      if (je.contains("sigma_rot"))
        e.sigma_rot = je["sigma_rot"].get<double>();
      if (je.contains("sigma_trans"))
        e.sigma_trans = je["sigma_trans"].get<double>();
      if (je.contains("inliers"))
        e.inliers = je["inliers"].get<int>();

      // Numeric payload validation. Shape-checking alone is not enough: the
      // 16 doubles must actually be an SE(3) element and the sigmas must be
      // usable as a Gaussian noise model, or the constraint silently corrupts
      // the solve instead of failing (docs/STANDARDS.md §5).
      if (!is_valid_se3(e.T_ij) || !is_valid_sigma(e.sigma_rot) ||
          !is_valid_sigma(e.sigma_trans)) {
        ++skipped_bad;
        continue;
      }

      // Dedup only AFTER the edge is known good, so a malformed first
      // occurrence does not consume the (i,j) slot of a valid later one.
      const auto key = std::minmax(e.i, e.j);
      if (!seen.emplace(key.first, key.second).second) {
        ++skipped_dup;
        continue;
      }

      total_inliers += e.inliers;
      edges.push_back(std::move(e));
    } catch (const nlohmann::json::type_error &) {
      ++skipped_bad;
      continue;
    }
  }

  const int skipped =
      skipped_unknown + skipped_self + skipped_dup + skipped_bad;
  if (skipped > 0)
    core::warn("load_loop_edges: skipped {} of {} edges from '{}' "
               "({} unknown-node, {} self-loop, {} duplicate, {} malformed)",
               skipped, j["edges"].size(), path, skipped_unknown, skipped_self,
               skipped_dup, skipped_bad);
  core::info("load_loop_edges: {} external loop edges (producer '{}', {} total "
             "inliers) from '{}'",
             edges.size(), producer, total_inliers, path);

  if (out_result) {
    out_result->candidates = static_cast<int>(j["edges"].size());
    out_result->edges = static_cast<int>(edges.size());
    out_result->total_inliers = total_inliers;
  }
  return edges;
}

} // namespace reusex::geometry
