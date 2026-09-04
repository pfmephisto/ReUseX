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
// the production path. The edges land in the very same GncOptimizer graph as
// the internally-detected ORB edges, so GNC / PCM / seed-disagreement gating
// all apply unchanged.

#include "slam/LoopClosure.hpp"

#include "core/logging.hpp"

#include <nlohmann/json.hpp>

#include <fstream>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace reusex::geometry {

namespace {
constexpr const char *kSchema = "reusex.loop_edges.v1";
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

    const auto key = std::minmax(e.i, e.j);
    if (!seen.emplace(key.first, key.second).second) {
      ++skipped_dup;
      continue;
    }

    // Row-major 4x4 into an Eigen::Matrix4d (Eigen default is column-major, so
    // fill element-wise rather than memcpy).
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        e.T_ij(r, c) = je["T_ij"][r * 4 + c].get<double>();

    if (je.contains("sigma_rot"))
      e.sigma_rot = je["sigma_rot"].get<double>();
    if (je.contains("sigma_trans"))
      e.sigma_trans = je["sigma_trans"].get<double>();
    if (je.contains("inliers"))
      e.inliers = je["inliers"].get<int>();

    total_inliers += e.inliers;
    edges.push_back(std::move(e));
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
