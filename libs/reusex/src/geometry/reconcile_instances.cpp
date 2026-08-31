// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/geometry/reconcile_instances.hpp"
#include "reusex/core/logging.hpp"

#include <algorithm>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace reusex::geometry {

ReconcileResult
reconcile_instance_identities(const CloudL *old_labels,
                              const CloudL &new_labels,
                              const std::vector<PriorInstance> &prior,
                              const std::map<uint32_t, uint32_t> &new_semantic,
                              const std::map<uint32_t, size_t> &new_sizes,
                              const std::function<std::string()> &make_guid,
                              const ReconcileOptions &opts) {

  ReconcileResult result;

  // Prior lookup by old instance_id.
  std::unordered_map<uint32_t, const PriorInstance *> prior_by_id;
  for (const auto &p : prior)
    prior_by_id.emplace(p.instance_id, &p);

  const bool have_old =
      old_labels != nullptr && !old_labels->empty() && !prior.empty();

  if (have_old && old_labels->size() != new_labels.size())
    throw std::invalid_argument(fmt::format(
        "reconcile_instance_identities: old label cloud has {} points, new has "
        "{} — clouds must be index-aligned over the same point cloud",
        old_labels->size(), new_labels.size()));

  // Point counts of old instances (from the label cloud itself, so they line
  // up with what we can intersect).
  std::unordered_map<uint32_t, size_t> old_size;
  // Point counts of new instances.
  std::unordered_map<uint32_t, size_t> new_size;
  // Overlap[new_id][old_id] = |intersection| in points.
  std::unordered_map<uint32_t, std::unordered_map<uint32_t, size_t>> overlap;

  if (have_old) {
    const size_t n = new_labels.size();
    for (size_t i = 0; i < n; ++i) {
      uint32_t nl = new_labels[i].label;
      uint32_t ol = (*old_labels)[i].label;
      if (nl > 0)
        ++new_size[nl];
      if (ol > 0)
        ++old_size[ol];
      if (nl > 0 && ol > 0)
        ++overlap[nl][ol];
    }
  }

  // Semantic class of an old instance (from prior rows).
  auto old_semantic = [&](uint32_t old_id) -> int {
    auto it = prior_by_id.find(old_id);
    return it == prior_by_id.end() ? -1 : it->second->semantic_class;
  };

  // Candidate matches: (overlap_fraction, new_id, old_id). We match greedily by
  // descending overlap fraction, one-to-one, only within the same semantic
  // class and above the threshold.
  struct Cand {
    double frac;
    uint32_t new_id;
    uint32_t old_id;
  };
  std::vector<Cand> candidates;

  for (const auto &[new_id, old_map] : overlap) {
    int sem_new = -1;
    if (auto s = new_semantic.find(new_id); s != new_semantic.end())
      sem_new = static_cast<int>(s->second);
    for (const auto &[old_id, inter] : old_map) {
      if (old_semantic(old_id) != sem_new)
        continue; // Only match within the same semantic class.
      size_t smaller = std::min(new_size[new_id], old_size[old_id]);
      if (smaller == 0)
        continue;
      double frac = static_cast<double>(inter) / static_cast<double>(smaller);
      candidates.push_back({frac, new_id, old_id});
    }
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const Cand &a, const Cand &b) { return a.frac > b.frac; });

  std::unordered_map<uint32_t, uint32_t> new_to_old; // matched new -> old
  std::unordered_set<uint32_t> used_old;
  std::unordered_set<uint32_t> used_new;
  // Track best overlap seen per old id (for orphan diagnostics).
  std::unordered_map<uint32_t, double> best_old_overlap;

  for (const auto &c : candidates) {
    best_old_overlap[c.old_id] = std::max(best_old_overlap[c.old_id], c.frac);
    if (c.frac <= opts.min_overlap)
      continue;
    if (used_new.count(c.new_id) || used_old.count(c.old_id))
      continue;
    new_to_old[c.new_id] = c.old_id;
    used_new.insert(c.new_id);
    used_old.insert(c.old_id);
  }

  // Build the reconciled instance list, one entry per new instance.
  for (const auto &[new_id, sem] : new_semantic) {
    ReconciledInstance ri;
    ri.instance_id = new_id;
    ri.semantic_class = static_cast<int>(sem);
    if (auto sz = new_sizes.find(new_id); sz != new_sizes.end())
      ri.point_count = static_cast<int>(sz->second);

    auto m = new_to_old.find(new_id);
    if (m != new_to_old.end()) {
      const PriorInstance *p = prior_by_id.at(m->second);
      ri.guid = p->guid;
      ri.carried_over = true;
      ri.matched_old_id = m->second;
      ++result.matched_count;
    } else {
      ri.guid = make_guid();
      ri.carried_over = false;
      ++result.fresh_count;
    }
    result.instances.push_back(std::move(ri));
  }

  std::sort(result.instances.begin(), result.instances.end(),
            [](const ReconciledInstance &a, const ReconciledInstance &b) {
              return a.instance_id < b.instance_id;
            });

  // Old instances that were not carried over are orphaned.
  for (const auto &p : prior) {
    if (used_old.count(p.instance_id))
      continue;
    OrphanedInstance o;
    o.instance_id = p.instance_id;
    o.semantic_class = p.semantic_class;
    o.guid = p.guid;
    if (auto it = best_old_overlap.find(p.instance_id);
        it != best_old_overlap.end())
      o.best_overlap = it->second;
    result.orphaned_old.push_back(std::move(o));
  }

  reusex::info("Instance identity reconciliation: {} carried over, {} fresh, "
               "{} orphaned old GUID(s)",
               result.matched_count, result.fresh_count,
               result.orphaned_old.size());

  return result;
}

} // namespace reusex::geometry
