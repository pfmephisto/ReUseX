// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "reusex/types.hpp"

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace reusex::geometry {

/// A single reconciled instance: its (new) label value, the semantic class it
/// belongs to, its point count, and the stable GUID it should carry. When
/// @ref carried_over is true the GUID (and any material link) was inherited
/// from a previous instance; otherwise the GUID is freshly generated.
struct ReconciledInstance {
  uint32_t instance_id = 0;
  int semantic_class = -1;
  int point_count = 0;
  std::string guid;
  bool carried_over = false;
  uint32_t matched_old_id = 0; ///< Old instance_id matched (0 = none).
};

/// A previous instance whose GUID could not be matched to any new instance.
struct OrphanedInstance {
  uint32_t instance_id = 0;
  int semantic_class = -1;
  std::string guid;
  double best_overlap = 0.0; ///< Best mutual overlap seen (for diagnostics).
};

/// Result of reconciling a freshly-segmented instance set against a previous
/// one. @ref instances always covers every new instance (matched or fresh).
struct ReconcileResult {
  std::vector<ReconciledInstance> instances;
  std::vector<OrphanedInstance> orphaned_old; ///< Old GUIDs with no match.
  size_t matched_count = 0;
  size_t fresh_count = 0;
};

/// Prior instance state (typically loaded from ProjectDB::instances()).
struct PriorInstance {
  uint32_t instance_id = 0;
  int semantic_class = -1;
  std::string guid;
};

/// Options controlling instance identity reconciliation.
struct ReconcileOptions {
  /// Minimum mutual point-set overlap (fraction of the smaller instance) for a
  /// new instance to inherit an old GUID. STANDARDS/issue #207 require > 0.5.
  double min_overlap = 0.5;
  /// Fixed seed forwarded to GUID generation callers (not used internally).
  unsigned int seed = 42;
};

/**
 * @brief Carry stable GUIDs across an instance-cloud regeneration.
 *
 * Both @p old_labels and @p new_labels are index-aligned instance-label clouds
 * over the *same* point cloud (identical indices; 0 = unlabeled). For each new
 * instance we find the old instance of the *same semantic class* with the
 * greatest mutual point-set overlap (intersection over the smaller instance);
 * if that overlap exceeds @ref ReconcileOptions::min_overlap the new instance
 * inherits the old GUID, otherwise it receives a fresh GUID from
 * @p make_guid. Matching is greedy by descending overlap and one-to-one: each
 * old GUID is carried to at most one new instance.
 *
 * @param old_labels    Previous instance-label cloud (may be null/empty).
 * @param new_labels    Newly-segmented instance-label cloud.
 * @param prior         Prior instance rows (semantic class + GUID per old id).
 * @param new_semantic  Map: new instance_id -> semantic class.
 * @param new_sizes     Map: new instance_id -> point count.
 * @param make_guid     Factory for fresh GUIDs (e.g. a UUID generator).
 * @param opts          Reconciliation options.
 * @throws std::invalid_argument if old/new label clouds differ in size.
 */
ReconcileResult
reconcile_instance_identities(const CloudL *old_labels,
                              const CloudL &new_labels,
                              const std::vector<PriorInstance> &prior,
                              const std::map<uint32_t, uint32_t> &new_semantic,
                              const std::map<uint32_t, size_t> &new_sizes,
                              const std::function<std::string()> &make_guid,
                              const ReconcileOptions &opts = {});

} // namespace reusex::geometry
