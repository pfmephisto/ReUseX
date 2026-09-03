// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace reusex::vision {

/// Configuration for ML annotation inference and data loading
struct AnnotationConfig {
  bool use_cuda = false;  ///< Force CUDA acceleration
  size_t batch_size = 16; ///< Batch size for inference (recommended: 8-64)
  bool shuffle = false;   ///< Shuffle dataset before processing
  size_t num_workers = 4; ///< Number of worker threads (recommended: 2-4)
  size_t prefetch_batches =
      8; ///< Batches to prefetch (recommended: 2-3x workers)
  bool skip_annotated = false; ///< Skip frames that already have segmentation
  float confidence = 0.5f;     ///< Detection confidence threshold [0,1]
  std::vector<std::string>
      prompts; ///< Concept/class prompts (empty = model default)
  std::optional<uint32_t> seed =
      42; ///< Shuffle RNG seed (fixed=deterministic, nullopt=entropy)
  bool video = false; ///< Use the stateful video-tracker path (SAM 3.1). Frames
                      ///< are processed in temporal order on a single thread;
                      ///< the shuffled Dataloader is bypassed.
};

/**
 * @brief Run semantic annotation on sensor frames using ML models
 *
 * @param dbPath Path to project database containing sensor frames
 * @param modelPath Path to ML model (.pt, .engine, .onnx)
 * @param config Configuration for inference and data loading
 * @return 0 on success, error code on failure
 */
auto annotate(const std::filesystem::path &dbPath,
              const std::filesystem::path &modelPath,
              const AnnotationConfig &config = AnnotationConfig{}) -> int;

/**
 * @brief Whether the video tracker must reset before the frame at @p index.
 *
 * A stateful video model must start a fresh temporal memory at the beginning of
 * every ordered sequence. Within one scan the sensor-frame node ids increase
 * monotonically (small forward gaps from dropped frames are normal and must NOT
 * trigger a reset), so a boundary is only the very first frame (index 0) or a
 * point where the node id fails to increase — which happens when a dataset
 * concatenates multiple sequences that restart their numbering.
 *
 * Kept as a free function (independent of any tracker/dataset object) so the
 * boundary logic can be unit-tested directly.
 *
 * @param index    Position in the (possibly prefix-trimmed) ordered list.
 * @param node_id  Node id of the frame at @p index.
 * @param prev_id  Node id of the frame at @p index-1 (unused when index==0).
 * @return true if the tracker should reset before processing this frame.
 */
inline bool is_sequence_boundary(std::size_t index, int node_id, int prev_id) {
  return index == 0 || node_id <= prev_id;
}

} // namespace reusex::vision
