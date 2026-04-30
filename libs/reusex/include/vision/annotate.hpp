// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <filesystem>

namespace reusex::vision {

/// Configuration for ML annotation inference and data loading
struct AnnotationConfig {
  bool use_cuda = false;           ///< Force CUDA acceleration
  size_t batch_size = 16;          ///< Batch size for inference (recommended: 8-64)
  bool shuffle = false;            ///< Shuffle dataset before processing
  size_t num_workers = 4;          ///< Number of worker threads (recommended: 2-4)
  size_t prefetch_batches = 8;     ///< Batches to prefetch (recommended: 2-3x workers)
  bool skip_annotated = false;     ///< Skip frames that already have segmentation
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

} // namespace reusex::vision
