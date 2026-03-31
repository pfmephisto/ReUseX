// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/vision/IMLBackend.hpp"
#include "reusex/vision/libtorch/Dataset.hpp"
#include "reusex/vision/libtorch/Yolo.hpp"

namespace ReUseX::vision::libtorch {

/** @brief LibTorch backend implementation of the IMLBackend interface.
 *
 * Provides factory methods for creating LibTorch-based models and datasets
 * for use with the backend-agnostic annotation pipeline.
 */
class LibTorchBackend : public IMLBackend {
    public:
  LibTorchBackend() = default;

  /** @brief Create a LibTorch model.
   * @param type Model type (currently only Yolo is supported).
   * @param modelPath Path to the TorchScript model file.
   * @param use_cuda Whether to use CUDA for inference (defaults to false).
   * @return Unique pointer to the created model.
   * @throws std::runtime_error if model type is not supported.
   */
  std::unique_ptr<IModel>
  create_model(const Model type,
               const std::filesystem::path &modelPath,
               bool use_cuda = false) override;

  /** @brief Create a LibTorch dataset from a database path.
   * @param datasetPath Path to the RTABMap database file.
   * @return Unique pointer to the created dataset.
   */
  std::unique_ptr<IDataset>
  create_dataset(const std::filesystem::path &datasetPath) override;
};

} // namespace ReUseX::vision::libtorch
