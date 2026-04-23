// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/vision/IMLBackend.hpp"

namespace reusex::vision::onnx {

/// @brief ONNX Runtime backend implementation of the IMLBackend interface.
///
/// Provides factory methods for creating ONNX Runtime-based models and datasets
/// for use with the backend-agnostic annotation pipeline.
class ONNXBackend : public IMLBackend {
    public:
  ONNXBackend() = default;

  /// @brief Create an ONNX Runtime model.
  /// @param type Model type (currently Sam3).
  /// @param modelPath Path to directory containing ONNX model files.
  /// @param use_cuda Whether to use CUDA execution provider (defaults to false).
  /// @return Unique pointer to the created model.
  /// @throws std::runtime_error if model type is not supported.
  std::unique_ptr<IModel>
  create_model(const Model type,
               const std::filesystem::path &modelPath,
               bool use_cuda = false) override;

  /// @brief Create an ONNX dataset from a database path.
  /// @param datasetPath Path to the ReUseX project database file.
  /// @return Unique pointer to the created dataset.
  std::unique_ptr<IDataset>
  create_dataset(const std::filesystem::path &datasetPath) override;
};

} // namespace reusex::vision::onnx
