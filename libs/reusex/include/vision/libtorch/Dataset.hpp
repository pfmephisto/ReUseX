// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <opencv2/core.hpp>
#include <torch/torch.h>

#include <filesystem>
#include <memory>
#include <vector>

// Forward declaration
namespace ReUseX {
class ProjectDB;
}

namespace ReUseX::vision::libtorch {
/**
 * @brief PyTorch Dataset for ReUseX project databases
 *
 * This dataset class implements the torch::data::datasets::Dataset interface
 * for use with PyTorch DataLoaders. It uses ProjectDB internally for
 * database access, eliminating code duplication.
 */
class TorchDataset : public torch::data::datasets::Dataset<TorchDataset> {
  using Example = torch::data::Example<>;

    public:
  /**
   * @brief Construct TorchDataset from database path
   * @param dbPath Path to ReUseX project database file
   */
  TorchDataset(std::filesystem::path dbPath = "");

  /**
   * @brief Get a data sample at the given index
   *
   * Returns a torch::data::Example containing the image tensor and node ID.
   * Images are automatically letterboxed to 640x640 for YOLO-style models.
   *
   * @param index Index in the dataset (0 to size()-1)
   * @return Example with image tensor (CHW format, normalized to [0,1]) and node ID
   */
  Example get(size_t index);

  /**
   * @brief Get the number of samples in the dataset
   * @return Optional size (always returns a value)
   */
  torch::optional<size_t> size() const;

  /**
   * @brief Save label images for multiple nodes
   *
   * @param imgs Vector of label images to save
   * @param index Tensor containing node IDs corresponding to each image
   */
  void save(std::vector<cv::Mat> imgs, torch::Tensor index);

    private:
  std::shared_ptr<ProjectDB> db_;
  std::vector<int> ids_;
};
} // namespace ReUseX::vision::libtorch
