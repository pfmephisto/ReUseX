// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/vision/IModel.hpp"
#include "reusex/vision/libtorch/Data.hpp"

#include <torch/script.h>
#include <torch/torch.h>

#include <filesystem>
#include <memory>
#include <span>
#include <vector>

namespace ReUseX::vision::libtorch {

/** @brief YOLO instance segmentation model using LibTorch (TorchScript).
 *
 * Loads a TorchScript YOLO segmentation model and implements the IModel
 * interface for use with the backend-agnostic annotation pipeline.
 */
class LibTorchYolo : public IModel {
    public:
  /** @brief Construct and load a YOLO model from a TorchScript file.
   * @param model_path Path to the .pt TorchScript model file.
   * @param use_cuda Whether to use CUDA for inference (falls back to CPU).
   */
  explicit LibTorchYolo(const std::filesystem::path &model_path,
                        bool use_cuda = true);

  /** @brief Factory method for creating a LibTorchYolo instance.
   * @param model_path Path to the .pt TorchScript model file.
   * @return Unique pointer to the created model.
   */
  static std::unique_ptr<LibTorchYolo>
  create(const std::filesystem::path &model_path);

  /** @brief Run YOLO inference on a batch of images.
   *
   * Each input Pair must contain a LibTorchData with a letterboxed image.
   * Output Pairs contain the same LibTorchData with label_image populated.
   *
   * @param input Span of (data, index) pairs from the dataset.
   * @return Vector of (data, index) pairs with label_image set.
   */
  std::vector<IDataset::Pair>
  forward(const std::span<IDataset::Pair> &input) override;

    private:
  /** @brief Convert a batch of LibTorchData images to a batched tensor.
   * @param inputs Vector of LibTorchData pointers.
   * @return Tensor of shape [B, 3, H, W] normalized to [0, 1].
   */
  torch::Tensor
  images_to_tensor(const std::vector<const LibTorchData *> &inputs) const;

  torch::jit::script::Module model_;
  torch::Device device_;
};

} // namespace ReUseX::vision::libtorch
