// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/logging.hpp"
#include "io/RTABMapDatabase.hpp"
#include "vision/libtorch/Dataset.hpp"
#include "vision/utils.hpp"

#include <opencv2/core.hpp>

namespace ReUseX::vision::libtorch {

TorchDataset::TorchDataset(std::filesystem::path dbPath)
    : db_(std::make_shared<io::RTABMapDatabase>(std::move(dbPath), false)) {

  // Cache node IDs from database
  ids_ = db_->getNodeIds(false);

  ReUseX::core::trace("TorchDataset initialized with {} entries", ids_.size());
}

TorchDataset::Example TorchDataset::get(size_t index) {
  int node_id = ids_.at(index);

  // Get image from database (already rotated by RTABMapDatabase)
  cv::Mat img = db_->getImage(node_id);

  // Apply letterbox transformation for YOLO-style models
  letterbox(img, img, cv::Size(640, 640));

  // Convert to torch tensor
  torch::Tensor tdata =
      torch::from_blob(img.data, {img.rows, img.cols, 3}, torch::kByte);
  tdata = tdata.toType(torch::kFloat32).div(255);
  tdata = tdata.permute({2, 0, 1}); // Change to CxHxW

  return {tdata, torch::tensor(node_id, torch::kLong)};
}

torch::optional<size_t> TorchDataset::size() const { return ids_.size(); }

void TorchDataset::save(std::vector<cv::Mat> imgs, torch::Tensor index) {
  ReUseX::core::trace("TorchDataset saving {} images", imgs.size());

  // Prepare node IDs and images for batch save
  std::vector<int> nodeIds;
  std::vector<cv::Mat> processedImgs;

  for (size_t i = 0; i < imgs.size(); ++i) {
    int nodeId = index[i].item<int>();
    nodeIds.push_back(nodeId);

    // Apply cropbox transformation (reverse of letterbox)
    cv::Mat processed;
    cropbox(imgs[i], processed, cv::Size(480, 640));

    // Note: rotation is handled by RTABMapDatabase when autoRotate=true
    processedImgs.push_back(processed);
  }

  // Use RTABMapDatabase for batch save with transaction
  // autoRotate=true means RTABMapDatabase will apply the 90° counterclockwise
  // rotation
  db_->saveLabels(nodeIds, processedImgs, true);

  ReUseX::core::trace("TorchDataset save completed");
}

} // namespace ReUseX::vision::libtorch
