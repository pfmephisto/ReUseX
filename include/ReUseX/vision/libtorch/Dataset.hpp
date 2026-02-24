// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
// #include <rtabmap/core/DBDriver.h>
#include <ReUseX/vision/IDataset.hpp>
#include <opencv2/core.hpp>
#include <sqlite3.h>
#include <torch/torch.h>

#include <filesystem>
#include <map>
#include <vector>

namespace ReUseX::vision::libtorch {
class TorchDataset : public torch::data::datasets::Dataset<TorchDataset>
//, public ReUseX::vision::IDataset
{
  using Example = torch::data::Example<>;

    public:
  TorchDataset(std::filesystem::path dbPath = "");
  Example get(size_t index);
  torch::optional<size_t> size() const;
  void save(std::vector<cv::Mat> img, torch::Tensor index);

    private:
  sqlite3 *db_ = nullptr;
  std::vector<int> ids_ = {};
};
} // namespace ReUseX::vision::libtorch
