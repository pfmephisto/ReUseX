// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/logging.hpp"
#include "vision/libtorch/Data.hpp"
#include "vision/libtorch/Dataset.hpp"
#include "vision/utils.hpp"

#include <opencv2/core.hpp>

namespace reusex::vision::libtorch {

IDataset::Pair LibTorchDataset::get(const std::size_t index) const {
  reusex::trace("LibTorchDataset getting data at index {}", index);

  auto data = std::make_unique<LibTorchData>();
  data->image = image(index);
  data->original_size = data->image.size();
  data->letterbox_scale =
      letterbox(data->image, data->image, cv::Size(data->target_size, data->target_size));

  return std::make_pair(std::move(data), index);
}

bool LibTorchDataset::save(const std::span<IDataset::Pair> &data) {
  reusex::info("Saving {} LibTorch dataset items", data.size());

  bool success = true;
  for (const auto &[item, index] : data) {
    auto *lt_data = dynamic_cast<LibTorchData *>(item.get());
    if (!lt_data) {
      reusex::error("Failed to cast IData to LibTorchData at index {}", index);
      success = false;
      continue;
    }

    // Reverse letterbox: crop label image back to original dimensions
    cv::Mat cropped;
    cropbox(lt_data->label_image, cropped, lt_data->original_size);

    success &= save_image(index, cropped);
  }

  reusex::debug("Save operation completed with success={}", success);
  return success;
}

} // namespace reusex::vision::libtorch
