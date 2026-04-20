// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/logging.hpp"
#include "vision/onnx/Sam3Data.hpp"
#include "vision/onnx/Sam3Dataset.hpp"

namespace ReUseX::vision::onnx {

IDataset::Pair ONNXSam3Dataset::get(const std::size_t index) const {
  ReUseX::core::trace("ONNXSam3Dataset getting data at index {}", index);

  auto data = std::make_unique<ONNXSam3Data>();
  data->image = image(index);

  return std::make_pair(std::move(data), index);
}

bool ONNXSam3Dataset::save(const std::span<IDataset::Pair> &data) {
  ReUseX::core::info("Saving {} ONNX SAM3 dataset items", data.size());

  bool success = true;
  for (const auto &[item, index] : data) {
    auto *sam3_data = dynamic_cast<ONNXSam3Data *>(item.get());
    if (!sam3_data) {
      ReUseX::core::error(
          "Failed to cast IData to ONNXSam3Data at index {}", index);
      success = false;
      continue;
    }

    success &= save_image(index, sam3_data->image);
  }

  ReUseX::core::debug("Save operation completed with success={}", success);
  return success;
}

} // namespace ReUseX::vision::onnx
