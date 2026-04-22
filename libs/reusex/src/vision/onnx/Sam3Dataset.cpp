// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/ProjectDB.hpp"
#include "core/logging.hpp"
#include "vision/onnx/Sam3Data.hpp"
#include "vision/onnx/Sam3Dataset.hpp"

#include <sstream>

namespace ReUseX::vision::onnx {

IDataset::Pair ONNXSam3Dataset::get(const std::size_t index) const {
  ReUseX::trace("ONNXSam3Dataset getting data at index {}", index);

  auto data = std::make_unique<ONNXSam3Data>();
  data->image = image(index);

  return std::make_pair(std::move(data), index);
}

bool ONNXSam3Dataset::save(const std::span<IDataset::Pair> &data) {
  ReUseX::info("Saving {} ONNX SAM3 dataset items", data.size());

  bool success = true;
  for (const auto &[item, index] : data) {
    auto *sam3_data = dynamic_cast<ONNXSam3Data *>(item.get());
    if (!sam3_data) {
      ReUseX::error(
          "Failed to cast IData to ONNXSam3Data at index {}", index);
      success = false;
      continue;
    }

    success &= save_image(index, sam3_data->image);

    if (!class_map_saved_) {
      std::ostringstream json;
      json << '{';
      for (size_t i = 0; i < sam3_data->prompts.size(); ++i) {
        if (i > 0)
          json << ',';
        json << '"' << i << "\":\"" << sam3_data->prompts[i].text << '"';
      }
      json << '}';
      database()->log_pipeline_start("annotate_class_map", json.str());
      class_map_saved_ = true;
      ReUseX::info("Saved segmentation class map ({} classes)",
                         sam3_data->prompts.size());
    }
  }

  ReUseX::debug("Save operation completed with success={}", success);
  return success;
}

} // namespace ReUseX::vision::onnx
