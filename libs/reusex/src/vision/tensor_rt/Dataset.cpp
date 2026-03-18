#include "core/logging.hpp"
#include "vision/IData.hpp"
#include "vision/IDataset.hpp"
#include "vision/tensor_rt/Data.hpp"
#include "vision/tensor_rt/Dataset.hpp"

namespace ReUseX::vision::tensor_rt {

ReUseX::vision::IDataset::Pair
TensorRTDataset::get(const std::size_t index) const {
  ReUseX::core::trace("TensorRTDataset getting data at index {}", index);
  auto data = std::make_pair(std::make_unique<TensorRTData>(), index);
  data.first->image = getImage(index);
  return data;
}

bool TensorRTDataset::save(const std::span<IDataset::Pair> &data) {
  ReUseX::core::info("Saving {} TensorRT dataset items", data.size());

  bool success = true;
  for (const auto &[item, index] : data) {
    TensorRTData *data = dynamic_cast<TensorRTData *>(item.get());
    cv::Mat &img = data->image;

    success &= saveImage(index, img);
  }

  ReUseX::core::debug("Save operation completed with success={}", success);
  return success;
}
} // namespace ReUseX::vision::tensor_rt
