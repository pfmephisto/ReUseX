#include "vision/tensor_rt/Dataset.hpp"
#include "core/ProjectDB.hpp"
#include "core/logging.hpp"
#include "vision/IData.hpp"
#include "vision/IDataset.hpp"
#include "vision/tensor_rt/Data.hpp"

#include <fmt/core.h>
#include <fmt/ranges.h>
#include <range/v3/all.hpp>

#include <sstream>

template <typename R> auto enumerate(R &&r) {
  return ranges::views::zip(ranges::views::iota(0ul, ranges::size(r)),
                            std::forward<R>(r));
}

namespace ReUseX::vision::tensor_rt {

ReUseX::vision::IDataset::Pair
TensorRTDataset::get(const std::size_t index) const {
  ReUseX::core::trace("TensorRTDataset getting data at index {}", index);
  auto data = std::make_pair(std::make_unique<TensorRTData>(), index);
  data.first->image = image(index);
  return data;
}

bool TensorRTDataset::save(const std::span<IDataset::Pair> &data) {
  ReUseX::core::info("Saving {} TensorRT dataset items", data.size());

  bool success = true;
  for (const auto &[item, index] : data) {
    TensorRTData *trt_data = dynamic_cast<TensorRTData *>(item.get());
    cv::Mat &img = trt_data->image;

    success &= save_image(index, img);

    if (!class_map_saved_ && trt_data) {
      auto json = fmt::format(
          "{{{}}}", fmt::join(enumerate(trt_data->prompts) |
                                  ranges::views::transform([](auto pair) {
                                    auto [i, prompt] = pair;
                                    return fmt::format("\"{}\":\"{}\"", i,
                                                       prompt.text);
                                  }),
                              ","));
      database()->log_pipeline_start("annotate_class_map", json);
      class_map_saved_ = true;
      ReUseX::core::info("Saved segmentation class map ({} classes)",
                         trt_data->prompts.size());
    }
  }

  ReUseX::core::debug("Save operation completed with success={}", success);
  return success;
}
} // namespace ReUseX::vision::tensor_rt
