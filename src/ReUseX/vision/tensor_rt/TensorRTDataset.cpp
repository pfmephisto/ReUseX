#include <ReUseX/vision/IData.hpp>
#include <ReUseX/vision/IDataset.hpp>
#include <ReUseX/vision/tensor_rt/TensorRTData.hpp>
#include <ReUseX/vision/tensor_rt/TensorRTDataset.hpp>

#include <spdlog/fmt/std.h>
#include <spdlog/spdlog.h>

// FIX: This is a temporary include for testing purposes.
#include <opencv4/opencv2/highgui.hpp>

namespace ReUseX::vision::tensor_rt {

ReUseX::vision::IDataset::Pair
TensorRTDataset::get(const std::size_t index) const {
  spdlog::trace("TensorRTDataset getting data at index {}", index);
  auto data = std::make_pair(std::make_unique<TensorRTData>(), index);
  data.first->image = getImage(index);
  return data;
}

bool TensorRTDataset::save(const std::span<IDataset::Pair> &data) {
  spdlog::info("Saving {} TensorRT dataset items", data.size());

  bool success = true;
  for (const auto &[item, index] : data) {
    TensorRTData *data = dynamic_cast<TensorRTData *>(item.get());
    cv::Mat &img = data->image;
    // TODO: Add postprocessing here
    spdlog::warn("Postprocessing is not implemented yet!");
    // TODO: Reenable the following
    // success &= saveImage(index, img);

    // TODO: Remove this after postprocessing is implemented
    cv::imshow("TensorRTDataset Save", img);
    cv::waitKey(0);
  }

  cv::destroyAllWindows();

  spdlog::debug("Save operation completed with success={}", success);
  return success;
}
} // namespace ReUseX::vision::tensor_rt
