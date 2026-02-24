#include <ReUseX/vision/IData.hpp>
#include <ReUseX/vision/IDataset.hpp>
#include <ReUseX/vision/tensor_rt/TensorRTData.hpp>
#include <ReUseX/vision/tensor_rt/TensorRTDataset.hpp>
#include <spdlog/spdlog.h>

namespace ReUseX::vision::tensor_rt {
std::unique_ptr<IData> TensorRTDataset::get(const std::size_t index) const {
  // TODO: Implement this as compiled code
  spdlog::error("Not implemented yet!");
  return std::unique_ptr<TensorRTData>(new TensorRTData());
}
} // namespace ReUseX::vision::tensor_rt
