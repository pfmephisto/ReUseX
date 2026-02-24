#pragma once
#include <ReUseX/vision/IData.hpp>
#include <ReUseX/vision/IDataset.hpp>
#include <ReUseX/vision/tensor_rt/TensorRTData.hpp>

namespace ReUseX::vision::tensor_rt {
class TensorRTDataset : public IDataset {
    public:
  using IDataset::IDataset;
  std::unique_ptr<IData> get(const std::size_t index) const override;
};
} // namespace ReUseX::vision::tensor_rt
