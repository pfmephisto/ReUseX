#pragma once
#include <ReUseX/vision/IMLBackend.hpp>
#include <ReUseX/vision/tensor_rt/TensorRTDataset.hpp>
#include <ReUseX/vision/tensor_rt/TensorRTSam3.hpp>

namespace ReUseX::vision::tensor_rt {
class TensorRTBackend : public IMLBackend {
    public:
  TensorRTBackend() = default;

  std::unique_ptr<IModel>
  createModel(const Model type,
              const std::filesystem::path &modelPath) override;

  std::unique_ptr<IDataset>
  createDataset(const std::filesystem::path &datasetPath) override;
};
} // namespace ReUseX::vision::tensor_rt
