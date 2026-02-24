#pragma once
#include <ReUseX/vision/IDataset.hpp>
#include <ReUseX/vision/IModel.hpp>
#include <filesystem>

namespace ReUseX::vision {

enum class Model { Yolo, Sam3 };

class IMLBackend {
    public:
  virtual ~IMLBackend() = default;

  virtual std::unique_ptr<IModel>
  createModel(const Model type, const std::filesystem::path &modelPath) = 0;
  virtual std::unique_ptr<IDataset>
  createDataset(const std::filesystem::path &datasetPath) = 0;
};
} // namespace ReUseX::vision
