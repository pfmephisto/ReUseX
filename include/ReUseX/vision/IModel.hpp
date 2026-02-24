#pragma once
#include <ReUseX/vision/IData.hpp>
#include <filesystem>
#include <vector>

namespace ReUseX::vision {
class IModel {
    public:
  // virtual IModel(const std::filesystem::path &path) = 0;
  virtual ~IModel() = default;

  static std::unique_ptr<IModel>
  create(const std::filesystem::path &model_path);

  virtual std::vector<IData> forward(const std::vector<IData> &input) const = 0;

  // virtual void save(const std::string &path) const = 0;
  // virtual std::vector<float> predict(const std::vector<float> &input) = 0;
};
} // namespace ReUseX::vision
