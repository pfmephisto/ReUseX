#pragma once
#include <ReUseX/vision/IData.hpp>
#include <ReUseX/vision/IDataset.hpp>
#include <filesystem>
#include <span>
#include <vector>

namespace ReUseX::vision {
class IModel {
    public:
  // virtual IModel(const std::filesystem::path &path) = 0;

  /* The destructor is declared as virtual to ensure that when an object of a
   * derived class is deleted through a pointer to the base class (IModel), the
   * destructor of the derived class is called, allowing for proper cleanup of
   * resources. This is important in C++ to prevent memory leaks and ensure that
   * any resources allocated by the derived class are released correctly when
   * the object is destroyed. */
  virtual ~IModel() = default;

  /* The create function is a static member function that serves as a factory
   * method for creating instances of classes that implement the IModel
   * interface. It takes a file path as an argument, which is likely used to
   * load a model from a file. The function returns a unique pointer to an
   * IModel instance, allowing for dynamic memory management and ensuring that
   * the created model is properly destroyed when it goes out of scope. This
   * design allows for flexibility in creating different types of models based
   * on the provided file path, while adhering to the IModel interface.
   * @param model_path The file path to the model that needs to be created.
   * @return A unique pointer to an instance of a class that implements the
   * IModel interface.
   */
  static std::unique_ptr<IModel>
  create(const std::filesystem::path &model_path);

  /* The forward function is a pure virtual function that must be implemented by
   * any class that inherits from the IModel interface. It takes a span of
   * IDataset::Pair objects as input and returns a vector of IDataset::Pair
   * objects as output. This function is likely responsible for performing the
   * forward pass of the model, processing the input data and producing the
   * corresponding output. The use of std::span allows for efficient handling of
   * contiguous sequences of data without the overhead of copying, while the
   * return type of std::vector provides flexibility in managing the output
   * data.
   * @param input A span of IDataset::Pair objects representing the input data
   * to be processed by the model.
   * @return A vector of IDataset::Pair objects representing the output produced
   * by the model after processing the input data.
   */
  virtual std::vector<IDataset::Pair>
  forward(const std::span<IDataset::Pair> &input) = 0;

  // virtual void save(const std::string &path) const = 0;
  // virtual std::vector<float> predict(const std::vector<float> &input) = 0;
};
} // namespace ReUseX::vision
