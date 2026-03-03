#pragma once
#include <ReUseX/vision/IDataset.hpp>
#include <ReUseX/vision/IModel.hpp>
#include <filesystem>

namespace ReUseX::vision {

enum class Model { Yolo, Sam3 };

class IMLBackend {
    public:
  /* The destructor is declared as virtual to ensure that the correct destructor
   * is called when an object of a derived class is deleted through a pointer to
   * the base class. This is important for proper resource management and to
   * avoid memory leaks. By declaring the destructor as virtual, we allow for
   * polymorphic behavior, enabling the correct cleanup of resources allocated
   * by derived classes when they are destroyed through a base class pointer. */
  virtual ~IMLBackend() = default;

  /* The createModel function is a pure virtual function that must be
   * implemented by any class that inherits from IMLBackend. It takes a Model
   * type and a filesystem path to the model as parameters and returns a unique
   * pointer to an IModel object. The function is responsible for creating and
   * initializing the appropriate model based on the specified type and model
   * path. The use of std::unique_ptr ensures that the created model object is
   * properly managed and will be automatically deallocated when it goes out of
   * scope, preventing memory leaks. The createDataset function is also a pure
   * virtual function that must be implemented by derived classes. It takes a
   * filesystem path to the dataset as a parameter and returns a unique pointer
   * to an IDataset object. This function is responsible for creating and
   * initializing the dataset based on the provided dataset path. Similar to
   * createModel, the use of std::unique_ptr ensures proper memory management
   * for the created dataset object.
   * @param type The type of model to create, specified as an enum value.
   * @param modelPath The filesystem path to the model file.
   * @return A unique pointer to the created IModel object.
   */
  virtual std::unique_ptr<IModel>
  createModel(const Model type, const std::filesystem::path &modelPath) = 0;

  /* The createDataset function is a pure virtual function that must be
   * implemented by any class that inherits from IMLBackend. It takes a
   * filesystem path to the dataset as a parameter and returns a unique pointer
   * to an IDataset object. This function is responsible for creating and
   * initializing the dataset based on the provided dataset path. Similar to
   * createModel, the use of std::unique_ptr ensures proper memory management
   * for the created dataset object.
   * @param datasetPath The filesystem path to the dataset.
   * @return A unique pointer to the created IDataset object.
   */
  virtual std::unique_ptr<IDataset>
  createDataset(const std::filesystem::path &datasetPath) = 0;
};
} // namespace ReUseX::vision
