#pragma once
#include "reusex/vision/IDataset.hpp"
#include "reusex/vision/IModel.hpp"

#include <filesystem>

namespace ReUseX::vision {

enum class Model { yolo, sam3 };

class IMLBackend {
    public:
  /* The destructor is declared as virtual to ensure that the correct destructor
   * is called when an object of a derived class is deleted through a pointer to
   * the base class. This is important for proper resource management and to
   * avoid memory leaks. By declaring the destructor as virtual, we allow for
   * polymorphic behavior, enabling the correct cleanup of resources allocated
   * by derived classes when they are destroyed through a base class pointer. */
  virtual ~IMLBackend() = default;

  /* Creates a model of the given type from the specified path.
   *
   * @param type The type of model to create, specified as an enum value.
   * @param modelPath The filesystem path to the model file.
   * @return A unique pointer to the created IModel object.
   */
  virtual std::unique_ptr<IModel>
  create_model(const Model type, const std::filesystem::path &modelPath) = 0;

  /* Creates a dataset from the specified path.
   * @param datasetPath The filesystem path to the dataset.
   * @return A unique pointer to the created IDataset object.
   */
  virtual std::unique_ptr<IDataset>
  create_dataset(const std::filesystem::path &datasetPath) = 0;
};
} // namespace ReUseX::vision
