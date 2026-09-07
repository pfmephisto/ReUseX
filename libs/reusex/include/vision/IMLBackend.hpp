#pragma once
#include "reusex/vision/IDataset.hpp"
#include "reusex/vision/IModel.hpp"
#include "reusex/vision/IVideoModel.hpp"

#include <filesystem>
#include <stdexcept>

namespace reusex::vision {

enum class Model { yolo, sam3, sam3p1 };

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
   * @param use_cuda Whether to use CUDA for inference (defaults to false).
   * @return A unique pointer to the created IModel object.
   */
  virtual std::unique_ptr<IModel>
  create_model(const Model type, const std::filesystem::path &modelPath,
               bool use_cuda = false) = 0;

  /* Creates a stateful video model (IVideoModel) of the given type. Unlike
   * create_model, this returns a temporally-stateful tracker that MUST be driven
   * in frame order (see IVideoModel). The default implementation throws, so
   * backends without a video path compile unchanged; backends that support it
   * (e.g. TensorRT/SAM 3.1) override this.
   *
   * @param type The type of video model to create.
   * @param modelPath The filesystem path to the model directory.
   * @param use_cuda Whether to use CUDA for inference (defaults to false).
   * @return A unique pointer to the created IVideoModel object.
   */
  virtual std::unique_ptr<IVideoModel>
  create_video_model(const Model type, const std::filesystem::path &modelPath,
                     bool use_cuda = false) {
    (void)type;
    (void)modelPath;
    (void)use_cuda;
    throw std::runtime_error("backend has no video model");
  }

  /* Creates a dataset from the specified path.
   * @param datasetPath The filesystem path to the dataset.
   * @return A unique pointer to the created IDataset object.
   */
  virtual std::unique_ptr<IDataset>
  create_dataset(const std::filesystem::path &datasetPath) = 0;
};
} // namespace reusex::vision
