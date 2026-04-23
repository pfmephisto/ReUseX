#pragma once
#include "reusex/vision/IMLBackend.hpp"
#include "reusex/vision/tensor_rt/Dataset.hpp"
#include "reusex/vision/tensor_rt/Sam3.hpp"

namespace reusex::vision::tensor_rt {
/* TensorRTBackend is a concrete implementation of the IMLBackend interface for
 * TensorRT. It provides methods to create TensorRT-based models and datasets.
 */
class TensorRTBackend : public IMLBackend {
    public:
  /* Constructor for TensorRTBackend. Initializes any necessary resources for
   * the backend. In this case, it is a default constructor.
   */
  TensorRTBackend() = default;

  /* Destructor for TensorRTBackend. Cleans up any resources allocated by the
   * backend. In this case, it is a default destructor.
   * @param: type - The type of model to create (not used in this
   * implementation).
   * @param: modelPath - The file path to the model to be created (not used in
   * this implementation).
   * @param: use_cuda - Whether to use CUDA for inference (ignored for TensorRT,
   * always uses GPU).
   * @return: A unique pointer to an IModel instance representing the created
   * model.
   */
  std::unique_ptr<IModel>
  create_model(const Model type,
               const std::filesystem::path &modelPath,
               bool use_cuda = false) override;

  /* Creates a dataset based on the provided dataset path. This method is
   * responsible for initializing and returning a dataset that can be used for
   * training or inference. The dataset is created based on the specified path,
   * which may contain the necessary data and configuration for the dataset.
   * @param: datasetPath - The file path to the dataset to be created.
   * @return: A unique pointer to an IDataset instance representing the created
   * dataset.
   */
  std::unique_ptr<IDataset>
  create_dataset(const std::filesystem::path &datasetPath) override;
};
} // namespace reusex::vision::tensor_rt
