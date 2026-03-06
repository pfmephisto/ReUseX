#pragma once
#include <ReUseX/vision/IData.hpp>
#include <ReUseX/vision/IDataset.hpp>
#include <ReUseX/vision/tensor_rt/Data.hpp>

namespace ReUseX::vision::tensor_rt {
/* TensorRTDataset is a dataset class that manages TensorRTData objects. It
 * inherits from IDataset and implements the get and save methods to handle
 * TensorRTData instances. This class allows for efficient storage and retrieval
 * of data in a format optimized for TensorRT operations. */
class TensorRTDataset : public IDataset {
    public:
  using IDataset::IDataset;

  /* The get method retrieves a Pair of TensorRTData objects from the dataset
   * based on the provided index. It overrides the virtual method from the
   * IDataset interface to return data in the specific format used by TensorRT.
   * @param index The index of the data pair to retrieve.
   * @return A Pair containing the input and output TensorRTData objects. */
  IDataset::Pair get(const std::size_t index) const override;

  /* The save method stores a Pair of TensorRTData objects in the dataset. It
   * overrides the virtual method from the IDataset interface to handle data in
   * the specific format used by TensorRT. This method allows for efficient
   * saving of data pairs that can be later retrieved using the get method.
   * @param data A span containing the Pair of TensorRTData objects to save.
   * @return A boolean value indicating whether the save operation was
   * successful. */
  bool save(const std::span<Pair> &data) override;
};
} // namespace ReUseX::vision::tensor_rt
