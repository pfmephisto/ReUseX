// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/vision/IData.hpp"
#include "reusex/vision/IDataset.hpp"
#include "reusex/vision/libtorch/Data.hpp"

namespace ReUseX::vision::libtorch {

/** @brief LibTorch dataset for RTABMap databases.
 *
 * Implements the IDataset interface for use with the backend-agnostic
 * annotation pipeline. Loads images, applies letterbox preprocessing,
 * and stores results as LibTorchData objects.
 */
class LibTorchDataset : public IDataset {
    public:
  using IDataset::IDataset;

  /** @brief Get a data sample at the given index.
   * @param index Index in the dataset (0 to size()-1)
   * @return Pair containing LibTorchData with letterboxed image and the index
   */
  IDataset::Pair get(const std::size_t index) const override;

  /** @brief Save processed results back to the database.
   * @param data Span of pairs containing LibTorchData with label_image set
   * @return true if all saves succeeded
   */
  bool save(const std::span<Pair> &data) override;
};

} // namespace ReUseX::vision::libtorch
