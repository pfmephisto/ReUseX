// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/vision/IDataset.hpp"
#include "reusex/vision/onnx/Sam3Data.hpp"

namespace reusex::vision::onnx {

/// @brief Dataset for ONNX Runtime SAM3 model inference.
///
/// Loads raw images from the ProjectDB and wraps them in ONNXSam3Data.
/// Saves label images back to the database after inference.
class ONNXSam3Dataset : public IDataset {
    public:
  using IDataset::IDataset;

  /// @brief Load an image at the given index into an ONNXSam3Data.
  /// @param index Dataset index (0 to size()-1).
  /// @return Pair of (Sam3Data with raw image, index).
  IDataset::Pair get(const std::size_t index) const override;

  /// @brief Save label images from completed inference back to the database.
  /// @param data Span of pairs containing ONNXSam3Data with label images.
  /// @return true if all saves succeeded.
  bool save(const std::span<Pair> &data) override;

    private:
  bool class_map_saved_ = false;
};

} // namespace reusex::vision::onnx
