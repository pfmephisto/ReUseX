#pragma once
#include "opencv2/opencv.hpp"

namespace reusex::vision::common::tensor {

/// @brief Lightweight, non-owning view of a BGR image buffer.
///
/// Image wraps a raw pixel pointer together with its dimensions, providing a
/// uniform interface for passing image data into GPU preprocessing pipelines
/// without incurring a copy.
struct Image {
  const void *bgrptr = nullptr; ///< Pointer to the BGR pixel data (row-major).
  int width = 0;                ///< Image width in pixels.
  int height = 0;               ///< Image height in pixels.

  Image() = default;

  /// @brief Construct from a raw pointer and explicit dimensions.
  /// @param bgrptr Pointer to the first byte of BGR pixel data.
  /// @param width  Image width in pixels.
  /// @param height Image height in pixels.
  Image(const void *bgrptr, int width, int height)
      : bgrptr(bgrptr), width(width), height(height) {}
};

/// @brief Create an Image view that wraps an OpenCV Mat (zero-copy).
/// @param image A BGR OpenCV Mat.  The Mat must outlive the returned Image.
/// @return An Image pointing at the Mat's data buffer.
Image cvimg(const cv::Mat &image);

} // namespace reusex::vision::common::tensor
