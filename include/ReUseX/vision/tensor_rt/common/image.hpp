#pragma once
#include <opencv4/opencv2/opencv.hpp>

namespace ReUseX::vision::tensor_rt::tensor {

struct Image {
  const void *bgrptr = nullptr;
  int width = 0, height = 0;

  Image() = default;
  Image(const void *bgrptr, int width, int height)
      : bgrptr(bgrptr), width(width), height(height) {}
};

Image cvimg(const cv::Mat &image);

} // namespace ReUseX::vision::tensor_rt::tensor
