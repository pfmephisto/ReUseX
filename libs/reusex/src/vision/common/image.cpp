#include "vision/common/image.hpp"

namespace ReUseX::vision::common::tensor {

Image cvimg(const cv::Mat &image) {
  return Image(image.data, image.cols, image.rows);
}

} // namespace ReUseX::vision::common::tensor
