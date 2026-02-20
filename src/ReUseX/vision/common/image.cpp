#include <ReUseX/vision/common/image.hpp>

namespace ReUseX::vision::tensor {

tensor::Image cvimg(const cv::Mat &image) {
  return Image(image.data, image.cols, image.rows);
}

} // namespace ReUseX::vision::tensor
