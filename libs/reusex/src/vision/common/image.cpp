#include "vision/common/image.hpp"

namespace reusex::vision::common::tensor {

Image cvimg(const cv::Mat &image) {
  return Image(image.data, image.cols, image.rows);
}

} // namespace reusex::vision::common::tensor
