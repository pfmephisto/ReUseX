#include "reusex/utils/cv.hpp"
#include <pcl/common/colors.h>

namespace ReUseX::utils {
const cv::Mat &get_glasbey_lut() {
  static cv::Mat lut = [] {
    cv::Mat m(1, 256, CV_8UC3);
    for (int i = 1; i < 255; ++i) {
      const auto &c = pcl::GlasbeyLUT::at(i);
      m.at<cv::Vec3b>(0, i) =
          cv::Vec3b(static_cast<uchar>(c.b), static_cast<uchar>(c.g),
                    static_cast<uchar>(c.r));
    }
    m.at<cv::Vec3b>(0, 255) = cv::Vec3b(0, 0, 0); // Background color (black)
    return m;
  }();
  return lut;
}
} // namespace ReUseX::utils
