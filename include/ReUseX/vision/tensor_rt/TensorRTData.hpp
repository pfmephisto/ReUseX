#pragma once
#include <ReUseX/vision/IData.hpp>
#include <opencv4/opencv2/core/mat.hpp>

namespace ReUseX::vision::tensor_rt {
struct TensorRTData : IData {
  cv::Mat image;
};
} // namespace ReUseX::vision::tensor_rt
