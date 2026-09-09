// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "vision/common/image.hpp"

namespace reusex::vision::common::tensor {

Image cvimg(const cv::Mat &image) {
  return Image(image.data, image.cols, image.rows);
}

} // namespace reusex::vision::common::tensor
