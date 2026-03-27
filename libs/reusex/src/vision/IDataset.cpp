// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/logging.hpp"
#include "core/project_db.hpp"
#include "vision/IDataset.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#ifndef NDEBUG
#include <opencv2/highgui.hpp>
#include <pcl/common/colors.h>

namespace {
const cv::Mat &getGlasbeyLUT() {
  static cv::Mat lut = [] {
    cv::Mat m(1, 256, CV_8UC3);
    for (int i = 0; i < 255; ++i) {
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

auto lut = getGlasbeyLUT();
} // namespace
#endif

namespace ReUseX::vision {

IDataset::IDataset(std::shared_ptr<ProjectDB> database)
    : db_(std::move(database)) {
  if (!db_) {
    throw std::runtime_error("Database pointer is null");
  }

  ReUseX::core::info("Initializing IDataset with database: {}", db_->getPath());

  // Cache sensor frame IDs from database
  ids_ = db_->getSensorFrameIds();

  ReUseX::core::info("Dataset initialized with {} entries", ids_.size());
}

IDataset::IDataset(std::filesystem::path dbPath)
    : IDataset(
          std::make_shared<ProjectDB>(std::move(dbPath), false)) {}

cv::Mat IDataset::getImage(const size_t index) const {
  ReUseX::core::trace("Getting image at index {}", index);
  int node_id = ids_.at(index);
  return db_->getSensorFrameImage(node_id);
}

bool IDataset::saveImage(const size_t index, const cv::Mat &image) {
  ReUseX::core::trace("Saving image at index {}", index);

  try {
    int node_id = ids_.at(index);
    db_->saveSegmentationImage(node_id, image);

#ifndef NDEBUG
    cv::Mat temp = image.clone();
    temp = temp & 255;
    temp.convertTo(temp, CV_8U);
    cv::cvtColor(temp, temp, cv::COLOR_GRAY2BGR);
    cv::LUT(temp, lut, temp);
    cv::imshow("Annotation", temp);
    cv::waitKey(1);
#endif

    return true;
  } catch (const std::exception &e) {
    ReUseX::core::error("Failed to save image at index {}: {}", index,
                        e.what());
    return false;
  }
}

std::shared_ptr<ProjectDB> IDataset::getDatabase() const {
  return db_;
}

size_t IDataset::size() const { return ids_.size(); }

} // namespace ReUseX::vision
