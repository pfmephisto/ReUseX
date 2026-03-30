// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "vision/IDataset.hpp"
#include "core/ProjectDB.hpp"
#include "core/logging.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#ifndef NDEBUG
#include "utils/cv.hpp"
namespace {
auto lut = ReUseX::utils::get_glasbey_lut();
}
#endif

namespace ReUseX::vision {

IDataset::IDataset(std::shared_ptr<ProjectDB> database)
    : db_(std::move(database)) {
  if (!db_) {
    throw std::runtime_error("Database pointer is null");
  }

  ReUseX::core::info("Initializing IDataset with database: {}", db_->path());

  // Cache sensor frame IDs from database
  ids_ = db_->sensor_frame_ids();

  ReUseX::core::info("Dataset initialized with {} entries", ids_.size());
}

IDataset::IDataset(std::filesystem::path dbPath)
    : IDataset(std::make_shared<ProjectDB>(std::move(dbPath), false)) {}

cv::Mat IDataset::image(const size_t index) const {
  ReUseX::core::trace("Getting image at index {}", index);
  int node_id = ids_.at(index);
  return db_->sensor_frame_image(node_id);
}

bool IDataset::save_image(const size_t index, const cv::Mat &image) {
  ReUseX::core::trace("Saving image at index {}", index);

  try {
    int node_id = ids_.at(index);
    db_->save_segmentation_image(node_id, image);

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

std::shared_ptr<ProjectDB> IDataset::database() const { return db_; }

size_t IDataset::size() const { return ids_.size(); }

} // namespace ReUseX::vision
