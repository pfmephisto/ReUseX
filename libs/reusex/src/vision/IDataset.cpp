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

  ReUseX::info("Initializing IDataset with database: {}", db_->path());

  // Cache sensor frame IDs from database
  ids_ = db_->sensor_frame_ids();

  ReUseX::info("Dataset initialized with {} entries", ids_.size());
}

IDataset::IDataset(std::filesystem::path dbPath)
    : IDataset(std::make_shared<ProjectDB>(std::move(dbPath), false)) {}

cv::Mat IDataset::image(const size_t index) const {
  ReUseX::trace("Getting image at index {}", index);
  int node_id = ids_.at(index);
  return db_->sensor_frame_image(node_id);
}

bool IDataset::save_image(const size_t index, const cv::Mat &image) {
  ReUseX::trace("Saving image at index {}", index);

  try {
    int node_id = ids_.at(index);
    db_->save_segmentation_image(node_id, image);

#ifndef NDEBUG
    // Colorize label image
    cv::Mat temp = image.clone();
    temp = temp & 255;
    temp.convertTo(temp, CV_8U);
    cv::cvtColor(temp, temp, cv::COLOR_GRAY2BGR);
    cv::LUT(temp, lut, temp);

    // Blend with RGB for overlay
    cv::Mat rgb = db_->sensor_frame_image(node_id);
    if (!rgb.empty()) {
      if (rgb.channels() == 1)
        cv::cvtColor(rgb, rgb, cv::COLOR_GRAY2BGR);
      if (rgb.size() != temp.size())
        cv::resize(temp, temp, rgb.size());
      cv::Mat blended;
      cv::addWeighted(rgb, 0.6, temp, 0.4, 0, blended);
      cv::imshow("Annotation", blended);
    } else {
      cv::imshow("Annotation", temp);
    }
    cv::waitKey(1);
#endif

    return true;
  } catch (const std::exception &e) {
    ReUseX::error("Failed to save image at index {}: {}", index,
                        e.what());
    return false;
  }
}

std::shared_ptr<ProjectDB> IDataset::database() const { return db_; }

size_t IDataset::size() const { return ids_.size(); }

} // namespace ReUseX::vision
