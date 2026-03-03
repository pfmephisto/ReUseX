// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ReUseX/vision/IDataset.hpp>
#include <ReUseX/vision/utils.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <spdlog/fmt/std.h>
#include <spdlog/spdlog.h>

#include <sqlite3.h>

namespace ReUseX::vision {

IDataset::IDataset(std::filesystem::path dbPath) {
  spdlog::info("Initializing IDataset with database: {}", dbPath);

  if (sqlite3_open(dbPath.string().c_str(), &db_) != SQLITE_OK) {
    spdlog::error("Cannot open database: {}", sqlite3_errmsg(db_));
    sqlite3_close(db_);
    throw std::runtime_error("Cannot open database");
  }

  // Create Segmentation table if it doesn't exist
  if (sqlite3_exec(db_,
                   "CREATE TABLE IF NOT EXISTS Segmentation ("
                   "id INTEGER PRIMARY KEY, "
                   "label_image BLOB NOT NULL,"
                   "FOREIGN KEY(id) REFERENCES Node(id));",
                   nullptr, nullptr, nullptr) != SQLITE_OK) {
    spdlog::error("Failed to create Segmentation table: {}",
                  sqlite3_errmsg(db_));
    sqlite3_close(db_);
    throw std::runtime_error("Failed to create Segmentation table");
  }

  // Cache node ids
  sqlite3_stmt *stmt;
  sqlite3_prepare_v2(db_, "SELECT id FROM Node;", -1, &stmt, nullptr);
  while (sqlite3_step(stmt) == SQLITE_ROW)
    ids_.push_back(sqlite3_column_int(stmt, 0));
  sqlite3_finalize(stmt);

  spdlog::info("Database initialized with {} entries", ids_.size());
}

cv::Mat IDataset::getImage(const size_t index) const {
  spdlog::trace("Getting image at index {}", index);
  int node_id = ids_.at(index);
  sqlite3_stmt *stmt;
  sqlite3_prepare_v2(db_, "SELECT image FROM Data WHERE id=?;", -1, &stmt,
                     nullptr);
  sqlite3_bind_int(stmt, 1, node_id);

  cv::Mat img;
  int idx = 0;
  if (sqlite3_step(stmt) == SQLITE_ROW) {
    const void *data = sqlite3_column_blob(stmt, idx);
    int datasize = sqlite3_column_bytes(stmt, idx++);
    img = cv::Mat(1, datasize, CV_8UC1, (void *)data);
    img = cv::imdecode(img, cv::IMREAD_UNCHANGED);
    spdlog::trace("Image at index {} decoded: {}x{}", index, img.cols,
                  img.rows);
  } else {
    spdlog::warn("No image found for index {} (node_id {})", index, node_id);
  }
  sqlite3_finalize(stmt);

  cv::rotate(img, img, cv::ROTATE_90_CLOCKWISE);
  // letterbox(img, img, cv::Size(640, 640));
  return img;
}

bool IDataset::saveImage(const size_t index, const cv::Mat &image) {
  spdlog::error("Saving images is not implemented yet");
  return false;
}

IDataset::~IDataset() {
  // TODO: Make sure the destruction is implemented correctly
  spdlog::trace("Closing database connection");
  sqlite3_close(db_);
};

size_t IDataset::size() const { return ids_.size(); }

} // namespace ReUseX::vision
