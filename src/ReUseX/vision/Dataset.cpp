// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ReUseX/vision/Dataset.hpp"
#include "ReUseX/vision/utils.hpp"

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/imgproc.hpp>

#include <opencv4/opencv2/highgui.hpp>
#include <spdlog/spdlog.h>

#include <sqlite3.h>

namespace ReUseX::vision {
Dataset::Dataset(fs::path dbPath) {

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

  spdlog::trace("Database initialized with {} entries", ids_.size());
};

Dataset::Example Dataset::get(size_t index) {
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
  }
  sqlite3_finalize(stmt);

  cv::rotate(img, img, cv::ROTATE_90_CLOCKWISE);
  letterbox(img, img, cv::Size(640, 640));

  // Convert to torch tensor
  torch::Tensor tdata = torch::from_blob(img.data, {img.rows, img.cols, 3},
                                         torch::kByte); //.clone();
  tdata = tdata.toType(torch::kFloat32).div(255);
  tdata = tdata.permute({2, 0, 1}); // Change to CxHxW

  return {tdata, torch::tensor(node_id, torch::kLong)};
};

torch::optional<size_t> Dataset::size() const { return ids_.size(); }

void Dataset::save(std::vector<cv::Mat> imgs, torch::Tensor index) {

  sqlite3_exec(db_, "BEGIN TRANSACTION;", nullptr, nullptr, nullptr);

  // Prepare insert statement once
  sqlite3_stmt *stmt;
  if (sqlite3_prepare_v2(
          db_,
          "INSERT INTO Segmentation (id, label_image) VALUES (?, ?) ON "
          "CONFLICT(id) DO UPDATE SET label_image = excluded.label_image;",
          //"UPDATE Segmentation SET label_image = ? WHERE id = ?;",
          -1, &stmt, nullptr) != SQLITE_OK) {
    spdlog::error("Failed to prepare statement: {}", sqlite3_errmsg(db_));
    sqlite3_close(db_);
    throw std::runtime_error("Failed to prepare statement");
  }

  for (size_t i = 0; i < imgs.size(); ++i) {

    cropbox(imgs[i], imgs[i], cv::Size(480, 640));
    cv::rotate(imgs[i], imgs[i], cv::ROTATE_90_COUNTERCLOCKWISE);

    std::vector<unsigned char> bytes;

    // save in 8bits-4channel
    cv::Mat bgra(imgs[i].size(), CV_8UC4, imgs[i].data);
    cv::imencode(".png", bgra, bytes);

    // Bind parameters
    sqlite3_bind_int(stmt, 1, index[i].item<int>());
    sqlite3_bind_blob(stmt, 2, bytes.data(), (int)bytes.size(), SQLITE_STATIC);

    // Execute
    if (sqlite3_step(stmt) != SQLITE_DONE) {
      spdlog::error("Insert failed: {}", sqlite3_errmsg(db_));
    }

    // Reset for next iteration
    sqlite3_reset(stmt);
  }

  // Finalize statement and commit transaction
  sqlite3_finalize(stmt);
  sqlite3_exec(db_, "COMMIT;", nullptr, nullptr, nullptr);
};

} // namespace ReUseX::vision
