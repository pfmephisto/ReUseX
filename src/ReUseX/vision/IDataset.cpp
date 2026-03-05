// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ReUseX/vision/IDataset.hpp>
#include <ReUseX/vision/utils.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <spdlog/fmt/std.h>
#include <spdlog/spdlog.h>

#include <sqlite3.h>
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

  spdlog::trace("Saving image at index {}", index);

  bool success = true;

  auto node_id = ids_.at(index);

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

  spdlog::trace("Rotating image");
  cv::Mat rotated;
  cv::rotate(image, rotated, cv::ROTATE_90_COUNTERCLOCKWISE);

  rotated += 1; // Move -1 to 0 so that it can be
                // stored as unsigned 16-bit integer

  std::vector<unsigned char> bytes;

  spdlog::trace("Encoding image to CV_16U png");
  cv::Mat labels16U;
  rotated.convertTo(labels16U, CV_16U);
  cv::imencode(".png", labels16U, bytes);

  // Bind parameters
  spdlog::trace("Binding parameters for index {}: {} bytes", node_id,
                bytes.size());
  sqlite3_bind_int(stmt, 1, node_id);
  sqlite3_bind_blob(stmt, 2, bytes.data(), (int)bytes.size(), SQLITE_STATIC);

  // Execute
  if (sqlite3_step(stmt) != SQLITE_DONE) {
    spdlog::error("Insert failed: {}", sqlite3_errmsg(db_));
    success = false;
  }

  // Reset for next iteration
  sqlite3_reset(stmt);

  // Finalize statement and commit transaction
  sqlite3_finalize(stmt);
  sqlite3_exec(db_, "COMMIT;", nullptr, nullptr, nullptr);

#ifndef NDEBUG
  cv::Mat temp = image.clone();
  temp = temp & 255;
  temp.convertTo(temp, CV_8U);
  // cv::normalize(temp, temp, 0, 255, cv::NORM_MINMAX, CV_8U);
  cv::cvtColor(temp, temp, cv::COLOR_GRAY2BGR);
  cv::LUT(temp, lut, temp);
  cv::imshow("Annotation", temp);
  cv::waitKey(1);
#endif

  return success;
}

IDataset::~IDataset() {
  // TODO: Make sure the destruction is implemented correctly
  spdlog::trace("Closing database connection");
  sqlite3_close(db_);
};

size_t IDataset::size() const { return ids_.size(); }

} // namespace ReUseX::vision
