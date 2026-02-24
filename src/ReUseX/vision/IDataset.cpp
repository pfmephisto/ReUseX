// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ReUseX/vision/IDataset.hpp>
#include <ReUseX/vision/utils.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <opencv2/highgui.hpp>
#include <spdlog/spdlog.h>

#include <sqlite3.h>

namespace ReUseX::vision {

IDataset::IDataset(std::filesystem::path dbPath) {

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
}

IDataset::~IDataset() {
  // TODO: Make sure the destruction is implemented correctly
  spdlog::trace("Closing database connection");
  sqlite3_close(db_);
};

size_t IDataset::size() const { return ids_.size(); }

} // namespace ReUseX::vision
