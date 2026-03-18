// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/logging.hpp"
#include "io/RTABMapDatabase.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <sqlite3.h>

#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Rtabmap.h>

namespace ReUseX::io {

// Pimpl implementation - hides RTABMap types from header
class RTABMapDatabase::Impl {
    public:
  std::filesystem::path dbPath;
  bool readOnly;
  sqlite3 *db = nullptr;
  rtabmap::DBDriver *driver = nullptr;
  rtabmap::Rtabmap *rtabmap = nullptr;

  Impl(std::filesystem::path path, bool ro)
      : dbPath(std::move(path)), readOnly(ro) {

    ReUseX::core::info("Opening RTABMap database: {}", dbPath);

    // Open sqlite3 connection for custom table access
    int flags = readOnly ? SQLITE_OPEN_READONLY
                         : (SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE);
    if (sqlite3_open_v2(dbPath.string().c_str(), &db, flags, nullptr) !=
        SQLITE_OK) {
      std::string error = sqlite3_errmsg(db);
      sqlite3_close(db);
      throw std::runtime_error("Cannot open database: " + error);
    }

    // Create RTABMap DBDriver for high-level operations
    driver = rtabmap::DBDriver::create();
    if (!driver) {
      sqlite3_close(db);
      throw std::runtime_error("Failed to create RTABMap DBDriver");
    }

    if (!driver->openConnection(dbPath.string(), readOnly)) {
      delete driver;
      sqlite3_close(db);
      throw std::runtime_error(
          "DBDriver failed to open connection to database");
    }

    // Create Rtabmap instance for graph operations
    rtabmap = new rtabmap::Rtabmap();
    try {
      rtabmap->init(dbPath.string());
    } catch (const std::exception &e) {
      delete rtabmap;
      delete driver;
      sqlite3_close(db);
      throw std::runtime_error(std::string("Rtabmap failed to initialize: ") +
                               e.what());
    }

    // Create Segmentation table if in write mode
    if (!readOnly) {
      createSegmentationTable();
    }

    ReUseX::core::info("RTABMap database opened successfully");
  }

  ~Impl() {
    ReUseX::core::trace("Closing RTABMap database connection");
    if (rtabmap) {
      delete rtabmap;
    }
    if (driver) {
      driver->closeConnection();
      delete driver;
    }
    if (db) {
      sqlite3_close(db);
    }
  }

  void createSegmentationTable() {
    const char *sql = "CREATE TABLE IF NOT EXISTS Segmentation ("
                      "id INTEGER PRIMARY KEY, "
                      "label_image BLOB NOT NULL, "
                      "FOREIGN KEY(id) REFERENCES Node(id));";

    if (sqlite3_exec(db, sql, nullptr, nullptr, nullptr) != SQLITE_OK) {
      std::string error = sqlite3_errmsg(db);
      throw std::runtime_error("Failed to create Segmentation table: " + error);
    }
    ReUseX::core::trace("Segmentation table created or already exists");
  }

  void validateSchema() const {
    // Check for required RTABMap tables
    const char *tables[] = {"Node", "Data", "Link"};

    for (const char *table : tables) {
      std::string query =
          "SELECT name FROM sqlite_master WHERE type='table' AND name='" +
          std::string(table) + "';";

      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(db, query.c_str(), -1, &stmt, nullptr) !=
          SQLITE_OK) {
        throw std::runtime_error("Schema validation failed: " +
                                 std::string(sqlite3_errmsg(db)));
      }

      bool found = (sqlite3_step(stmt) == SQLITE_ROW);
      sqlite3_finalize(stmt);

      if (!found) {
        throw std::runtime_error("Required table '" + std::string(table) +
                                 "' not found in database");
      }
    }
    ReUseX::core::trace("Database schema validation passed");
  }

  std::vector<int> getNodeIds(bool ignoreChildren) const {
    std::vector<int> ids;

    std::string query =
        ignoreChildren
            ? "SELECT id FROM Node WHERE time_enter >= 0.0;" // Parent nodes
                                                             // only
            : "SELECT id FROM Node;";                        // All nodes

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, query.c_str(), -1, &stmt, nullptr) !=
        SQLITE_OK) {
      throw std::runtime_error("Failed to query node IDs: " +
                               std::string(sqlite3_errmsg(db)));
    }

    while (sqlite3_step(stmt) == SQLITE_ROW) {
      ids.push_back(sqlite3_column_int(stmt, 0));
    }
    sqlite3_finalize(stmt);

    ReUseX::core::trace("Retrieved {} node IDs", ids.size());
    return ids;
  }

  cv::Mat getImage(int nodeId) const {
    ReUseX::core::trace("Getting image for node {}", nodeId);

    sqlite3_stmt *stmt;
    const char *query = "SELECT image FROM Data WHERE id=?;";

    if (sqlite3_prepare_v2(db, query, -1, &stmt, nullptr) != SQLITE_OK) {
      throw std::runtime_error("Failed to prepare image query: " +
                               std::string(sqlite3_errmsg(db)));
    }

    sqlite3_bind_int(stmt, 1, nodeId);

    cv::Mat img;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
      const void *data = sqlite3_column_blob(stmt, 0);
      int datasize = sqlite3_column_bytes(stmt, 0);

      cv::Mat encoded(1, datasize, CV_8UC1, const_cast<void *>(data));
      img = cv::imdecode(encoded, cv::IMREAD_UNCHANGED);

      if (!img.empty()) {
        ReUseX::core::trace("Image decoded: {}x{}", img.cols, img.rows);
        cv::rotate(img, img, cv::ROTATE_90_CLOCKWISE);
      }
    } else {
      ReUseX::core::warn("No image found for node {}", nodeId);
    }

    sqlite3_finalize(stmt);
    return img;
  }

  bool hasSegmentation(int nodeId) const {
    sqlite3_stmt *stmt;
    const char *query = "SELECT 1 FROM Segmentation WHERE id=? LIMIT 1;";

    if (sqlite3_prepare_v2(db, query, -1, &stmt, nullptr) != SQLITE_OK) {
      return false;
    }

    sqlite3_bind_int(stmt, 1, nodeId);
    bool exists = (sqlite3_step(stmt) == SQLITE_ROW);
    sqlite3_finalize(stmt);

    return exists;
  }

  cv::Mat getLabels(int nodeId) const {
    ReUseX::core::trace("Getting labels for node {}", nodeId);

    sqlite3_stmt *stmt;
    const char *query = "SELECT label_image FROM Segmentation WHERE id=?;";

    if (sqlite3_prepare_v2(db, query, -1, &stmt, nullptr) != SQLITE_OK) {
      throw std::runtime_error("Failed to prepare labels query: " +
                               std::string(sqlite3_errmsg(db)));
    }

    sqlite3_bind_int(stmt, 1, nodeId);

    cv::Mat labels;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
      const void *data = sqlite3_column_blob(stmt, 0);
      int datasize = sqlite3_column_bytes(stmt, 0);

      cv::Mat encoded(1, datasize, CV_8UC1, const_cast<void *>(data));
      cv::Mat labels16U = cv::imdecode(encoded, cv::IMREAD_UNCHANGED);

      if (!labels16U.empty()) {
        // Convert CV_16U to CV_32S and apply -1 offset (0 -> -1 for background)
        labels16U.convertTo(labels, CV_32S);
        labels -= 1;

        // Apply rotation to match image coordinates
        cv::rotate(labels, labels, cv::ROTATE_90_CLOCKWISE);

        ReUseX::core::trace("Labels decoded: {}x{}", labels.cols, labels.rows);
      }
    } else {
      ReUseX::core::debug("No labels found for node {}", nodeId);
    }

    sqlite3_finalize(stmt);
    return labels;
  }

  void saveLabels(int nodeId, const cv::Mat &labels, bool autoRotate) {
    ReUseX::core::trace("Saving labels for node {}", nodeId);

    if (labels.empty()) {
      throw std::runtime_error("Cannot save empty labels");
    }

    // Prepare the image data
    cv::Mat toSave = labels.clone();

    // Apply rotation if requested
    if (autoRotate) {
      cv::rotate(toSave, toSave, cv::ROTATE_90_COUNTERCLOCKWISE);
    }

    // Apply +1 offset and convert to CV_16U for storage (0 = background)
    toSave += 1;
    cv::Mat labels16U;
    toSave.convertTo(labels16U, CV_16U);

    // Encode as PNG
    std::vector<unsigned char> bytes;
    if (!cv::imencode(".png", labels16U, bytes)) {
      throw std::runtime_error("Failed to encode labels as PNG");
    }

    // Save to database
    sqlite3_stmt *stmt;
    const char *query =
        "INSERT INTO Segmentation (id, label_image) VALUES (?, ?) "
        "ON CONFLICT(id) DO UPDATE SET label_image = excluded.label_image;";

    if (sqlite3_prepare_v2(db, query, -1, &stmt, nullptr) != SQLITE_OK) {
      throw std::runtime_error("Failed to prepare insert statement: " +
                               std::string(sqlite3_errmsg(db)));
    }

    sqlite3_bind_int(stmt, 1, nodeId);
    sqlite3_bind_blob(stmt, 2, bytes.data(), static_cast<int>(bytes.size()),
                      SQLITE_STATIC);

    if (sqlite3_step(stmt) != SQLITE_DONE) {
      std::string error = sqlite3_errmsg(db);
      sqlite3_finalize(stmt);
      throw std::runtime_error("Failed to insert labels: " + error);
    }

    sqlite3_finalize(stmt);
    ReUseX::core::trace("Labels saved successfully for node {}", nodeId);
  }

  void saveLabelsBatch(const std::vector<int> &nodeIds,
                       const std::vector<cv::Mat> &labels, bool autoRotate) {
    if (nodeIds.size() != labels.size()) {
      throw std::runtime_error(
          "nodeIds and labels vectors must have same size");
    }

    ReUseX::core::trace("Batch saving labels for {} nodes", nodeIds.size());

    // Begin transaction
    if (sqlite3_exec(db, "BEGIN TRANSACTION;", nullptr, nullptr, nullptr) !=
        SQLITE_OK) {
      throw std::runtime_error("Failed to begin transaction: " +
                               std::string(sqlite3_errmsg(db)));
    }

    try {
      for (size_t i = 0; i < nodeIds.size(); ++i) {
        saveLabels(nodeIds[i], labels[i], autoRotate);
      }

      // Commit transaction
      if (sqlite3_exec(db, "COMMIT;", nullptr, nullptr, nullptr) != SQLITE_OK) {
        throw std::runtime_error("Failed to commit transaction: " +
                                 std::string(sqlite3_errmsg(db)));
      }

      ReUseX::core::trace("Batch save completed successfully");

    } catch (...) {
      // Rollback on any error
      sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, nullptr);
      throw;
    }
  }
};

// --- Public Interface Implementation ---

RTABMapDatabase::RTABMapDatabase(std::filesystem::path dbPath, bool readOnly)
    : impl_(std::make_unique<Impl>(std::move(dbPath), readOnly)) {
  impl_->validateSchema();
}

RTABMapDatabase::~RTABMapDatabase() = default;

RTABMapDatabase::RTABMapDatabase(RTABMapDatabase &&) noexcept = default;
RTABMapDatabase &
RTABMapDatabase::operator=(RTABMapDatabase &&) noexcept = default;

bool RTABMapDatabase::isOpen() const noexcept {
  return impl_ && impl_->db != nullptr;
}

const std::filesystem::path &RTABMapDatabase::getPath() const noexcept {
  return impl_->dbPath;
}

void RTABMapDatabase::validateSchema() const { impl_->validateSchema(); }

std::vector<int> RTABMapDatabase::getNodeIds(bool ignoreChildren) const {
  return impl_->getNodeIds(ignoreChildren);
}

cv::Mat RTABMapDatabase::getImage(int nodeId) const {
  return impl_->getImage(nodeId);
}

void RTABMapDatabase::getGraph(std::map<int, rtabmap::Transform> &poses,
                               std::multimap<int, rtabmap::Link> &links,
                               std::map<int, rtabmap::Signature> *signatures,
                               bool optimized, bool withImages,
                               bool withScan) const {

  ReUseX::core::trace("Retrieving graph from database");

  if (!impl_->rtabmap) {
    throw std::runtime_error("Rtabmap instance not initialized");
  }

  impl_->rtabmap->getGraph(poses, links, optimized, withImages, signatures,
                           withScan);

  ReUseX::core::trace("Graph retrieved: {} poses, {} links", poses.size(),
                      links.size());
}

bool RTABMapDatabase::hasSegmentation(int nodeId) const {
  return impl_->hasSegmentation(nodeId);
}

cv::Mat RTABMapDatabase::getLabels(int nodeId) const {
  return impl_->getLabels(nodeId);
}

void RTABMapDatabase::saveLabels(int nodeId, const cv::Mat &labels,
                                 bool autoRotate) {
  impl_->saveLabels(nodeId, labels, autoRotate);
}

void RTABMapDatabase::saveLabels(const std::vector<int> &nodeIds,
                                 const std::vector<cv::Mat> &labels,
                                 bool autoRotate) {
  impl_->saveLabelsBatch(nodeIds, labels, autoRotate);
}

} // namespace ReUseX::io
