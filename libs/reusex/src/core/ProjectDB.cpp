// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/ProjectDB.hpp"
#include "core/MaterialPassport.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/materialepas_serialization.hpp"
#include "core/materialepas_traits.hpp"
#include "core/logging.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/io/ply_io.h>
#include <sqlite3.h>

#include <cstring>
#include <fstream>
#include <sstream>

namespace ReUseX {

// ── RAII helper for sqlite3_stmt ────────────────────────────────────────
class StmtGuard {
    sqlite3_stmt *stmt_;
  public:
    explicit StmtGuard(sqlite3_stmt *s) : stmt_(s) {}
    ~StmtGuard() { if (stmt_) sqlite3_finalize(stmt_); }
    StmtGuard(const StmtGuard &) = delete;
    StmtGuard &operator=(const StmtGuard &) = delete;
    sqlite3_stmt *get() const { return stmt_; }
};

// ── Compact point serialization helpers ─────────────────────────────────

static constexpr int XYZRGB_STEP = 16; // 3 float + 1 uint32
static constexpr int NORMAL_STEP = 16; // 3 float + curvature
static constexpr int LABEL_STEP  = 4;  // 1 uint32
static constexpr int XYZ_STEP    = 12; // 3 float

static std::vector<uint8_t> serializeXYZRGB(const Cloud &cloud) {
  std::vector<uint8_t> buf(cloud.size() * XYZRGB_STEP);
  auto *dst = buf.data();
  for (const auto &pt : cloud.points) {
    std::memcpy(dst, &pt.x, sizeof(float));
    std::memcpy(dst + 4, &pt.y, sizeof(float));
    std::memcpy(dst + 8, &pt.z, sizeof(float));
    std::memcpy(dst + 12, &pt.rgba, sizeof(uint32_t));
    dst += XYZRGB_STEP;
  }
  return buf;
}

static CloudPtr deserializeXYZRGB(const void *data, size_t size,
                                  uint32_t width, uint32_t height) {
  auto cloud = std::make_shared<Cloud>();
  size_t count = size / XYZRGB_STEP;
  cloud->points.resize(count);
  cloud->width = width;
  cloud->height = height;
  cloud->is_dense = false;
  auto *src = static_cast<const uint8_t *>(data);
  for (size_t i = 0; i < count; ++i) {
    auto &pt = cloud->points[i];
    std::memcpy(&pt.x, src, sizeof(float));
    std::memcpy(&pt.y, src + 4, sizeof(float));
    std::memcpy(&pt.z, src + 8, sizeof(float));
    std::memcpy(&pt.rgba, src + 12, sizeof(uint32_t));
    src += XYZRGB_STEP;
  }
  return cloud;
}

static std::vector<uint8_t> serializeNormal(const CloudN &cloud) {
  std::vector<uint8_t> buf(cloud.size() * NORMAL_STEP);
  auto *dst = buf.data();
  for (const auto &pt : cloud.points) {
    std::memcpy(dst, &pt.normal_x, sizeof(float));
    std::memcpy(dst + 4, &pt.normal_y, sizeof(float));
    std::memcpy(dst + 8, &pt.normal_z, sizeof(float));
    std::memcpy(dst + 12, &pt.curvature, sizeof(float));
    dst += NORMAL_STEP;
  }
  return buf;
}

static CloudNPtr deserializeNormal(const void *data, size_t size,
                                   uint32_t width, uint32_t height) {
  auto cloud = std::make_shared<CloudN>();
  size_t count = size / NORMAL_STEP;
  cloud->points.resize(count);
  cloud->width = width;
  cloud->height = height;
  cloud->is_dense = false;
  auto *src = static_cast<const uint8_t *>(data);
  for (size_t i = 0; i < count; ++i) {
    auto &pt = cloud->points[i];
    std::memcpy(&pt.normal_x, src, sizeof(float));
    std::memcpy(&pt.normal_y, src + 4, sizeof(float));
    std::memcpy(&pt.normal_z, src + 8, sizeof(float));
    std::memcpy(&pt.curvature, src + 12, sizeof(float));
    src += NORMAL_STEP;
  }
  return cloud;
}

static std::vector<uint8_t> serializeLabel(const CloudL &cloud) {
  std::vector<uint8_t> buf(cloud.size() * LABEL_STEP);
  auto *dst = buf.data();
  for (const auto &pt : cloud.points) {
    std::memcpy(dst, &pt.label, sizeof(uint32_t));
    dst += LABEL_STEP;
  }
  return buf;
}

static CloudLPtr deserializeLabel(const void *data, size_t size,
                                  uint32_t width, uint32_t height) {
  auto cloud = std::make_shared<CloudL>();
  size_t count = size / LABEL_STEP;
  cloud->points.resize(count);
  cloud->width = width;
  cloud->height = height;
  cloud->is_dense = false;
  auto *src = static_cast<const uint8_t *>(data);
  for (size_t i = 0; i < count; ++i) {
    std::memcpy(&cloud->points[i].label, src, sizeof(uint32_t));
    src += LABEL_STEP;
  }
  return cloud;
}

static std::vector<uint8_t>
serializeXYZ(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  std::vector<uint8_t> buf(cloud.size() * XYZ_STEP);
  auto *dst = buf.data();
  for (const auto &pt : cloud.points) {
    std::memcpy(dst, &pt.x, sizeof(float));
    std::memcpy(dst + 4, &pt.y, sizeof(float));
    std::memcpy(dst + 8, &pt.z, sizeof(float));
    dst += XYZ_STEP;
  }
  return buf;
}

static pcl::PointCloud<pcl::PointXYZ>::Ptr
deserializeXYZ(const void *data, size_t size,
               uint32_t width, uint32_t height) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  size_t count = size / XYZ_STEP;
  cloud->points.resize(count);
  cloud->width = width;
  cloud->height = height;
  cloud->is_dense = false;
  auto *src = static_cast<const uint8_t *>(data);
  for (size_t i = 0; i < count; ++i) {
    auto &pt = cloud->points[i];
    std::memcpy(&pt.x, src, sizeof(float));
    std::memcpy(&pt.y, src + 4, sizeof(float));
    std::memcpy(&pt.z, src + 8, sizeof(float));
    src += XYZ_STEP;
  }
  return cloud;
}

// ── Impl ────────────────────────────────────────────────────────────────

class ProjectDB::Impl {
    public:
  std::filesystem::path dbPath;
  bool readOnly;
  sqlite3 *db = nullptr;

  static constexpr int LATEST_SCHEMA_VERSION = 3;

  void execOrThrow(const char *sql) {
    char *errMsg = nullptr;
    if (sqlite3_exec(db, sql, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::string error = errMsg ? errMsg : "unknown error";
      sqlite3_free(errMsg);
      throw std::runtime_error(std::string("SQL exec failed: ") + error);
    }
  }

  Impl(std::filesystem::path path, bool ro)
      : dbPath(std::move(path)), readOnly(ro) {
    ReUseX::core::info("Opening ReUseX database: {}", dbPath);

    // Open sqlite3 connection for project database
    int flags = readOnly ? SQLITE_OPEN_READONLY
                         : (SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE);
    if (sqlite3_open_v2(dbPath.string().c_str(), &db, flags, nullptr) !=
        SQLITE_OK) {
      std::string error = sqlite3_errmsg(db);
      sqlite3_close(db);
      throw std::runtime_error("Cannot open database: " + error);
    }

    if (!readOnly) {
      // Performance pragmas for write mode
      sqlite3_exec(db, "PRAGMA journal_mode = WAL;", nullptr, nullptr,
                   nullptr);
      sqlite3_exec(db, "PRAGMA synchronous = NORMAL;", nullptr, nullptr,
                   nullptr);
      sqlite3_exec(db, "PRAGMA foreign_keys = ON;", nullptr, nullptr,
                   nullptr);

      // Capture legacy state BEFORE createTables() adds material_passports
      bool isLegacyDb = !tableExists("schema_version") &&
                         tableExists("material_passports");

      // Create legacy passport tables (always needed)
      createTables();
      // Run schema migrations
      runMigrations(isLegacyDb);
    } else {
      sqlite3_exec(db, "PRAGMA foreign_keys = ON;", nullptr, nullptr,
                   nullptr);
    }

    ReUseX::core::info("Project database opened successfully");
  }

  ~Impl() {
    ReUseX::core::trace("Closing project database connection");
    if (db) {
      sqlite3_close(db);
    }
  }

  // ── Schema versioning ──────────────────────────────────────────────

  bool tableExists(const char *tableName) const {
    const char *query =
        "SELECT name FROM sqlite_master WHERE type='table' AND name=?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, query, -1, &stmt, nullptr) != SQLITE_OK)
      return false;
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, tableName, -1, SQLITE_STATIC);
    return sqlite3_step(stmt) == SQLITE_ROW;
  }

  void createSchemaVersionTable() {
    const char *sql = R"(
      CREATE TABLE IF NOT EXISTS schema_version (
        version     INTEGER NOT NULL,
        applied_at  TEXT NOT NULL DEFAULT (datetime('now')),
        description TEXT
      );
    )";
    char *errMsg = nullptr;
    if (sqlite3_exec(db, sql, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::string error = errMsg;
      sqlite3_free(errMsg);
      throw std::runtime_error(
          "Failed to create schema_version table: " + error);
    }
  }

  int getCurrentSchemaVersion() const {
    if (!tableExists("schema_version"))
      return -1;

    const char *query = "SELECT MAX(version) FROM schema_version;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, query, -1, &stmt, nullptr) != SQLITE_OK)
      return -1;
    StmtGuard guard(stmt);

    if (sqlite3_step(stmt) == SQLITE_ROW &&
        sqlite3_column_type(stmt, 0) != SQLITE_NULL) {
      return sqlite3_column_int(stmt, 0);
    }
    return -1;
  }

  void runMigrations(bool isLegacyDb) {
    createSchemaVersionTable();

    int current = getCurrentSchemaVersion();

    if (current < 0) {
      // Fresh DB or legacy DB — use pre-createTables() state to decide
      if (isLegacyDb) {
        // Legacy DB: mark as version 0, then migrate
        insertSchemaVersion(0, "Legacy passport schema");
        current = 0;
      }
      // else: completely fresh DB, will go straight to v1
    }

    if (current < 1) {
      migrateToV1();
    }

    if (current < 2) {
      migrateToV2();
    }

    if (current < 3) {
      migrateToV3();
    }

    ReUseX::core::trace("Schema version: {}", getCurrentSchemaVersion());
  }

  void insertSchemaVersion(int version, const char *description) {
    const char *sql =
        "INSERT INTO schema_version (version, description) VALUES (?, ?);";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to insert schema version: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, version);
    sqlite3_bind_text(stmt, 2, description, -1, SQLITE_STATIC);
    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to insert schema version row");
  }

  void migrateToV1() {
    ReUseX::core::info("Migrating database to schema version 1");

    const char *v1_schema = R"(
      CREATE TABLE IF NOT EXISTS point_clouds (
        id          INTEGER PRIMARY KEY AUTOINCREMENT,
        name        TEXT NOT NULL UNIQUE,
        point_type  TEXT NOT NULL,
        point_count INTEGER NOT NULL,
        point_step  INTEGER NOT NULL,
        width       INTEGER NOT NULL,
        height      INTEGER NOT NULL DEFAULT 1,
        created_at  TEXT NOT NULL DEFAULT (datetime('now')),
        stage       TEXT,
        parameters  TEXT
      );

      CREATE TABLE IF NOT EXISTS point_cloud_data (
        cloud_id    INTEGER PRIMARY KEY REFERENCES point_clouds(id) ON DELETE CASCADE,
        data        BLOB NOT NULL
      );

      CREATE TABLE IF NOT EXISTS label_definitions (
        cloud_id    INTEGER NOT NULL REFERENCES point_clouds(id) ON DELETE CASCADE,
        label_id    INTEGER NOT NULL,
        name        TEXT NOT NULL,
        color_rgb   INTEGER,
        PRIMARY KEY (cloud_id, label_id)
      );

      CREATE TABLE IF NOT EXISTS meshes (
        id          INTEGER PRIMARY KEY AUTOINCREMENT,
        name        TEXT NOT NULL UNIQUE,
        format      TEXT NOT NULL,
        data        BLOB NOT NULL,
        vertex_count INTEGER,
        face_count   INTEGER,
        created_at  TEXT NOT NULL DEFAULT (datetime('now')),
        stage       TEXT,
        parameters  TEXT
      );

      CREATE TABLE IF NOT EXISTS sensor_frames (
        node_id     INTEGER PRIMARY KEY,
        color       BLOB,
        depth       BLOB,
        confidence  BLOB,
        transform   BLOB,
        width       INTEGER,
        height      INTEGER
      );

      CREATE TABLE IF NOT EXISTS pipeline_log (
        id          INTEGER PRIMARY KEY AUTOINCREMENT,
        stage       TEXT NOT NULL,
        started_at  TEXT NOT NULL DEFAULT (datetime('now')),
        finished_at TEXT,
        parameters  TEXT,
        status      TEXT NOT NULL DEFAULT 'running',
        error_msg   TEXT
      );
    )";

    char *errMsg = nullptr;
    if (sqlite3_exec(db, v1_schema, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::string error = errMsg;
      sqlite3_free(errMsg);
      throw std::runtime_error("Migration to v1 failed: " + error);
    }

    insertSchemaVersion(1, "Add point clouds, meshes, sensor frames, "
                           "pipeline log");
    ReUseX::core::info("Migration to schema version 1 complete");
  }

  void migrateToV2() {
    ReUseX::core::info("Migrating database to schema version 2");

    const char *v2_schema = R"(
      CREATE TABLE IF NOT EXISTS segmentation_images (
        node_id     INTEGER PRIMARY KEY REFERENCES sensor_frames(node_id) ON DELETE CASCADE,
        label_image BLOB NOT NULL
      );
    )";

    char *errMsg = nullptr;
    if (sqlite3_exec(db, v2_schema, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::string error = errMsg;
      sqlite3_free(errMsg);
      throw std::runtime_error("Migration to v2 failed: " + error);
    }

    insertSchemaVersion(2, "Add segmentation_images table");
    ReUseX::core::info("Migration to schema version 2 complete");
  }

  void migrateToV3() {
    ReUseX::core::info("Migrating database to schema version 3");

    // sensor_frames already has depth, confidence, transform columns from v1.
    // Only need to add camera_model TEXT column.
    const char *alter = "ALTER TABLE sensor_frames ADD COLUMN camera_model TEXT;";

    char *errMsg = nullptr;
    if (sqlite3_exec(db, alter, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::string error = errMsg;
      sqlite3_free(errMsg);
      // Column may already exist if DB was created fresh (v1 schema doesn't
      // include it, but a manual migration might have added it).  Ignore
      // "duplicate column" errors.
      if (error.find("duplicate column") == std::string::npos) {
        throw std::runtime_error("Migration to v3 failed: " + error);
      }
    }

    insertSchemaVersion(3, "Add camera_model column to sensor_frames");
    ReUseX::core::info("Migration to schema version 3 complete");
  }

  // ── Sensor frame CRUD ─────────────────────────────────────────────

  void saveSensorFrame(int nodeId, const cv::Mat &colorImage) {
    if (colorImage.empty()) {
      throw std::runtime_error("Cannot save empty color image");
    }

    std::vector<unsigned char> jpegBytes;
    if (!cv::imencode(".jpg", colorImage, jpegBytes,
                      {cv::IMWRITE_JPEG_QUALITY, 95})) {
      throw std::runtime_error("Failed to encode color image as JPEG");
    }

    const char *upsert = R"(
      INSERT INTO sensor_frames (node_id, color, width, height)
      VALUES (?, ?, ?, ?)
      ON CONFLICT(node_id) DO UPDATE SET
        color = excluded.color,
        width = excluded.width,
        height = excluded.height;
    )";

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, upsert, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare sensor frame upsert: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);

    sqlite3_bind_int(stmt, 1, nodeId);
    sqlite3_bind_blob(stmt, 2, jpegBytes.data(),
                      static_cast<int>(jpegBytes.size()), SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 3, colorImage.cols);
    sqlite3_bind_int(stmt, 4, colorImage.rows);

    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to upsert sensor frame: " +
                               std::string(sqlite3_errmsg(db)));
  }

  std::vector<int> getSensorFrameIds() const {
    const char *sql =
        "SELECT node_id FROM sensor_frames ORDER BY node_id;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to query sensor frame IDs: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);

    std::vector<int> ids;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      ids.push_back(sqlite3_column_int(stmt, 0));
    }
    return ids;
  }

  cv::Mat getSensorFrameImage(int nodeId) const {
    const char *sql =
        "SELECT color FROM sensor_frames WHERE node_id = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare sensor frame query: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, nodeId);

    if (sqlite3_step(stmt) != SQLITE_ROW)
      return cv::Mat(); // Not found

    const void *blob = sqlite3_column_blob(stmt, 0);
    int blobSize = sqlite3_column_bytes(stmt, 0);

    if (!blob || blobSize == 0)
      return cv::Mat();

    cv::Mat encoded(1, blobSize, CV_8UC1, const_cast<void *>(blob));
    return cv::imdecode(encoded, cv::IMREAD_UNCHANGED);
  }

  void saveSensorFrameFull(int nodeId, const cv::Mat &color,
                           const cv::Mat &depth, const cv::Mat &confidence,
                           const std::array<double, 16> &worldPose,
                           const ReUseX::core::SensorIntrinsics &intrinsics) {
    if (color.empty())
      throw std::runtime_error("Cannot save empty color image");

    // Encode color as JPEG
    std::vector<unsigned char> jpegBytes;
    if (!cv::imencode(".jpg", color, jpegBytes,
                      {cv::IMWRITE_JPEG_QUALITY, 95}))
      throw std::runtime_error("Failed to encode color image as JPEG");

    // Encode depth as PNG (CV_16UC1 millimeters)
    std::vector<unsigned char> depthBytes;
    if (!depth.empty()) {
      cv::Mat depth16;
      if (depth.type() == CV_32FC1) {
        // Convert meters to millimeters
        cv::Mat mm;
        depth.convertTo(mm, CV_16UC1, 1000.0);
        depth16 = mm;
      } else if (depth.type() == CV_16UC1) {
        depth16 = depth;
      } else {
        depth.convertTo(depth16, CV_16UC1);
      }
      if (!cv::imencode(".png", depth16, depthBytes))
        throw std::runtime_error("Failed to encode depth as PNG");
    }

    // Encode confidence as PNG (CV_8UC1)
    std::vector<unsigned char> confBytes;
    if (!confidence.empty()) {
      cv::Mat conf8;
      if (confidence.type() != CV_8UC1)
        confidence.convertTo(conf8, CV_8UC1);
      else
        conf8 = confidence;
      if (!cv::imencode(".png", conf8, confBytes))
        throw std::runtime_error("Failed to encode confidence as PNG");
    }

    // Camera model as JSON TEXT
    std::string cameraJson = intrinsics.to_json();

    const char *upsert = R"(
      INSERT INTO sensor_frames (node_id, color, depth, confidence, transform, width, height, camera_model)
      VALUES (?, ?, ?, ?, ?, ?, ?, ?)
      ON CONFLICT(node_id) DO UPDATE SET
        color = excluded.color,
        depth = excluded.depth,
        confidence = excluded.confidence,
        transform = excluded.transform,
        width = excluded.width,
        height = excluded.height,
        camera_model = excluded.camera_model;
    )";

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, upsert, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare sensor frame upsert: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);

    sqlite3_bind_int(stmt, 1, nodeId);
    sqlite3_bind_blob(stmt, 2, jpegBytes.data(),
                      static_cast<int>(jpegBytes.size()), SQLITE_TRANSIENT);
    if (!depthBytes.empty())
      sqlite3_bind_blob(stmt, 3, depthBytes.data(),
                        static_cast<int>(depthBytes.size()), SQLITE_TRANSIENT);
    else
      sqlite3_bind_null(stmt, 3);

    if (!confBytes.empty())
      sqlite3_bind_blob(stmt, 4, confBytes.data(),
                        static_cast<int>(confBytes.size()), SQLITE_TRANSIENT);
    else
      sqlite3_bind_null(stmt, 4);

    // Transform: 16 x float64 = 128 bytes, row-major
    sqlite3_bind_blob(stmt, 5, worldPose.data(),
                      static_cast<int>(worldPose.size() * sizeof(double)),
                      SQLITE_TRANSIENT);

    sqlite3_bind_int(stmt, 6, color.cols);
    sqlite3_bind_int(stmt, 7, color.rows);
    sqlite3_bind_text(stmt, 8, cameraJson.c_str(), -1, SQLITE_TRANSIENT);

    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to upsert sensor frame: " +
                               std::string(sqlite3_errmsg(db)));
  }

  cv::Mat getSensorFrameDepth(int nodeId) const {
    const char *sql =
        "SELECT depth FROM sensor_frames WHERE node_id = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare depth query: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, nodeId);

    if (sqlite3_step(stmt) != SQLITE_ROW)
      return cv::Mat();

    const void *blob = sqlite3_column_blob(stmt, 0);
    int blobSize = sqlite3_column_bytes(stmt, 0);
    if (!blob || blobSize == 0)
      return cv::Mat();

    cv::Mat encoded(1, blobSize, CV_8UC1, const_cast<void *>(blob));
    return cv::imdecode(encoded, cv::IMREAD_UNCHANGED); // CV_16UC1
  }

  cv::Mat getSensorFrameConfidence(int nodeId) const {
    const char *sql =
        "SELECT confidence FROM sensor_frames WHERE node_id = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare confidence query: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, nodeId);

    if (sqlite3_step(stmt) != SQLITE_ROW)
      return cv::Mat();

    const void *blob = sqlite3_column_blob(stmt, 0);
    int blobSize = sqlite3_column_bytes(stmt, 0);
    if (!blob || blobSize == 0)
      return cv::Mat();

    cv::Mat encoded(1, blobSize, CV_8UC1, const_cast<void *>(blob));
    return cv::imdecode(encoded, cv::IMREAD_UNCHANGED); // CV_8UC1
  }

  std::array<double, 16> getSensorFramePose(int nodeId) const {
    const char *sql =
        "SELECT transform FROM sensor_frames WHERE node_id = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare pose query: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, nodeId);

    std::array<double, 16> pose = {1, 0, 0, 0, 0, 1, 0, 0,
                                   0, 0, 1, 0, 0, 0, 0, 1};
    if (sqlite3_step(stmt) != SQLITE_ROW)
      return pose;

    const void *blob = sqlite3_column_blob(stmt, 0);
    int blobSize = sqlite3_column_bytes(stmt, 0);
    if (blob && blobSize == static_cast<int>(16 * sizeof(double)))
      std::memcpy(pose.data(), blob, 16 * sizeof(double));

    return pose;
  }

  ReUseX::core::SensorIntrinsics getSensorFrameIntrinsics(int nodeId) const {
    const char *sql =
        "SELECT camera_model FROM sensor_frames WHERE node_id = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare intrinsics query: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, nodeId);

    if (sqlite3_step(stmt) != SQLITE_ROW)
      return {};

    const char *text = reinterpret_cast<const char *>(
        sqlite3_column_text(stmt, 0));
    if (!text)
      return {};

    return ReUseX::core::SensorIntrinsics::from_json(text);
  }

  bool hasSensorFrame(int nodeId) const {
    const char *sql =
        "SELECT 1 FROM sensor_frames WHERE node_id = ? LIMIT 1;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      return false;
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, nodeId);
    return sqlite3_step(stmt) == SQLITE_ROW;
  }

  // ── Segmentation image CRUD ───────────────────────────────────────

  bool hasSegmentationImage(int nodeId) const {
    const char *sql =
        "SELECT 1 FROM segmentation_images WHERE node_id = ? LIMIT 1;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      return false;
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, nodeId);
    return sqlite3_step(stmt) == SQLITE_ROW;
  }

  cv::Mat getSegmentationImage(int nodeId) const {
    const char *sql =
        "SELECT label_image FROM segmentation_images WHERE node_id = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare segmentation query: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, nodeId);

    if (sqlite3_step(stmt) != SQLITE_ROW)
      return cv::Mat(); // Not found

    const void *blob = sqlite3_column_blob(stmt, 0);
    int blobSize = sqlite3_column_bytes(stmt, 0);

    cv::Mat encoded(1, blobSize, CV_8UC1, const_cast<void *>(blob));
    cv::Mat labels16U = cv::imdecode(encoded, cv::IMREAD_UNCHANGED);

    if (labels16U.empty())
      return cv::Mat();

    // Convert CV_16U to CV_32S and apply -1 offset (0 -> -1 for background)
    cv::Mat labels;
    labels16U.convertTo(labels, CV_32S);
    labels -= 1;
    return labels;
  }

  void saveSegmentationImage(int nodeId, const cv::Mat &labels) {
    if (labels.empty()) {
      throw std::runtime_error("Cannot save empty segmentation labels");
    }

    // Apply +1 offset and convert to CV_16U for storage (0 = background)
    cv::Mat toSave;
    cv::Mat offset = labels + 1;
    offset.convertTo(toSave, CV_16U);

    std::vector<unsigned char> pngBytes;
    if (!cv::imencode(".png", toSave, pngBytes)) {
      throw std::runtime_error("Failed to encode segmentation labels as PNG");
    }

    const char *upsert = R"(
      INSERT INTO segmentation_images (node_id, label_image)
      VALUES (?, ?)
      ON CONFLICT(node_id) DO UPDATE SET label_image = excluded.label_image;
    )";

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, upsert, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare segmentation upsert: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);

    sqlite3_bind_int(stmt, 1, nodeId);
    sqlite3_bind_blob(stmt, 2, pngBytes.data(),
                      static_cast<int>(pngBytes.size()), SQLITE_TRANSIENT);

    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to upsert segmentation image: " +
                               std::string(sqlite3_errmsg(db)));
  }

  void saveSegmentationImages(const std::vector<int> &nodeIds,
                              const std::vector<cv::Mat> &labels) {
    if (nodeIds.size() != labels.size()) {
      throw std::runtime_error(
          "nodeIds and labels vectors must have same size");
    }

    execOrThrow("BEGIN TRANSACTION;");
    try {
      for (size_t i = 0; i < nodeIds.size(); ++i) {
        saveSegmentationImage(nodeIds[i], labels[i]);
      }
      sqlite3_exec(db, "COMMIT;", nullptr, nullptr, nullptr);
    } catch (...) {
      sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, nullptr);
      throw;
    }
  }

  // ── Legacy passport tables ─────────────────────────────────────────

  void createTables() {
    const char *schema = R"(
      CREATE TABLE IF NOT EXISTS projects (
        id                   TEXT PRIMARY KEY,
        name                 TEXT,
        building_address     TEXT,
        year_of_construction INTEGER,
        survey_date          TEXT,
        survey_organisation  TEXT,
        notes                TEXT
      );

      CREATE TABLE IF NOT EXISTS property_definitions (
        id            TEXT PRIMARY KEY,
        leksikon_guid TEXT UNIQUE,
        name_en       TEXT NOT NULL,
        category      TEXT,
        data_type     TEXT,
        unit          TEXT,
        is_array      INTEGER DEFAULT 0
      );

      CREATE TABLE IF NOT EXISTS material_passports (
        id             TEXT PRIMARY KEY,
        project_id     TEXT REFERENCES projects(id),
        document_guid  TEXT UNIQUE,
        created_at     TEXT,
        revised_at     TEXT,
        version_number TEXT,
        version_date   TEXT
      );

      CREATE TABLE IF NOT EXISTS passport_property_values (
        id            TEXT PRIMARY KEY,
        passport_id   TEXT REFERENCES material_passports(id),
        property_id   TEXT REFERENCES property_definitions(id),
        leksikon_guid TEXT,
        sort_order    INTEGER,
        value         BLOB,
        UNIQUE(passport_id, leksikon_guid)
      );

      CREATE TABLE IF NOT EXISTS passport_log (
        id          TEXT PRIMARY KEY,
        passport_id TEXT REFERENCES material_passports(id),
        entry_type  TEXT,
        target_guid TEXT,
        edited_by   TEXT,
        edited_at   TEXT,
        old_value   TEXT,
        new_value   TEXT
      );
    )";

    char *errMsg = nullptr;
    if (sqlite3_exec(db, schema, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::string error = errMsg;
      sqlite3_free(errMsg);
      throw std::runtime_error("Failed to create schema: " + error);
    }

    ReUseX::core::trace("Database schema created successfully");
  }

  void validateSchema() const {
    // Check for required project database tables
    const char *tables[] = {"projects", "property_definitions",
                            "material_passports", "passport_property_values",
                            "passport_log"};

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

  // ── Point cloud CRUD ───────────────────────────────────────────────

  void savePointCloudMeta(std::string_view name, const char *pointType,
                          size_t pointCount, int pointStep, uint32_t width,
                          uint32_t height, const std::vector<uint8_t> &data,
                          std::string_view stage,
                          std::string_view paramsJson) {
    execOrThrow("BEGIN TRANSACTION;");
    try {
      // Upsert point_clouds row
      const char *upsert = R"(
        INSERT INTO point_clouds (name, point_type, point_count, point_step, width, height, stage, parameters)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?)
        ON CONFLICT(name) DO UPDATE SET
          point_type = excluded.point_type,
          point_count = excluded.point_count,
          point_step = excluded.point_step,
          width = excluded.width,
          height = excluded.height,
          created_at = datetime('now'),
          stage = excluded.stage,
          parameters = excluded.parameters;
      )";

      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(db, upsert, -1, &stmt, nullptr) != SQLITE_OK)
        throw std::runtime_error("Failed to prepare point cloud upsert: " +
                                 std::string(sqlite3_errmsg(db)));
      StmtGuard guard(stmt);

      sqlite3_bind_text(stmt, 1, name.data(),
                        static_cast<int>(name.size()), SQLITE_TRANSIENT);
      sqlite3_bind_text(stmt, 2, pointType, -1, SQLITE_STATIC);
      sqlite3_bind_int64(stmt, 3, static_cast<sqlite3_int64>(pointCount));
      sqlite3_bind_int(stmt, 4, pointStep);
      sqlite3_bind_int(stmt, 5, static_cast<int>(width));
      sqlite3_bind_int(stmt, 6, static_cast<int>(height));
      if (!stage.empty())
        sqlite3_bind_text(stmt, 7, stage.data(),
                          static_cast<int>(stage.size()), SQLITE_TRANSIENT);
      else
        sqlite3_bind_null(stmt, 7);
      if (!paramsJson.empty())
        sqlite3_bind_text(stmt, 8, paramsJson.data(),
                          static_cast<int>(paramsJson.size()),
                          SQLITE_TRANSIENT);
      else
        sqlite3_bind_null(stmt, 8);

      if (sqlite3_step(stmt) != SQLITE_DONE)
        throw std::runtime_error("Failed to upsert point cloud metadata: " +
                                 std::string(sqlite3_errmsg(db)));

      // Get the cloud_id (might be new or existing)
      int cloudId = getCloudId(name);

      // Upsert point_cloud_data
      const char *dataUpsert = R"(
        INSERT INTO point_cloud_data (cloud_id, data)
        VALUES (?, ?)
        ON CONFLICT(cloud_id) DO UPDATE SET data = excluded.data;
      )";

      sqlite3_stmt *dstmt;
      if (sqlite3_prepare_v2(db, dataUpsert, -1, &dstmt, nullptr) != SQLITE_OK)
        throw std::runtime_error("Failed to prepare data upsert: " +
                                 std::string(sqlite3_errmsg(db)));
      StmtGuard dguard(dstmt);

      sqlite3_bind_int(dstmt, 1, cloudId);
      sqlite3_bind_blob(dstmt, 2, data.data(),
                        static_cast<int>(data.size()), SQLITE_TRANSIENT);
      if (sqlite3_step(dstmt) != SQLITE_DONE)
        throw std::runtime_error("Failed to upsert point cloud data: " +
                                 std::string(sqlite3_errmsg(db)));

      sqlite3_exec(db, "COMMIT;", nullptr, nullptr, nullptr);
    } catch (...) {
      sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, nullptr);
      throw;
    }
  }

  int getCloudId(std::string_view name) const {
    const char *sql = "SELECT id FROM point_clouds WHERE name = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to query cloud ID");
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, name.data(),
                      static_cast<int>(name.size()), SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) != SQLITE_ROW)
      throw std::runtime_error("Point cloud not found: " +
                               std::string(name));
    return sqlite3_column_int(stmt, 0);
  }

  // Generic point cloud loader: returns raw data, point_type, width, height
  struct CloudMeta {
    std::string point_type;
    uint32_t width;
    uint32_t height;
    std::vector<uint8_t> data;
  };

  CloudMeta loadCloudRaw(std::string_view name) const {
    const char *sql = R"(
      SELECT pc.point_type, pc.width, pc.height, pcd.data
      FROM point_clouds pc
      JOIN point_cloud_data pcd ON pcd.cloud_id = pc.id
      WHERE pc.name = ?;
    )";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare cloud load: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, name.data(),
                      static_cast<int>(name.size()), SQLITE_TRANSIENT);

    if (sqlite3_step(stmt) != SQLITE_ROW)
      throw std::runtime_error("Point cloud not found: " +
                               std::string(name));

    CloudMeta meta;
    meta.point_type = reinterpret_cast<const char *>(
        sqlite3_column_text(stmt, 0));
    meta.width = static_cast<uint32_t>(sqlite3_column_int(stmt, 1));
    meta.height = static_cast<uint32_t>(sqlite3_column_int(stmt, 2));

    const void *blob = sqlite3_column_blob(stmt, 3);
    int blobSize = sqlite3_column_bytes(stmt, 3);
    if (!blob || blobSize <= 0)
      throw std::runtime_error("Point cloud data is NULL or empty for: " +
                               std::string(name));
    meta.data.assign(static_cast<const uint8_t *>(blob),
                     static_cast<const uint8_t *>(blob) + blobSize);
    return meta;
  }

  bool hasPointCloud(std::string_view name) const {
    const char *sql =
        "SELECT COUNT(*) FROM point_clouds WHERE name = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      return false;
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, name.data(),
                      static_cast<int>(name.size()), SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) == SQLITE_ROW)
      return sqlite3_column_int(stmt, 0) > 0;
    return false;
  }

  void deletePointCloud(std::string_view name) {
    // CASCADE will remove point_cloud_data and label_definitions rows
    const char *sql = "DELETE FROM point_clouds WHERE name = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare delete: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, name.data(),
                      static_cast<int>(name.size()), SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to delete point cloud: " +
                               std::string(sqlite3_errmsg(db)));
  }

  std::vector<std::string> listPointClouds() const {
    const char *sql = "SELECT name FROM point_clouds ORDER BY id;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to list point clouds");
    StmtGuard guard(stmt);

    std::vector<std::string> names;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      names.emplace_back(
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0)));
    }
    return names;
  }

  std::string getPointCloudType(std::string_view name) const {
    const char *sql = "SELECT point_type FROM point_clouds WHERE name = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to query point cloud type");
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, name.data(),
                      static_cast<int>(name.size()), SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) != SQLITE_ROW)
      throw std::runtime_error("Point cloud not found: " +
                               std::string(name));
    return reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
  }

  // ── Label definitions ──────────────────────────────────────────────

  void saveLabelDefinitions(std::string_view cloudName,
                            const std::map<int, std::string> &labelMap) {
    int cloudId = getCloudId(cloudName);

    execOrThrow("BEGIN TRANSACTION;");
    try {
      // Delete existing definitions for this cloud
      {
        const char *del =
            "DELETE FROM label_definitions WHERE cloud_id = ?;";
        sqlite3_stmt *stmt;
        if (sqlite3_prepare_v2(db, del, -1, &stmt, nullptr) != SQLITE_OK)
          throw std::runtime_error("Failed to prepare label delete");
        StmtGuard guard(stmt);
        sqlite3_bind_int(stmt, 1, cloudId);
        sqlite3_step(stmt);
      }

      // Insert new definitions
      const char *ins = R"(
        INSERT INTO label_definitions (cloud_id, label_id, name)
        VALUES (?, ?, ?);
      )";
      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(db, ins, -1, &stmt, nullptr) != SQLITE_OK)
        throw std::runtime_error("Failed to prepare label insert");
      StmtGuard guard(stmt);

      for (const auto &[labelId, labelName] : labelMap) {
        sqlite3_bind_int(stmt, 1, cloudId);
        sqlite3_bind_int(stmt, 2, labelId);
        sqlite3_bind_text(stmt, 3, labelName.c_str(), -1, SQLITE_TRANSIENT);
        if (sqlite3_step(stmt) != SQLITE_DONE)
          throw std::runtime_error("Failed to insert label definition");
        sqlite3_reset(stmt);
      }

      sqlite3_exec(db, "COMMIT;", nullptr, nullptr, nullptr);
    } catch (...) {
      sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, nullptr);
      throw;
    }
  }

  std::map<int, std::string>
  getLabelDefinitions(std::string_view cloudName) const {
    int cloudId = getCloudId(cloudName);

    const char *sql =
        "SELECT label_id, name FROM label_definitions WHERE cloud_id = ? "
        "ORDER BY label_id;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to query label definitions");
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, cloudId);

    std::map<int, std::string> result;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      int id = sqlite3_column_int(stmt, 0);
      const char *name = reinterpret_cast<const char *>(
          sqlite3_column_text(stmt, 1));
      result[id] = name;
    }
    return result;
  }

  // ── Mesh CRUD ──────────────────────────────────────────────────────

  void saveMesh(std::string_view name, const pcl::PolygonMesh &mesh,
                std::string_view stage, std::string_view paramsJson) {
    // Serialize mesh to PLY binary via temp file
    auto tmpPath = std::filesystem::temp_directory_path() /
                   ("reusex_mesh_" + std::to_string(reinterpret_cast<uintptr_t>(&mesh)) + ".ply");
    pcl::io::savePLYFileBinary(tmpPath.string(), mesh);

    // Read the file into a buffer
    std::ifstream ifs(tmpPath, std::ios::binary | std::ios::ate);
    if (!ifs)
      throw std::runtime_error("Failed to read temporary PLY file");
    auto fileSize = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    std::vector<char> plyData(static_cast<size_t>(fileSize));
    ifs.read(plyData.data(), fileSize);
    ifs.close();
    std::filesystem::remove(tmpPath);

    // Count vertices and faces
    int vertexCount = 0;
    int faceCount = static_cast<int>(mesh.polygons.size());
    if (mesh.cloud.height > 0)
      vertexCount = static_cast<int>(mesh.cloud.width * mesh.cloud.height);

    const char *upsert = R"(
      INSERT INTO meshes (name, format, data, vertex_count, face_count, stage, parameters)
      VALUES (?, 'ply_binary', ?, ?, ?, ?, ?)
      ON CONFLICT(name) DO UPDATE SET
        format = excluded.format,
        data = excluded.data,
        vertex_count = excluded.vertex_count,
        face_count = excluded.face_count,
        created_at = datetime('now'),
        stage = excluded.stage,
        parameters = excluded.parameters;
    )";

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, upsert, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare mesh upsert: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);

    sqlite3_bind_text(stmt, 1, name.data(),
                      static_cast<int>(name.size()), SQLITE_TRANSIENT);
    sqlite3_bind_blob(stmt, 2, plyData.data(),
                      static_cast<int>(plyData.size()), SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 3, vertexCount);
    sqlite3_bind_int(stmt, 4, faceCount);
    if (!stage.empty())
      sqlite3_bind_text(stmt, 5, stage.data(),
                        static_cast<int>(stage.size()), SQLITE_TRANSIENT);
    else
      sqlite3_bind_null(stmt, 5);
    if (!paramsJson.empty())
      sqlite3_bind_text(stmt, 6, paramsJson.data(),
                        static_cast<int>(paramsJson.size()),
                        SQLITE_TRANSIENT);
    else
      sqlite3_bind_null(stmt, 6);

    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to upsert mesh: " +
                               std::string(sqlite3_errmsg(db)));
  }

  pcl::PolygonMesh::Ptr getMesh(std::string_view name) const {
    const char *sql = "SELECT data FROM meshes WHERE name = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare mesh load");
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, name.data(),
                      static_cast<int>(name.size()), SQLITE_TRANSIENT);

    if (sqlite3_step(stmt) != SQLITE_ROW)
      throw std::runtime_error("Mesh not found: " + std::string(name));

    const void *blob = sqlite3_column_blob(stmt, 0);
    int blobSize = sqlite3_column_bytes(stmt, 0);

    // Write to temp file and load via PCL (use hash to avoid path traversal)
    auto tmpPath = std::filesystem::temp_directory_path() /
                   ("reusex_mesh_load_" +
                    std::to_string(std::hash<std::string>{}(std::string(name))) +
                    ".ply");
    {
      std::ofstream ofs(tmpPath, std::ios::binary);
      ofs.write(static_cast<const char *>(blob), blobSize);
    }

    auto mesh = std::make_shared<pcl::PolygonMesh>();
    if (pcl::io::loadPLYFile(tmpPath.string(), *mesh) < 0) {
      std::filesystem::remove(tmpPath);
      throw std::runtime_error("Failed to parse PLY mesh data");
    }
    std::filesystem::remove(tmpPath);
    return mesh;
  }

  bool hasMesh(std::string_view name) const {
    const char *sql = "SELECT COUNT(*) FROM meshes WHERE name = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      return false;
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, name.data(),
                      static_cast<int>(name.size()), SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) == SQLITE_ROW)
      return sqlite3_column_int(stmt, 0) > 0;
    return false;
  }

  std::vector<std::string> listMeshes() const {
    const char *sql = "SELECT name FROM meshes ORDER BY id;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to list meshes");
    StmtGuard guard(stmt);

    std::vector<std::string> names;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      names.emplace_back(
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0)));
    }
    return names;
  }

  // ── Pipeline log ───────────────────────────────────────────────────

  int logPipelineStart(std::string_view stage, std::string_view paramsJson) {
    const char *sql = R"(
      INSERT INTO pipeline_log (stage, parameters) VALUES (?, ?);
    )";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare pipeline log insert");
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, stage.data(),
                      static_cast<int>(stage.size()), SQLITE_TRANSIENT);
    if (!paramsJson.empty())
      sqlite3_bind_text(stmt, 2, paramsJson.data(),
                        static_cast<int>(paramsJson.size()),
                        SQLITE_TRANSIENT);
    else
      sqlite3_bind_null(stmt, 2);

    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to log pipeline start");

    return static_cast<int>(sqlite3_last_insert_rowid(db));
  }

  void logPipelineEnd(int logId, bool success, std::string_view errorMsg) {
    const char *sql = R"(
      UPDATE pipeline_log
      SET finished_at = datetime('now'), status = ?, error_msg = ?
      WHERE id = ?;
    )";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare pipeline log update");
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, success ? "success" : "failed", -1,
                      SQLITE_STATIC);
    if (!errorMsg.empty())
      sqlite3_bind_text(stmt, 2, errorMsg.data(),
                        static_cast<int>(errorMsg.size()),
                        SQLITE_TRANSIENT);
    else
      sqlite3_bind_null(stmt, 2);
    sqlite3_bind_int(stmt, 3, logId);

    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to log pipeline end");
  }

  // ── Material passport operations (unchanged) ──────────────────────

  size_t getNuberOfMaterials() const {
    const char *query = "SELECT COUNT(*) FROM material_passports;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, query, -1, &stmt, nullptr) != SQLITE_OK) {
      throw std::runtime_error("Failed to count materials: " +
                               std::string(sqlite3_errmsg(db)));
    }

    size_t count = 0;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
      count = static_cast<size_t>(sqlite3_column_int64(stmt, 0));
    }
    sqlite3_finalize(stmt);
    return count;
  }

  class MaterialPassportIterator {
    const Impl &impl;
    std::vector<std::string> document_guids_;
    size_t index = 0;

      public:
    MaterialPassportIterator(const Impl &impl_ref) : impl(impl_ref) {
      // Fetch all document GUIDs (lightweight query)
      const char *query = "SELECT document_guid FROM material_passports ORDER BY created_at;";
      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(impl.db, query, -1, &stmt, nullptr) == SQLITE_OK) {
        while (sqlite3_step(stmt) == SQLITE_ROW) {
          const char *guid = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
          if (guid) {
            document_guids_.emplace_back(guid);
          }
        }
        sqlite3_finalize(stmt);
      }
    }

    bool hasNext() const { return index < document_guids_.size(); }

    ReUseX::core::MaterialPassport next() {
      if (!hasNext()) {
        throw std::out_of_range("No more material passports available");
      }
      return impl.getMaterialPassport(document_guids_[index++]);
    }
  };

  MaterialPassportIterator
  getMaterialPassports() const {
    return MaterialPassportIterator(*this);
  }

  ReUseX::core::MaterialPassportMetadata
  fetchPassportMetadata(std::string_view documentGuid) const {
    const char *query =
        "SELECT document_guid, created_at, revised_at, version_number, version_date "
        "FROM material_passports WHERE document_guid = ?;";

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, query, -1, &stmt, nullptr) != SQLITE_OK) {
      throw std::runtime_error("Failed to prepare metadata query: " +
                               std::string(sqlite3_errmsg(db)));
    }

    sqlite3_bind_text(stmt, 1, documentGuid.data(), documentGuid.size(),
                      SQLITE_TRANSIENT);

    ReUseX::core::MaterialPassportMetadata metadata;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
      metadata.document_guid =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
      metadata.creation_date =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
      metadata.revision_date =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 2));
      metadata.version_number =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 3));
      metadata.version_date =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 4));
    } else {
      sqlite3_finalize(stmt);
      throw std::runtime_error("Material passport not found: " +
                               std::string(documentGuid));
    }

    sqlite3_finalize(stmt);
    return metadata;
  }

  std::map<std::string, ReUseX::core::serialization::PropertyValue>
  fetchPropertyValues(std::string_view documentGuid) const {
    const char *query =
        "SELECT pd.leksikon_guid, pd.data_type, ppv.value "
        "FROM material_passports mp "
        "JOIN passport_property_values ppv ON ppv.passport_id = mp.id "
        "JOIN property_definitions pd ON pd.id = ppv.property_id "
        "WHERE mp.document_guid = ? "
        "ORDER BY ppv.sort_order;";

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, query, -1, &stmt, nullptr) != SQLITE_OK) {
      throw std::runtime_error("Failed to prepare property values query: " +
                               std::string(sqlite3_errmsg(db)));
    }

    sqlite3_bind_text(stmt, 1, documentGuid.data(), documentGuid.size(),
                      SQLITE_TRANSIENT);

    std::map<std::string, ReUseX::core::serialization::PropertyValue>
        property_map;

    while (sqlite3_step(stmt) == SQLITE_ROW) {
      const char *guid =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
      const char *data_type =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
      const void *blob_data = sqlite3_column_blob(stmt, 2);
      int blob_size = sqlite3_column_bytes(stmt, 2);

      if (guid && data_type) {
        auto type =
            ReUseX::core::traits::property_type_from_string(data_type);
        if (type) {
          property_map.emplace(
              guid, ReUseX::core::serialization::PropertyValue(
                        blob_data, blob_size, *type));
        }
      }
    }

    sqlite3_finalize(stmt);
    return property_map;
  }

  std::vector<ReUseX::core::TransactionLogEntry>
  fetchTransactionLog(std::string_view documentGuid) const {
    const char *query =
        "SELECT entry_type, target_guid, edited_by, edited_at, old_value, new_value "
        "FROM passport_log pl "
        "JOIN material_passports mp ON pl.passport_id = mp.id "
        "WHERE mp.document_guid = ? "
        "ORDER BY pl.edited_at;";

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, query, -1, &stmt, nullptr) != SQLITE_OK) {
      throw std::runtime_error("Failed to prepare transaction log query: " +
                               std::string(sqlite3_errmsg(db)));
    }

    sqlite3_bind_text(stmt, 1, documentGuid.data(), documentGuid.size(),
                      SQLITE_TRANSIENT);

    std::vector<ReUseX::core::TransactionLogEntry> log;

    while (sqlite3_step(stmt) == SQLITE_ROW) {
      ReUseX::core::TransactionLogEntry entry;

      const char *entry_type =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
      if (auto type =
              ReUseX::core::transaction_type_from_string(entry_type)) {
        entry.type = *type;
      }

      entry.guid =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
      entry.edited_by =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 2));
      entry.edited_date =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 3));
      entry.old_value =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 4));
      entry.new_value =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 5));

      log.push_back(std::move(entry));
    }

    sqlite3_finalize(stmt);
    return log;
  }

  ReUseX::core::MaterialPassport
  getMaterialPassport(std::string_view documentGuid) const {
    ReUseX::core::info("Fetching MaterialPassport: {}", documentGuid);

    // 1. Fetch metadata
    auto metadata = fetchPassportMetadata(documentGuid);

    // 2. Fetch all property values
    auto property_map = fetchPropertyValues(documentGuid);

    // 3. Deserialize into MaterialPassport sections
    ReUseX::core::MaterialPassport passport;
    passport.metadata = std::move(metadata);

    using namespace ReUseX::core::serialization;
    Deserializer::deserialize(passport.owner, property_map);
    Deserializer::deserialize(passport.description, property_map);
    Deserializer::deserialize(passport.product, property_map);
    Deserializer::deserialize(passport.certifications, property_map);
    Deserializer::deserialize(passport.dimensions, property_map);
    Deserializer::deserialize(passport.condition, property_map);
    Deserializer::deserialize(passport.pollution, property_map);
    Deserializer::deserialize(passport.environmental, property_map);
    Deserializer::deserialize(passport.fire, property_map);
    Deserializer::deserialize(passport.history, property_map);

    // 4. Fetch transaction log
    passport.transaction_log = fetchTransactionLog(documentGuid);

    ReUseX::core::trace("MaterialPassport fetched successfully: {} properties",
                        property_map.size());

    return passport;
  }

  void ensurePropertyDefinitions(
      const ReUseX::core::traits::PropertyDescriptor *props, size_t count,
      const char *category, sqlite3_stmt *stmt) {
    for (size_t i = 0; i < count; ++i) {
      const auto &prop = props[i];

      if (prop.type == ReUseX::core::traits::PropertyType::ObjectArray) {
        auto data_type =
            ReUseX::core::traits::to_data_type_string(prop.type);

        sqlite3_bind_text(stmt, 1, prop.field_name, -1, SQLITE_STATIC);
        sqlite3_bind_text(stmt, 2, prop.field_name, -1, SQLITE_STATIC);
        sqlite3_bind_text(stmt, 3, prop.field_name, -1, SQLITE_STATIC);
        sqlite3_bind_text(stmt, 4, category, -1, SQLITE_STATIC);
        sqlite3_bind_text(stmt, 5, data_type.data(),
                          static_cast<int>(data_type.size()), SQLITE_STATIC);

        if (sqlite3_step(stmt) != SQLITE_DONE) {
          std::string error = sqlite3_errmsg(db);
          sqlite3_finalize(stmt);
          throw std::runtime_error(
              "Failed to insert property definition: " + error);
        }
        sqlite3_reset(stmt);
        continue;
      }

      auto data_type =
          ReUseX::core::traits::to_data_type_string(prop.type);

      sqlite3_bind_text(stmt, 1, prop.leksikon_guid, -1, SQLITE_STATIC);
      sqlite3_bind_text(stmt, 2, prop.leksikon_guid, -1, SQLITE_STATIC);
      sqlite3_bind_text(stmt, 3, prop.field_name, -1, SQLITE_STATIC);
      sqlite3_bind_text(stmt, 4, category, -1, SQLITE_STATIC);
      sqlite3_bind_text(stmt, 5, data_type.data(),
                        static_cast<int>(data_type.size()), SQLITE_STATIC);

      if (sqlite3_step(stmt) != SQLITE_DONE) {
        std::string error = sqlite3_errmsg(db);
        sqlite3_finalize(stmt);
        throw std::runtime_error(
            "Failed to insert property definition: " + error);
      }
      sqlite3_reset(stmt);
    }
  }

  void ensureAllPropertyDefinitions() {
    const char *query =
        "INSERT OR IGNORE INTO property_definitions "
        "(id, leksikon_guid, name_en, category, data_type) "
        "VALUES (?, ?, ?, ?, ?);";

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, query, -1, &stmt, nullptr) != SQLITE_OK) {
      throw std::runtime_error(
          "Failed to prepare property definition insert: " +
          std::string(sqlite3_errmsg(db)));
    }

    using namespace ReUseX::core::traits;

    ensurePropertyDefinitions(
        PropertyTraits<ReUseX::core::Owner>::properties(),
        PropertyTraits<ReUseX::core::Owner>::property_count(), "Owner", stmt);
    ensurePropertyDefinitions(
        PropertyTraits<ReUseX::core::ConstructionItemDescription>::properties(),
        PropertyTraits<ReUseX::core::ConstructionItemDescription>::
            property_count(),
        "Description", stmt);
    ensurePropertyDefinitions(
        PropertyTraits<ReUseX::core::ProductInformation>::properties(),
        PropertyTraits<ReUseX::core::ProductInformation>::property_count(),
        "Product", stmt);
    ensurePropertyDefinitions(
        PropertyTraits<ReUseX::core::Certifications>::properties(),
        PropertyTraits<ReUseX::core::Certifications>::property_count(),
        "Certifications", stmt);
    ensurePropertyDefinitions(
        PropertyTraits<ReUseX::core::Dimensions>::properties(),
        PropertyTraits<ReUseX::core::Dimensions>::property_count(),
        "Dimensions", stmt);
    ensurePropertyDefinitions(
        PropertyTraits<ReUseX::core::Condition>::properties(),
        PropertyTraits<ReUseX::core::Condition>::property_count(), "Condition",
        stmt);
    ensurePropertyDefinitions(
        PropertyTraits<ReUseX::core::Pollution>::properties(),
        PropertyTraits<ReUseX::core::Pollution>::property_count(), "Pollution",
        stmt);
    ensurePropertyDefinitions(
        PropertyTraits<ReUseX::core::EnvironmentalPotential>::properties(),
        PropertyTraits<ReUseX::core::EnvironmentalPotential>::property_count(),
        "Environmental", stmt);
    ensurePropertyDefinitions(
        PropertyTraits<ReUseX::core::FireProperties>::properties(),
        PropertyTraits<ReUseX::core::FireProperties>::property_count(), "Fire",
        stmt);
    ensurePropertyDefinitions(
        PropertyTraits<ReUseX::core::History>::properties(),
        PropertyTraits<ReUseX::core::History>::property_count(), "History",
        stmt);

    sqlite3_finalize(stmt);
  }

  void addMaterialPassport(const ReUseX::core::MaterialPassport &passport,
                           std::string_view projectId) {
    ReUseX::core::info("Adding MaterialPassport: {}",
                       passport.metadata.document_guid);

    // Begin transaction for atomicity
    execOrThrow("BEGIN TRANSACTION;");

    try {
      // 0. Ensure property_definitions are populated
      ensureAllPropertyDefinitions();

      // 0b. Ensure project row exists
      {
        const char *upsert_project =
            "INSERT OR IGNORE INTO projects (id) VALUES (?);";
        sqlite3_stmt *proj_stmt;
        if (sqlite3_prepare_v2(db, upsert_project, -1, &proj_stmt, nullptr) !=
            SQLITE_OK) {
          throw std::runtime_error(
              "Failed to prepare project insert: " +
              std::string(sqlite3_errmsg(db)));
        }
        sqlite3_bind_text(proj_stmt, 1, projectId.data(), projectId.size(),
                          SQLITE_TRANSIENT);
        if (sqlite3_step(proj_stmt) != SQLITE_DONE) {
          std::string error = sqlite3_errmsg(db);
          sqlite3_finalize(proj_stmt);
          throw std::runtime_error("Failed to insert project: " + error);
        }
        sqlite3_finalize(proj_stmt);
      }

      // 1. Insert material_passports row
      const char *insert_passport_query =
          "INSERT INTO material_passports (id, project_id, document_guid, "
          "created_at, revised_at, version_number, version_date) "
          "VALUES (?, ?, ?, ?, ?, ?, ?);";

      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(db, insert_passport_query, -1, &stmt, nullptr) !=
          SQLITE_OK) {
        throw std::runtime_error("Failed to prepare insert passport statement: " +
                                 std::string(sqlite3_errmsg(db)));
      }

      std::string passport_id = passport.metadata.document_guid;
      sqlite3_bind_text(stmt, 1, passport_id.c_str(), -1, SQLITE_TRANSIENT);
      sqlite3_bind_text(stmt, 2, projectId.data(), projectId.size(), SQLITE_TRANSIENT);
      sqlite3_bind_text(stmt, 3, passport.metadata.document_guid.c_str(), -1,
                        SQLITE_TRANSIENT);
      sqlite3_bind_text(stmt, 4, passport.metadata.creation_date.c_str(), -1,
                        SQLITE_TRANSIENT);
      sqlite3_bind_text(stmt, 5, passport.metadata.revision_date.c_str(), -1,
                        SQLITE_TRANSIENT);
      sqlite3_bind_text(stmt, 6, passport.metadata.version_number.c_str(), -1,
                        SQLITE_TRANSIENT);
      sqlite3_bind_text(stmt, 7, passport.metadata.version_date.c_str(), -1,
                        SQLITE_TRANSIENT);

      if (sqlite3_step(stmt) != SQLITE_DONE) {
        std::string error = sqlite3_errmsg(db);
        sqlite3_finalize(stmt);
        throw std::runtime_error("Failed to insert passport: " + error);
      }
      sqlite3_finalize(stmt);

      // 2. Serialize all sections to property values
      using namespace ReUseX::core::serialization;
      std::map<std::string, PropertyValue> all_values;

      auto merge_values = [&all_values](auto &&values) {
        all_values.insert(values.begin(), values.end());
      };

      merge_values(Serializer::serialize(passport.owner));
      merge_values(Serializer::serialize(passport.description));
      merge_values(Serializer::serialize(passport.product));
      merge_values(Serializer::serialize(passport.certifications));
      merge_values(Serializer::serialize(passport.dimensions));
      merge_values(Serializer::serialize(passport.condition));
      merge_values(Serializer::serialize(passport.pollution));
      merge_values(Serializer::serialize(passport.environmental));
      merge_values(Serializer::serialize(passport.fire));
      merge_values(Serializer::serialize(passport.history));

      // 3. Insert passport_property_values rows
      const char *insert_value_query =
          "INSERT INTO passport_property_values (id, passport_id, property_id, "
          "leksikon_guid, sort_order, value) VALUES (?, ?, ?, ?, ?, ?);";

      if (sqlite3_prepare_v2(db, insert_value_query, -1, &stmt, nullptr) !=
          SQLITE_OK) {
        throw std::runtime_error("Failed to prepare insert value statement: " +
                                 std::string(sqlite3_errmsg(db)));
      }

      int sort_order = 0;
      for (const auto &[guid, value] : all_values) {
        std::string value_id = passport_id + "_" + guid;

        sqlite3_bind_text(stmt, 1, value_id.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 2, passport_id.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 3, guid.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 4, guid.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_int(stmt, 5, sort_order++);
        sqlite3_bind_blob(stmt, 6, value.blob().data(), value.blob().size(),
                          SQLITE_TRANSIENT);

        if (sqlite3_step(stmt) != SQLITE_DONE) {
          std::string error = sqlite3_errmsg(db);
          sqlite3_finalize(stmt);
          throw std::runtime_error("Failed to insert property value: " + error);
        }

        sqlite3_reset(stmt);
      }
      sqlite3_finalize(stmt);

      // 4. Insert transaction log entries
      const char *insert_log_query =
          "INSERT INTO passport_log (id, passport_id, entry_type, target_guid, "
          "edited_by, edited_at, old_value, new_value) VALUES (?, ?, ?, ?, ?, ?, ?, ?);";

      if (sqlite3_prepare_v2(db, insert_log_query, -1, &stmt, nullptr) !=
          SQLITE_OK) {
        throw std::runtime_error("Failed to prepare insert log statement: " +
                                 std::string(sqlite3_errmsg(db)));
      }

      int log_idx = 0;
      for (const auto &entry : passport.transaction_log) {
        std::string log_id = passport_id + "_log_" + std::to_string(log_idx++);

        sqlite3_bind_text(stmt, 1, log_id.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 2, passport_id.c_str(), -1, SQLITE_TRANSIENT);

        std::string type_str(ReUseX::core::to_string(entry.type));
        sqlite3_bind_text(stmt, 3, type_str.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 4, entry.guid.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 5, entry.edited_by.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 6, entry.edited_date.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 7, entry.old_value.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 8, entry.new_value.c_str(), -1, SQLITE_TRANSIENT);

        if (sqlite3_step(stmt) != SQLITE_DONE) {
          std::string error = sqlite3_errmsg(db);
          sqlite3_finalize(stmt);
          throw std::runtime_error("Failed to insert log entry: " + error);
        }

        sqlite3_reset(stmt);
      }
      sqlite3_finalize(stmt);

      // Commit transaction
      sqlite3_exec(db, "COMMIT;", nullptr, nullptr, nullptr);

      ReUseX::core::info("MaterialPassport added successfully: {} properties",
                         all_values.size());

    } catch (...) {
      sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, nullptr);
      throw;
    }
  }
};

// ── Public Interface Implementation ─────────────────────────────────────

ProjectDB::ProjectDB(std::filesystem::path dbPath, bool readOnly)
    : impl_(std::make_unique<Impl>(std::move(dbPath), readOnly)) {
  impl_->validateSchema();
}

ProjectDB::~ProjectDB() = default;

ProjectDB::ProjectDB(ProjectDB &&) noexcept = default;
ProjectDB &ProjectDB::operator=(ProjectDB &&) noexcept = default;

bool ProjectDB::is_open() const noexcept {
  return impl_ && impl_->db != nullptr;
}

const std::filesystem::path &ProjectDB::path() const noexcept {
  return impl_->dbPath;
}

int ProjectDB::schema_version() const {
  return impl_->getCurrentSchemaVersion();
}

void ProjectDB::validate_schema() const { impl_->validateSchema(); }

// --- Sensor Frame Operations ---

void ProjectDB::save_sensor_frame(int nodeId, const cv::Mat &colorImage) {
  impl_->saveSensorFrame(nodeId, colorImage);
}

void ProjectDB::save_sensor_frame(int nodeId, const cv::Mat &color,
                                   const cv::Mat &depth,
                                   const cv::Mat &confidence,
                                   const std::array<double, 16> &worldPose,
                                   const core::SensorIntrinsics &intrinsics) {
  impl_->saveSensorFrameFull(nodeId, color, depth, confidence, worldPose,
                             intrinsics);
}

std::vector<int> ProjectDB::sensor_frame_ids() const {
  return impl_->getSensorFrameIds();
}

cv::Mat ProjectDB::sensor_frame_image(int nodeId) const {
  return impl_->getSensorFrameImage(nodeId);
}

cv::Mat ProjectDB::sensor_frame_depth(int nodeId) const {
  return impl_->getSensorFrameDepth(nodeId);
}

cv::Mat ProjectDB::sensor_frame_confidence(int nodeId) const {
  return impl_->getSensorFrameConfidence(nodeId);
}

std::array<double, 16> ProjectDB::sensor_frame_pose(int nodeId) const {
  return impl_->getSensorFramePose(nodeId);
}

core::SensorIntrinsics ProjectDB::sensor_frame_intrinsics(int nodeId) const {
  return impl_->getSensorFrameIntrinsics(nodeId);
}

bool ProjectDB::has_sensor_frame(int nodeId) const {
  return impl_->hasSensorFrame(nodeId);
}

// --- Segmentation Image Operations ---

bool ProjectDB::has_segmentation_image(int nodeId) const {
  return impl_->hasSegmentationImage(nodeId);
}

cv::Mat ProjectDB::segmentation_image(int nodeId) const {
  return impl_->getSegmentationImage(nodeId);
}

void ProjectDB::save_segmentation_image(int nodeId, const cv::Mat &labels) {
  impl_->saveSegmentationImage(nodeId, labels);
}

void ProjectDB::save_segmentation_images(const std::vector<int> &nodeIds,
                                         const std::vector<cv::Mat> &labels) {
  impl_->saveSegmentationImages(nodeIds, labels);
}

// --- Point Cloud Operations ---

void ProjectDB::save_point_cloud(std::string_view name, const Cloud &cloud,
                                 std::string_view stage,
                                 std::string_view paramsJson) {
  auto data = serializeXYZRGB(cloud);
  impl_->savePointCloudMeta(name, "PointXYZRGB", cloud.size(), XYZRGB_STEP,
                            cloud.width, cloud.height, data, stage,
                            paramsJson);
}

void ProjectDB::save_point_cloud(std::string_view name, const CloudN &cloud,
                                 std::string_view stage,
                                 std::string_view paramsJson) {
  auto data = serializeNormal(cloud);
  impl_->savePointCloudMeta(name, "Normal", cloud.size(), NORMAL_STEP,
                            cloud.width, cloud.height, data, stage,
                            paramsJson);
}

void ProjectDB::save_point_cloud(std::string_view name, const CloudL &cloud,
                                 std::string_view stage,
                                 std::string_view paramsJson) {
  auto data = serializeLabel(cloud);
  impl_->savePointCloudMeta(name, "Label", cloud.size(), LABEL_STEP,
                            cloud.width, cloud.height, data, stage,
                            paramsJson);
}

void ProjectDB::save_point_cloud(std::string_view name,
                                 const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                 std::string_view stage,
                                 std::string_view paramsJson) {
  auto data = serializeXYZ(cloud);
  impl_->savePointCloudMeta(name, "PointXYZ", cloud.size(), XYZ_STEP,
                            cloud.width, cloud.height, data, stage,
                            paramsJson);
}

CloudPtr ProjectDB::point_cloud_xyzrgb(std::string_view name) const {
  auto meta = impl_->loadCloudRaw(name);
  if (meta.point_type != "PointXYZRGB")
    throw std::runtime_error("Point cloud '" + std::string(name) +
                             "' is type " + meta.point_type +
                             ", expected PointXYZRGB");
  return deserializeXYZRGB(meta.data.data(), meta.data.size(), meta.width,
                           meta.height);
}

CloudNPtr ProjectDB::point_cloud_normal(std::string_view name) const {
  auto meta = impl_->loadCloudRaw(name);
  if (meta.point_type != "Normal")
    throw std::runtime_error("Point cloud '" + std::string(name) +
                             "' is type " + meta.point_type +
                             ", expected Normal");
  return deserializeNormal(meta.data.data(), meta.data.size(), meta.width,
                           meta.height);
}

CloudLPtr ProjectDB::point_cloud_label(std::string_view name) const {
  auto meta = impl_->loadCloudRaw(name);
  if (meta.point_type != "Label")
    throw std::runtime_error("Point cloud '" + std::string(name) +
                             "' is type " + meta.point_type +
                             ", expected Label");
  return deserializeLabel(meta.data.data(), meta.data.size(), meta.width,
                          meta.height);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
ProjectDB::point_cloud_xyz(std::string_view name) const {
  auto meta = impl_->loadCloudRaw(name);
  if (meta.point_type != "PointXYZ")
    throw std::runtime_error("Point cloud '" + std::string(name) +
                             "' is type " + meta.point_type +
                             ", expected PointXYZ");
  return deserializeXYZ(meta.data.data(), meta.data.size(), meta.width,
                        meta.height);
}

bool ProjectDB::has_point_cloud(std::string_view name) const {
  return impl_->hasPointCloud(name);
}

void ProjectDB::delete_point_cloud(std::string_view name) {
  impl_->deletePointCloud(name);
}

std::vector<std::string> ProjectDB::list_point_clouds() const {
  return impl_->listPointClouds();
}

std::string ProjectDB::point_cloud_type(std::string_view name) const {
  return impl_->getPointCloudType(name);
}

// --- Label Definitions ---

void ProjectDB::save_label_definitions(
    std::string_view cloudName,
    const std::map<int, std::string> &labelMap) {
  impl_->saveLabelDefinitions(cloudName, labelMap);
}

std::map<int, std::string>
ProjectDB::label_definitions(std::string_view cloudName) const {
  return impl_->getLabelDefinitions(cloudName);
}

// --- Mesh Operations ---

void ProjectDB::save_mesh(std::string_view name, const pcl::PolygonMesh &mesh,
                           std::string_view stage,
                           std::string_view paramsJson) {
  impl_->saveMesh(name, mesh, stage, paramsJson);
}

pcl::PolygonMesh::Ptr ProjectDB::mesh(std::string_view name) const {
  return impl_->getMesh(name);
}

bool ProjectDB::has_mesh(std::string_view name) const {
  return impl_->hasMesh(name);
}

std::vector<std::string> ProjectDB::list_meshes() const {
  return impl_->listMeshes();
}

// --- Pipeline Log ---

int ProjectDB::log_pipeline_start(std::string_view stage,
                                  std::string_view paramsJson) {
  return impl_->logPipelineStart(stage, paramsJson);
}

void ProjectDB::log_pipeline_end(int logId, bool success,
                                 std::string_view errorMsg) {
  impl_->logPipelineEnd(logId, success, errorMsg);
}

// --- Material Passport Operations ---

ReUseX::core::MaterialPassport
ProjectDB::material_passport(std::string_view documentGuid) const {
  return impl_->getMaterialPassport(documentGuid);
}

std::vector<ReUseX::core::MaterialPassport>
ProjectDB::all_material_passports() const {
  auto iter = impl_->getMaterialPassports();
  std::vector<ReUseX::core::MaterialPassport> result;
  while (iter.hasNext()) {
    result.push_back(iter.next());
  }
  return result;
}

void ProjectDB::add_material_passport(const ReUseX::core::MaterialPassport &passport,
                                      std::string_view projectId) {
  impl_->addMaterialPassport(passport, projectId);
}

} // namespace ReUseX
