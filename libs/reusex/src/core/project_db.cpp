// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/project_db.hpp"
#include "core/MaterialPassport.hpp"
#include "core/materialepas_serialization.hpp"
#include "core/materialepas_traits.hpp"
#include "core/logging.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sqlite3.h>

namespace ReUseX {
class ProjectDB::Impl {
    public:
  std::filesystem::path dbPath;
  bool readOnly;
  sqlite3 *db = nullptr;

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

    // Create project tables if in write mode
    if (!readOnly) {
      createTables();
    }

    ReUseX::core::info("Project database opened successfully");
  }

  ~Impl() {
    ReUseX::core::trace("Closing project database connection");
    if (db) {
      sqlite3_close(db);
    }
  }

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

  /**
   * @brief Ensure a property definition exists in the database
   *
   * Uses INSERT OR IGNORE to upsert property definitions from PropertyTraits
   * metadata. This is called during addMaterialPassport to guarantee that
   * the property_definitions table is populated before inserting values.
   */
  void ensurePropertyDefinitions(
      const ReUseX::core::traits::PropertyDescriptor *props, size_t count,
      const char *category, sqlite3_stmt *stmt) {
    for (size_t i = 0; i < count; ++i) {
      const auto &prop = props[i];

      // Nested object arrays use field_name as ID (not leksikon_guid)
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

  /**
   * @brief Populate property_definitions for all MaterialPassport sections
   */
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
    sqlite3_exec(db, "BEGIN TRANSACTION;", nullptr, nullptr, nullptr);

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

      std::string passport_id = passport.metadata.document_guid; // Use document_guid as id
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

// --- Public Interface Implementation ---
ProjectDB::ProjectDB(std::filesystem::path dbPath, bool readOnly)
    : impl_(std::make_unique<Impl>(std::move(dbPath), readOnly)) {
  impl_->validateSchema();
}

ProjectDB::~ProjectDB() = default;

ProjectDB::ProjectDB(ProjectDB &&) noexcept = default;
ProjectDB &ProjectDB::operator=(ProjectDB &&) noexcept = default;

bool ProjectDB::isOpen() const noexcept {
  return impl_ && impl_->db != nullptr;
}

const std::filesystem::path &ProjectDB::getPath() const noexcept {
  return impl_->dbPath;
}

void ProjectDB::validateSchema() const { impl_->validateSchema(); }

// --- Material Passport Operations ---

ReUseX::core::MaterialPassport
ProjectDB::getMaterialPassport(std::string_view documentGuid) const {
  return impl_->getMaterialPassport(documentGuid);
}

void ProjectDB::addMaterialPassport(const ReUseX::core::MaterialPassport &passport,
                                    std::string_view projectId) {
  impl_->addMaterialPassport(passport, projectId);
}

/*
std::vector<int> ProjectDB::getNodeIds(bool ignoreChildren) const {
  return impl_->getNodeIds(ignoreChildren);
}

cv::Mat ProjectDB::getImage(int nodeId) const {
  return impl_->getImage(nodeId);
}

void ProjectDB::getGraph(std::map<int, rtabmap::Transform> &poses,
                         std::multimap<int, rtabmap::Link> &links,
                         std::map<int, rtabmap::Signature> *signatures,
                         bool optimized, bool withImages, bool withScan) const {

  ReUseX::core::trace("Retrieving graph from database");

  if (!impl_->rtabmap) {
    throw std::runtime_error("Rtabmap instance not initialized");
  }

  impl_->rtabmap->getGraph(poses, links, optimized, withImages, signatures,
                           withScan);

  ReUseX::core::trace("Graph retrieved: {} poses, {} links", poses.size(),
                      links.size());
}

bool ProjectDB::hasSegmentation(int nodeId) const {
  return impl_->hasSegmentation(nodeId);
}

cv::Mat ProjectDB::getLabels(int nodeId) const {
  return impl_->getLabels(nodeId);
}

void ProjectDB::saveLabels(int nodeId, const cv::Mat &labels, bool autoRotate) {
  impl_->saveLabels(nodeId, labels, autoRotate);
}

void ProjectDB::saveLabels(const std::vector<int> &nodeIds,
                           const std::vector<cv::Mat> &labels,
                           bool autoRotate) {
  impl_->saveLabelsBatch(nodeIds, labels, autoRotate);
}
*/
} // namespace ReUseX
