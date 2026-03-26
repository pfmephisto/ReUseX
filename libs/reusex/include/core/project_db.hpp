#pragma once
#include <filesystem>
#include <map>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <string_view>
#include <vector>

namespace ReUseX {

// Forward declarations
namespace core {
struct MaterialPassport;
}

// PCL type aliases (mirror types.hpp to avoid heavy include)
using PointT = pcl::PointXYZRGB;
using NormalT = pcl::Normal;
using LabelT = pcl::Label;
using Cloud = pcl::PointCloud<PointT>;
using CloudPtr = Cloud::Ptr;
using CloudN = pcl::PointCloud<NormalT>;
using CloudNPtr = CloudN::Ptr;
using CloudL = pcl::PointCloud<LabelT>;
using CloudLPtr = CloudL::Ptr;

class ProjectDB {
    public:
  /**
   * @brief Opens a ReUseX project database and validates its schema
   *
   * Creates the following tables if they don't exist (write mode only):
   * - projects: Project metadata (building info, survey details)
   * - property_definitions: Leksikon-based property definitions
   * - material_passports: Material passport documents
   * - passport_property_values: Property values for each passport
   * - passport_log: Audit log for tracking changes
   * - schema_version: Schema version tracking
   * - point_clouds / point_cloud_data: Point cloud storage
   * - label_definitions: Semantic label lookup
   * - meshes: Mesh storage
   * - sensor_frames: Imported sensor frame color images
   * - segmentation_images: Per-frame semantic label images
   * - pipeline_log: Pipeline provenance log
   *
   * @param dbPath Path to the database file
   * @param readOnly If true, opens database in read-only mode
   * @throws std::runtime_error if database cannot be opened or schema is
   * invalid
   */
  explicit ProjectDB(std::filesystem::path dbPath, bool readOnly = false);
  /**
   * @brief Destructor closes database connection
   */
  ~ProjectDB();

  // Non-copyable (RAII resource management)
  ProjectDB(const ProjectDB &) = delete;
  ProjectDB &operator=(const ProjectDB &) = delete;

  // Movable
  ProjectDB(ProjectDB &&) noexcept;
  ProjectDB &operator=(ProjectDB &&) noexcept;

  // --- Core Database Operations ---

  /**
   * @brief Check if database connection is open
   * @return true if database is open and valid
   */
  bool isOpen() const noexcept;

  /**
   * @brief Get the path to the database file
   * @return filesystem path to the database
   */
  const std::filesystem::path &getPath() const noexcept;

  /**
   * @brief Get the current schema version
   * @return schema version number, or -1 if no schema_version table exists
   */
  int getSchemaVersion() const;

  /**
   * @brief Validate that required project database tables exist
   * @throws std::runtime_error if required tables are missing
   */
  void validateSchema() const;

  // --- Sensor Frame Operations ---

  /**
   * @brief Save a color image for a sensor frame
   *
   * Images are stored JPEG-encoded in display orientation (pre-rotated).
   * Replaces any existing frame with the same node_id.
   */
  void saveSensorFrame(int nodeId, const cv::Mat &colorImage);

  /**
   * @brief Get all sensor frame node IDs
   * @return Sorted vector of node IDs
   */
  std::vector<int> getSensorFrameIds() const;

  /**
   * @brief Get the color image for a sensor frame
   * @return Decoded cv::Mat (BGR), empty if not found
   */
  cv::Mat getSensorFrameImage(int nodeId) const;

  // --- Segmentation Image Operations ---

  /**
   * @brief Check if a segmentation image exists for a node
   */
  bool hasSegmentationImage(int nodeId) const;

  /**
   * @brief Get segmentation labels for a node
   *
   * Returns labels as CV_32S with -1 for background, 0+ for class labels.
   * No rotation is applied (images stored in display orientation).
   */
  cv::Mat getSegmentationImage(int nodeId) const;

  /**
   * @brief Save segmentation labels for a single node
   *
   * Labels should be CV_32S with -1 for background. Internally converts
   * to CV_16U with +1 offset for storage (0 = background).
   * No rotation is applied (images assumed in display orientation).
   */
  void saveSegmentationImage(int nodeId, const cv::Mat &labels);

  /**
   * @brief Batch save segmentation labels in a transaction
   */
  void saveSegmentationImages(const std::vector<int> &nodeIds,
                              const std::vector<cv::Mat> &labels);

  // --- Point Cloud Operations ---

  /**
   * @brief Save a PointXYZRGB cloud to the database
   *
   * Uses compact binary serialization (16 bytes/point: xyz + rgba).
   * Replaces any existing cloud with the same name.
   */
  void savePointCloud(std::string_view name, const Cloud &cloud,
                      std::string_view stage = "",
                      std::string_view paramsJson = "");

  /**
   * @brief Save a Normal cloud to the database
   *
   * Uses compact binary serialization (16 bytes/point: nx, ny, nz, curvature).
   */
  void savePointCloud(std::string_view name, const CloudN &cloud,
                      std::string_view stage = "",
                      std::string_view paramsJson = "");

  /**
   * @brief Save a Label cloud to the database
   *
   * Uses compact binary serialization (4 bytes/point: label uint32).
   */
  void savePointCloud(std::string_view name, const CloudL &cloud,
                      std::string_view stage = "",
                      std::string_view paramsJson = "");

  /**
   * @brief Save a PointXYZ cloud to the database
   *
   * Uses compact binary serialization (12 bytes/point: xyz).
   */
  void savePointCloud(std::string_view name,
                      const pcl::PointCloud<pcl::PointXYZ> &cloud,
                      std::string_view stage = "",
                      std::string_view paramsJson = "");

  CloudPtr getPointCloudXYZRGB(std::string_view name) const;
  CloudNPtr getPointCloudNormal(std::string_view name) const;
  CloudLPtr getPointCloudLabel(std::string_view name) const;
  pcl::PointCloud<pcl::PointXYZ>::Ptr
  getPointCloudXYZ(std::string_view name) const;

  bool hasPointCloud(std::string_view name) const;
  void deletePointCloud(std::string_view name);
  std::vector<std::string> listPointClouds() const;

  // --- Label Definitions ---

  /**
   * @brief Save label definitions for a named label cloud
   *
   * Maps integer label IDs to human-readable names (e.g., 0 -> "wall").
   * Replaces any existing definitions for this cloud.
   */
  void saveLabelDefinitions(std::string_view cloudName,
                            const std::map<int, std::string> &labelMap);

  std::map<int, std::string>
  getLabelDefinitions(std::string_view cloudName) const;

  // --- Mesh Operations ---

  /**
   * @brief Save a polygon mesh to the database as binary PLY
   *
   * Replaces any existing mesh with the same name.
   */
  void saveMesh(std::string_view name, const pcl::PolygonMesh &mesh,
                std::string_view stage = "",
                std::string_view paramsJson = "");

  pcl::PolygonMesh::Ptr getMesh(std::string_view name) const;
  bool hasMesh(std::string_view name) const;

  // --- Pipeline Log ---

  /**
   * @brief Log the start of a pipeline stage
   * @return log entry ID (use with logPipelineEnd)
   */
  int logPipelineStart(std::string_view stage,
                       std::string_view paramsJson = "");

  void logPipelineEnd(int logId, bool success,
                      std::string_view errorMsg = "");

  // --- Material Passport Operations ---

  /**
   * @brief Retrieve a material passport by document GUID
   *
   * @param documentGuid Unique document identifier for the passport
   * @return MaterialPassport struct with all sections populated
   * @throws std::runtime_error if passport not found or database error
   */
  core::MaterialPassport getMaterialPassport(std::string_view documentGuid) const;

  /**
   * @brief Retrieve all material passports from the database
   *
   * Iterates over every passport in the material_passports table,
   * ordered by creation date, and returns them as a vector.
   *
   * @return Vector of MaterialPassport structs with all sections populated
   * @throws std::runtime_error if database access fails
   */
  std::vector<core::MaterialPassport> getAllMaterialPassports() const;

  /**
   * @brief Add a new material passport to the database
   *
   * @param passport MaterialPassport to store
   * @param projectId Project ID to associate this passport with
   * @throws std::runtime_error if database write fails
   */
  void addMaterialPassport(const core::MaterialPassport &passport,
                           std::string_view projectId);

    private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace ReUseX
