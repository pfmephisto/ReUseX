#pragma once
#include "reusex/types.hpp"

#include <array>
#include <filesystem>
#include <map>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <pcl/PolygonMesh.h>
#include <pcl/TextureMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <string_view>
#include <vector>

namespace ReUseX {

// Forward declarations
namespace core {
struct MaterialPassport;
struct SensorIntrinsics;
}

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

  bool is_open() const noexcept;
  const std::filesystem::path &path() const noexcept;
  int schema_version() const;
  void validate_schema() const;

  // --- Sensor Frame Operations ---

  void save_sensor_frame(int nodeId, const cv::Mat &colorImage);

  void save_sensor_frame(int nodeId, const cv::Mat &color,
                         const cv::Mat &depth, const cv::Mat &confidence,
                         const std::array<double, 16> &worldPose,
                         const core::SensorIntrinsics &intrinsics);

  std::vector<int> sensor_frame_ids() const;
  cv::Mat sensor_frame_image(int nodeId) const;
  cv::Mat sensor_frame_depth(int nodeId) const;
  cv::Mat sensor_frame_confidence(int nodeId) const;
  std::array<double, 16> sensor_frame_pose(int nodeId) const;
  core::SensorIntrinsics sensor_frame_intrinsics(int nodeId) const;
  bool has_sensor_frame(int nodeId) const;

  // --- Segmentation Image Operations ---

  bool has_segmentation_image(int nodeId) const;
  cv::Mat segmentation_image(int nodeId) const;
  void save_segmentation_image(int nodeId, const cv::Mat &labels);
  void save_segmentation_images(const std::vector<int> &nodeIds,
                                const std::vector<cv::Mat> &labels);

  // --- Point Cloud Operations ---

  void save_point_cloud(std::string_view name, const Cloud &cloud,
                        std::string_view stage = "",
                        std::string_view paramsJson = "");

  void save_point_cloud(std::string_view name, const CloudN &cloud,
                        std::string_view stage = "",
                        std::string_view paramsJson = "");

  void save_point_cloud(std::string_view name, const CloudL &cloud,
                        std::string_view stage = "",
                        std::string_view paramsJson = "");

  void save_point_cloud(std::string_view name,
                        const pcl::PointCloud<pcl::PointXYZ> &cloud,
                        std::string_view stage = "",
                        std::string_view paramsJson = "");

  CloudPtr point_cloud_xyzrgb(std::string_view name) const;
  CloudNPtr point_cloud_normal(std::string_view name) const;
  CloudLPtr point_cloud_label(std::string_view name) const;
  pcl::PointCloud<pcl::PointXYZ>::Ptr
  point_cloud_xyz(std::string_view name) const;

  bool has_point_cloud(std::string_view name) const;
  void delete_point_cloud(std::string_view name);
  std::vector<std::string> list_point_clouds() const;
  std::string point_cloud_type(std::string_view name) const;

  // --- Label Definitions ---

  void save_label_definitions(std::string_view cloudName,
                              const std::map<int, std::string> &labelMap);

  std::map<int, std::string>
  label_definitions(std::string_view cloudName) const;

  // --- Mesh Operations ---

  void save_mesh(std::string_view name, const pcl::PolygonMesh &mesh,
                 std::string_view stage = "", std::string_view paramsJson = "");

  void save_mesh(std::string_view name, const pcl::TextureMesh &mesh,
                 std::string_view stage = "", std::string_view paramsJson = "");

  pcl::PolygonMesh::Ptr mesh(std::string_view name) const;
  pcl::TextureMesh::Ptr texture_mesh(std::string_view name) const;
  bool has_mesh(std::string_view name) const;
  std::vector<std::string> list_meshes() const;

  // --- Pipeline Log ---

  int log_pipeline_start(std::string_view stage,
                         std::string_view paramsJson = "");

  void log_pipeline_end(int logId, bool success,
                        std::string_view errorMsg = "");

  struct PipelineLogEntry {
    int id;
    std::string stage;
    std::string started_at;
    std::string finished_at;  // Empty if still running
    std::string parameters;   // JSON string
    std::string status;       // "running", "success", "failed"
    std::string error_msg;    // Empty if no error
  };

  std::vector<PipelineLogEntry> pipeline_log(int limit = 0) const;

  // --- Project Summary ---

  struct ProjectSummary {
    struct CloudInfo {
      std::string name;
      std::string type;           // "PointXYZRGB", "Normal", "Label", "PointXYZ"
      size_t point_count;
      size_t width;
      size_t height;
      bool organized;             // height > 1
      std::map<int, std::string> labels;  // Only for Label clouds
    };

    struct MeshInfo {
      std::string name;
      int vertex_count;
      int face_count;
    };

    struct SensorFrameInfo {
      int total_count;
      int width;                  // 0 if no frames
      int height;                 // 0 if no frames
      int segmented_count;        // frames with segmentation
    };

    std::filesystem::path path;
    int schema_version;
    std::vector<CloudInfo> clouds;
    std::vector<MeshInfo> meshes;
    SensorFrameInfo sensor_frames;
    int material_passport_count;
  };

  ProjectSummary project_summary() const;

  // --- Material Passport Operations ---

  core::MaterialPassport material_passport(std::string_view documentGuid) const;
  std::vector<core::MaterialPassport> all_material_passports() const;

  void add_material_passport(const core::MaterialPassport &passport,
                             std::string_view projectId);

  /**
   * @brief Delete a material passport by GUID
   * @param documentGuid Document GUID to delete
   * @throws std::runtime_error if passport does not exist
   */
  void delete_material_passport(std::string_view documentGuid);

  // --- Project Metadata Operations ---

  struct ProjectMetadata {
    std::string id;
    std::string name;
    std::string building_address;
    int year_of_construction = 0;  // 0 = not set
    std::string survey_date;
    std::string survey_organisation;
    std::string notes;
  };

  /**
   * @brief Get project metadata by project ID
   * @param projectId Project identifier (default: "default")
   * @return Project metadata
   * @throws std::runtime_error if project does not exist
   */
  ProjectMetadata get_project_metadata(std::string_view projectId = "default") const;

  /**
   * @brief Update project metadata
   * @param metadata Project metadata to update
   * Creates project if it doesn't exist
   */
  void update_project_metadata(const ProjectMetadata &metadata);

  /**
   * @brief List all project IDs in the database
   * @return Vector of project IDs
   */
  std::vector<std::string> list_project_ids() const;

    private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace ReUseX
