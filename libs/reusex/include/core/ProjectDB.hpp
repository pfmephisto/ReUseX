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

  bool is_open() const noexcept;
  const std::filesystem::path &path() const noexcept;
  int schema_version() const;
  void validate_schema() const;

  // --- Sensor Frame Operations ---

  void save_sensor_frame(int nodeId, const cv::Mat &colorImage);
  std::vector<int> sensor_frame_ids() const;
  cv::Mat sensor_frame_image(int nodeId) const;

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

  // --- Label Definitions ---

  void save_label_definitions(std::string_view cloudName,
                              const std::map<int, std::string> &labelMap);

  std::map<int, std::string>
  label_definitions(std::string_view cloudName) const;

  // --- Mesh Operations ---

  void save_mesh(std::string_view name, const pcl::PolygonMesh &mesh,
                 std::string_view stage = "",
                 std::string_view paramsJson = "");

  pcl::PolygonMesh::Ptr mesh(std::string_view name) const;
  bool has_mesh(std::string_view name) const;

  // --- Pipeline Log ---

  int log_pipeline_start(std::string_view stage,
                         std::string_view paramsJson = "");

  void log_pipeline_end(int logId, bool success,
                        std::string_view errorMsg = "");

  // --- Material Passport Operations ---

  core::MaterialPassport material_passport(std::string_view documentGuid) const;

  void add_material_passport(const core::MaterialPassport &passport,
                             std::string_view projectId);

    private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace ReUseX
