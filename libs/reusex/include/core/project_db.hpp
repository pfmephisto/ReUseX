#pragma once
#include <filesystem>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <string_view>
#include <vector>

namespace ReUseX {

// Forward declarations
namespace core {
struct MaterialPassport;
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
   * @brief Validate that required project database tables exist
   * @throws std::runtime_error if required tables are missing
   */
  void validateSchema() const;

  // // --- Node Operations ---

  // /**
  //  * @brief Get list of all node IDs in the database
  //  *
  //  * @param ignoreChildren If true, only return parent nodes (not children)
  //  * @return Vector of node IDs
  //  */
  // std::vector<int> getNodeIds(bool ignoreChildren = false) const;

  // /**
  //  * @brief Get image data for a specific node
  //  *
  //  * Retrieves the image from the Data table and applies 90° clockwise
  //  * rotation to match RTABMap's coordinate convention.
  //  *
  //  * @param nodeId The node ID to retrieve image for
  //  * @return OpenCV Mat containing the image (empty if not found)
  //  * @throws std::runtime_error if database access fails
  //  */
  // cv::Mat getImage(int nodeId) const;

  // // --- Graph Operations ---

  // /**
  //  * @brief Retrieve the SLAM graph (poses, links, signatures)
  //  *
  //  * This uses RTABMap's high-level Rtabmap class to extract graph data.
  //  *
  //  * @param poses Output map of node IDs to 3D transforms
  //  * @param links Output multimap of node IDs to links (edges)
  //  * @param signatures Optional output map of node IDs to full signatures
  //  * @param optimized If true, return optimized poses
  //  * @param withImages If true, include image data in signatures
  //  * @param withScan If true, include laser scan data in signatures
  //  * @throws std::runtime_error if graph retrieval fails
  //  */
  // void getGraph(std::map<int, rtabmap::Transform> &poses,
  //               std::multimap<int, rtabmap::Link> &links,
  //               std::map<int, rtabmap::Signature> *signatures = nullptr,
  //               bool optimized = true, bool withImages = false,
  //               bool withScan = false) const;

  // // --- Segmentation Table Operations ---

  // /**
  //  * @brief Check if a node has segmentation labels stored
  //  *
  //  * @param nodeId Node ID to check
  //  * @return true if labels exist for this node
  //  */
  // bool hasSegmentation(int nodeId) const;

  // /**
  //  * @brief Get segmentation labels for a node
  //  *
  //  * Returns labels as CV_32S with -1 for unlabeled/background pixels
  //  * and 0+ for class labels. Applies 90° clockwise rotation to match
  //  * the image coordinate system.
  //  *
  //  * @param nodeId Node ID to retrieve labels for
  //  * @return OpenCV Mat with labels (CV_32S), empty if not found
  //  * @throws std::runtime_error if database access fails
  //  */
  // cv::Mat getLabels(int nodeId) const;

  // /**
  //  * @brief Save segmentation labels for a single node
  //  *
  //  * Labels should be CV_32S with -1 for unlabeled/background and 0+
  //  * for class labels. Internally converts to CV_16U with +1 offset
  //  * for storage (0 = background).
  //  *
  //  * @param nodeId Node ID to save labels for
  //  * @param labels OpenCV Mat with labels (CV_32S)
  //  * @param autoRotate If true, applies 90° counterclockwise rotation before
  //  * saving
  //  * @throws std::runtime_error if node doesn't exist or save fails
  //  */
  // void saveLabels(int nodeId, const cv::Mat &labels, bool autoRotate = true);

  // /**
  //  * @brief Save segmentation labels for multiple nodes (batch operation)
  //  *
  //  * Uses a database transaction for efficiency. All nodes must exist
  //  * in the Node table or the entire operation fails.
  //  *
  //  * @param nodeIds Vector of node IDs
  //  * @param labels Vector of label Mats (must match nodeIds length)
  //  * @param autoRotate If true, applies 90° counterclockwise rotation before
  //  * saving
  //  * @throws std::runtime_error if sizes don't match, nodes don't exist, or
  //  save
  //  * fails
  //  */
  // void saveLabels(const std::vector<int> &nodeIds,
  //                 const std::vector<cv::Mat> &labels, bool autoRotate =
  //                 true);

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
