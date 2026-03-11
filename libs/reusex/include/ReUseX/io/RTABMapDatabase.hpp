// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <filesystem>
#include <map>
#include <memory>
#include <vector>

#include <opencv2/core/mat.hpp>

// Forward declarations to avoid including RTABMap headers
namespace rtabmap {
class Transform;
class Link;
class Signature;
} // namespace rtabmap

namespace ReUseX::io {

/**
 * @brief Core database class that wraps RTABMap's database functionality
 *
 * This class provides a unified interface for accessing RTABMap databases,
 * including both the core RTABMap tables (Node, Data, Link) and custom
 * extensions (Segmentation table for labels).
 *
 * The class uses RAII for resource management and the Pimpl idiom to hide
 * RTABMap implementation details. It is not thread-safe; create separate
 * instances for each thread if concurrent access is needed.
 *
 * Thread Safety: NOT thread-safe. sqlite3 connections should not be shared
 * between threads. Create per-thread instances if needed.
 */
class RTABMapDatabase {
public:
    /**
     * @brief Opens an RTABMap database and validates its schema
     *
     * @param dbPath Path to the database file
     * @param readOnly If true, opens database in read-only mode
     * @throws std::runtime_error if database cannot be opened or schema is invalid
     */
    explicit RTABMapDatabase(std::filesystem::path dbPath, bool readOnly = false);

    /**
     * @brief Destructor closes database connection
     */
    ~RTABMapDatabase();

    // Non-copyable (RAII resource management)
    RTABMapDatabase(const RTABMapDatabase&) = delete;
    RTABMapDatabase& operator=(const RTABMapDatabase&) = delete;

    // Movable
    RTABMapDatabase(RTABMapDatabase&&) noexcept;
    RTABMapDatabase& operator=(RTABMapDatabase&&) noexcept;

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
    const std::filesystem::path& getPath() const noexcept;

    /**
     * @brief Validate that required RTABMap tables exist
     * @throws std::runtime_error if required tables are missing
     */
    void validateSchema() const;

    // --- Node Operations ---

    /**
     * @brief Get list of all node IDs in the database
     *
     * @param ignoreChildren If true, only return parent nodes (not children)
     * @return Vector of node IDs
     */
    std::vector<int> getNodeIds(bool ignoreChildren = false) const;

    /**
     * @brief Get image data for a specific node
     *
     * Retrieves the image from the Data table and applies 90° clockwise
     * rotation to match RTABMap's coordinate convention.
     *
     * @param nodeId The node ID to retrieve image for
     * @return OpenCV Mat containing the image (empty if not found)
     * @throws std::runtime_error if database access fails
     */
    cv::Mat getImage(int nodeId) const;

    // --- Graph Operations ---

    /**
     * @brief Retrieve the SLAM graph (poses, links, signatures)
     *
     * This uses RTABMap's high-level Rtabmap class to extract graph data.
     *
     * @param poses Output map of node IDs to 3D transforms
     * @param links Output multimap of node IDs to links (edges)
     * @param signatures Optional output map of node IDs to full signatures
     * @param optimized If true, return optimized poses
     * @param withImages If true, include image data in signatures
     * @param withScan If true, include laser scan data in signatures
     * @throws std::runtime_error if graph retrieval fails
     */
    void getGraph(
        std::map<int, rtabmap::Transform>& poses,
        std::multimap<int, rtabmap::Link>& links,
        std::map<int, rtabmap::Signature>* signatures = nullptr,
        bool optimized = true,
        bool withImages = false,
        bool withScan = false) const;

    // --- Segmentation Table Operations ---

    /**
     * @brief Check if a node has segmentation labels stored
     *
     * @param nodeId Node ID to check
     * @return true if labels exist for this node
     */
    bool hasSegmentation(int nodeId) const;

    /**
     * @brief Get segmentation labels for a node
     *
     * Returns labels as CV_32S with -1 for unlabeled/background pixels
     * and 0+ for class labels. Applies 90° clockwise rotation to match
     * the image coordinate system.
     *
     * @param nodeId Node ID to retrieve labels for
     * @return OpenCV Mat with labels (CV_32S), empty if not found
     * @throws std::runtime_error if database access fails
     */
    cv::Mat getLabels(int nodeId) const;

    /**
     * @brief Save segmentation labels for a single node
     *
     * Labels should be CV_32S with -1 for unlabeled/background and 0+
     * for class labels. Internally converts to CV_16U with +1 offset
     * for storage (0 = background).
     *
     * @param nodeId Node ID to save labels for
     * @param labels OpenCV Mat with labels (CV_32S)
     * @param autoRotate If true, applies 90° counterclockwise rotation before saving
     * @throws std::runtime_error if node doesn't exist or save fails
     */
    void saveLabels(int nodeId, const cv::Mat& labels, bool autoRotate = true);

    /**
     * @brief Save segmentation labels for multiple nodes (batch operation)
     *
     * Uses a database transaction for efficiency. All nodes must exist
     * in the Node table or the entire operation fails.
     *
     * @param nodeIds Vector of node IDs
     * @param labels Vector of label Mats (must match nodeIds length)
     * @param autoRotate If true, applies 90° counterclockwise rotation before saving
     * @throws std::runtime_error if sizes don't match, nodes don't exist, or save fails
     */
    void saveLabels(const std::vector<int>& nodeIds,
                   const std::vector<cv::Mat>& labels,
                   bool autoRotate = true);

private:
    // Pimpl idiom to hide RTABMap dependencies
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace ReUseX::io
