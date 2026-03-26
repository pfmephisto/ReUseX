// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/types.hpp"

#include <Eigen/Core>
#include <pcl/PolygonMesh.h>

#include <nlohmann/json.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace ReUseX::io::speckle {

// --- Speckle Object Model ---

/// Base class for all Speckle objects.
struct Base {
    std::string speckle_type = "Base";
    std::string applicationId;
    std::map<std::string, nlohmann::json> properties;

    /// Child objects (serialized as detached @elements).
    std::vector<std::shared_ptr<Base>> elements;

    virtual ~Base() = default;
};

/// 3D point.
struct Point : Base {
    double x = 0, y = 0, z = 0;
    std::string units = "m";

    Point() { speckle_type = "Objects.Geometry.Point"; }
    Point(double x, double y, double z) : x(x), y(y), z(z) {
        speckle_type = "Objects.Geometry.Point";
    }
};

/// Line segment between two points.
struct Line : Base {
    Point start, end;
    std::string units = "m";

    Line() { speckle_type = "Objects.Geometry.Line"; }
};

/// Triangle/quad mesh.
struct Mesh : Base {
    std::vector<double> vertices; ///< Flat [x,y,z,x,y,z,...]
    std::vector<int> faces;      ///< Packed [n, i0, i1, ..., n, i0, i1, ...]
    std::vector<int> colors;     ///< ARGB integers
    std::string units = "m";

    Mesh() { speckle_type = "Objects.Geometry.Mesh"; }
};

/// Point cloud.
struct Pointcloud : Base {
    std::vector<double> points; ///< Flat [x,y,z,x,y,z,...]
    std::vector<int> colors;   ///< ARGB integers
    std::vector<double> sizes;  ///< Per-point sizes
    std::string units = "m";

    Pointcloud() { speckle_type = "Objects.Geometry.Pointcloud"; }
};

/// Collection / container for grouping objects.
struct Collection : Base {
    std::string name;
    std::string collectionType = "Container";

    Collection() { speckle_type = "Speckle.Core.Models.Collection"; }
};

// --- Client ---

/// Upload-only Speckle client.
class SpeckleClient {
  public:
    /// Construct a client.
    /// @param server_url  Speckle server URL (e.g. "https://app.speckle.systems")
    /// @param project_id  Project/stream ID
    /// @param token       Personal access token (falls back to SPECKLE_TOKEN env)
    SpeckleClient(std::string server_url, std::string project_id,
                  std::string token = "");

    /// Send a root object and all its children to the server.
    /// @return The root object hash (MD5 of canonical JSON).
    std::string send(const Base &root);

    /// Create a commit/version pointing to a root object hash.
    /// @return The commit ID.
    std::string commit(const std::string &object_id,
                       const std::string &branch = "main",
                       const std::string &message = "ReUseX upload");

    /// Convenience: send + commit in one call.
    /// @return The commit ID.
    std::string upload(const Base &root, const std::string &branch = "main",
                       const std::string &message = "ReUseX upload");

    /// Max HTTP batch payload size in bytes (default: 25 MB).
    void set_max_batch_size(std::size_t bytes);

  private:
    std::string server_url_;
    std::string project_id_;
    std::string token_;
    std::size_t max_batch_bytes_ = 25 * 1024 * 1024; // 25 MB
};

// --- Conversion Helpers ---

/// Convert a PCL point cloud to a Speckle Pointcloud.
Pointcloud to_speckle(CloudConstPtr cloud);

/// Convert a PCL PolygonMesh to a Speckle Mesh.
Mesh to_speckle(const pcl::PolygonMesh &mesh);

/// Convert Eigen vertex/face matrices (from Solidifier::toMesh) to a Speckle Mesh.
Mesh to_speckle(const Eigen::MatrixXd &vertices, const Eigen::MatrixXi &faces);

} // namespace ReUseX::io::speckle
