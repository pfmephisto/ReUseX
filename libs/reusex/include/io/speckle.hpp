// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/types.hpp"

#include <Eigen/Core>
#include <pcl/PolygonMesh.h>

#include <nlohmann/json.hpp>

#include <array>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace reusex::io {
struct ExportScene;
}

namespace reusex::io::speckle {

// --- Speckle Object Model ---

/// Base class for all Speckle objects.
struct Base {
  std::string speckle_type = "Base";
  std::string applicationId;
  std::string name;
  /// Custom properties — serialized nested under a "properties" sub-path
  /// (Speckle v3 DataObject convention). Values may themselves be objects.
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
  std::vector<int> faces;       ///< Packed [n, i0, i1, ..., n, i0, i1, ...]
  std::vector<int> colors;      ///< ARGB integers
  std::string units = "m";

  Mesh() { speckle_type = "Objects.Geometry.Mesh"; }
};

/// Point cloud.
struct Pointcloud : Base {
  std::vector<double> points; ///< Flat [x,y,z,x,y,z,...]
  std::vector<int> colors;    ///< ARGB integers
  std::vector<double> sizes;  ///< Per-point sizes
  std::string units = "m";

  Pointcloud() { speckle_type = "Objects.Geometry.Pointcloud"; }
};

/// Shared definition for instance proxies. The `definitionAppId` is what
/// InstanceProxy.definitionId must match. The `objects` list holds the
/// `applicationId` of every geometry object that participates in the
/// definition; those objects live as siblings of the InstanceProxy entries
/// in the host collection's `elements` (Speckle v3 proxy pattern).
struct InstanceDefinitionProxy : Base {
  std::string definitionAppId;
  int maxDepth = 0;
  std::string units = "m";
  std::vector<std::string> objects;

  InstanceDefinitionProxy() {
    speckle_type = "Speckle.Core.Models.Instances.InstanceDefinitionProxy";
  }
};

/// Positioned reference to an InstanceDefinitionProxy. Holds a 4x4 transform
/// (row-major) and points at the definition via `definitionId`.
struct InstanceProxy : Base {
  std::string definitionId;
  std::array<double, 16> transform{1, 0, 0, 0, 0, 1, 0, 0,
                                   0, 0, 1, 0, 0, 0, 0, 1};
  int maxDepth = 0;
  std::string units = "m";

  InstanceProxy() {
    speckle_type = "Speckle.Core.Models.Instances.InstanceProxy";
  }
};

/// Collection / container for grouping objects. Speckle SDK v3 keeps proxy
/// lists (instance definitions, render materials, color proxies) as
/// top-level siblings of `@elements` on the root collection, so they live
/// here rather than being mixed into the elements list.
struct Collection : Base {
  std::string collectionType;
  /// Speckle SDK version marker. Set to 3 on the root collection of a
  /// v3-style model; leave at 0 elsewhere (the field is then omitted).
  int version = 0;
  /// When true, child `elements` are serialized inline (under the key
  /// `elements`, no `@` prefix) instead of as detached references under
  /// `@elements`. Required for clients (e.g. the reuse-x webapp) that read
  /// `speckleRoot.elements` directly without resolving Speckle-style
  /// detached references — the JS SDK Traverser keeps the `@elements` key
  /// verbatim, so the webapp never sees the data when it's detached. Only
  /// enable for collections small enough to fit comfortably in a single
  /// Speckle object payload.
  bool embed_elements = false;
  std::vector<std::shared_ptr<InstanceDefinitionProxy>>
      instanceDefinitionProxies;

  Collection() {
    speckle_type = "Speckle.Core.Models.Collections.Collection";
  }
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
  /// Create the branch if it does not already exist (no-op when it does).
  void ensure_branch(const std::string &branch);

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

/// Convert Eigen vertex/face matrices (from Solidifier::toMesh) to a Speckle
/// Mesh.
Mesh to_speckle(const Eigen::MatrixXd &vertices, const Eigen::MatrixXi &faces);

// --- Scene Export ---

/// One model (branch) to upload to Speckle.
struct SpeckleModel {
  std::string model_name;     ///< branch name (e.g., "cloud", "semantic")
  std::shared_ptr<Base> root; ///< root object for this model's version
};

/// Configuration for building Speckle objects from an ExportScene.
/// Currently controls how external image URLs are constructed.
struct ExportConfig {
  std::string project_id;
  std::string image_url_base = "https://minio.chrk.site/files/reusex";
};

/// Build per-model Speckle objects from an ExportScene.
/// Returns one SpeckleModel per non-empty category.
auto export_to_speckle(const ExportScene &scene, const ExportConfig &cfg)
    -> std::vector<SpeckleModel>;

} // namespace reusex::io::speckle
