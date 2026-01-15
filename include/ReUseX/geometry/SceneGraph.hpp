// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <ReUseX/geometry/Registry.hpp>
#include <ReUseX/types.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graphviz.hpp> // write_graphviz, read_graphviz
#include <boost/iterator/filter_iterator.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <fstream>

#include <spdlog/spdlog.h>

#include <fmt/format.h>

#include <set>
#include <string>
#include <vector>

#include <Eigen/StdVector>

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <range/v3/range/concepts.hpp>
#include <range/v3/view/zip.hpp>

enum class NodeType { PointCluster, Plane, Object };

struct PointCluster {
  std::vector<int> point_indices;
};

struct Plane {
  Eigen::Vector4d coefficients;
  Eigen::Vector3d origin;
};

struct Object {
  int label;
};

// Generic vertex bundle
struct VertexData {
  Eigen::Vector3d centroid;
  NodeType type;
  std::variant<PointCluster, Plane> data;
};

struct EdgeData {
  //
};

namespace ReUseX::geometry {

// TODO: Implement the SceneGraph class and make it the core of ReUseX
class SceneGraph
    : public boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                                   VertexData, EdgeData>,
      public Registry {
    protected:
  using Graph = boost::adjacency_list<boost::vecS, boost::vecS,
                                      boost::undirectedS, VertexData, EdgeData>;

    public:
  using Vertex = boost::graph_traits<Graph>::vertex_descriptor;

    protected:
    private:
  CloudConstPtr cloud_ = nullptr;
  std::map<int, std::string> label_map_ = {};
  std::filesystem::path database_path_ = "";

    public:
  SceneGraph() = default;
  ~SceneGraph() = default;

  /** @brief Construct a scene graph from a point cloud and its normals
   *
   * This constructor takes a point cloud and its corresponding normals as
   * input and constructs a hierarchical scene graph representation of the
   * scene.
   */
  SceneGraph(CloudConstPtr cloud, CloudNConstPtr normals);

  /** @brief Extract point clouds corresponding to a specific label
   *
   * This method extracts all point clouds from the scene graph that are
   * associated with a specific semantic label.
   */
  std::vector<CloudConstPtr> extract(int label) const;

  /** @brief Extract point clouds corresponding to a specific label name
   *
   * This method extracts all point clouds from the scene graph that are
   * associated with a specific semantic label name.
   */
  std::vector<CloudConstPtr> ectract(std::string const label_name) const;

  /** @brief Get all unique labels in the scene graph
   *
   * This method returns a set of all unique semantic labels present in the
   * scene graph.
   */
  std::set<int> get_labels() const;

  /** @brief Set the database path
   *
   * This method sets the path to the database containing the precomputed
   * labels for the point cloud views.
   */
  void set_database_path(std::filesystem::path db_path);

  /** @brief Save the scene graph to a file
   *
   * This method serizalizes the scene graph and saves it to a file.
   */
  int save(const std::filesystem::path &path) const;

  /** @brief Export the scene graph to differnet formats
   *
   * This method exports the scene graph to different formats such as .dot or
   * json for use in other applications.
   */
  int export(const std::filesystem::path &path) const;

  /** @brief Load the scene graph from a file
   *
   * This method deserializes the scene graph from a file.
   */
  static SceneGraph load(const std::filesystem::path &path);

    private:
  /** @brief Segment the point cloud into similar patches which are the smallest
   * building block of the graph
   *
   * This method uses super point to segment the point cloud into small patches
   * based on spatial proximity and normal similarity. Those patches are then
   * the leaf nodes in the scene graph and all higher level nodes are
   * constructed as compositions of those patches.
   */
  void patch_segmentation();

  /** @brief Grow planar regions from the patches
   *
   * This method iteratively grows planar regions from the patches by merging
   * neighboring patches that satisfy certain planarity criteria such as normal
   * similarity and distance to the plane.
   *
   * The result are Plane nodes in the scene graph that connect to the leaf
   * patches
   */
  void planar_region_growing();

  /** @brief Project the labels stored in the database on tho the assebled point
   * cloud.
   *
   * For each view in the database this constructs a croped point cloud,
   * computes a z-buffer and the assigns the precomputed labels to the closses
   * point in the point cloud.
   *
   * The result are semantic label nodes which are connected to the leaf
   * patches.
   */
  void project_labels_from_database();

  /** @brief Segment the scene into rooms
   *
   * This method segments the scene into rooms based on spatial visibility
   * relations. It uses raytracing to determine which parts of the scene are
   * visible from any other part and uses markov clustering to group the parts
   * into rooms based on visibility.
   *
   * The result are Room nodes in the scene graph that connect to
   * the semantic label nodes.
   */
  void segment_rooms();
};
} // namespace ReUseX::geometry

template <> struct fmt::formatter<ReUseX::geometry::SceneGraph> {
  // Parse function (optional)
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) {
    return ctx.begin();
  }

  // Format function
  template <typename FormatContext>
  auto format(const ReUseX::geometry::SceneGraph &obj,
              FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "SceneGraph");
  }
};

// Implementation files
