// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "ReUseX/registry.hpp"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/iterator/filter_iterator.hpp>

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

namespace fs = std::filesystem;

// Tags to distinguish node types
enum class NodeType { Cell, Face, Vertex };

struct VertexData {};
struct FaceData {
  int plane_id;
};
struct CellData {
  std::set<int> wall_ids{};
};

// Generic vertex bundle
struct VerteciesData {
  // TODO: Consider moving the ID field to here
  NodeType type;
  int id;
  Eigen::Vector3d pos;
  std::variant<VertexData, FaceData, CellData> data;
};

struct EdgeData {
  bool is_main = false;
};

template <typename Scalar, int Rows>
using EigenVectorContainer =
    std::vector<Eigen::Matrix<Scalar, Rows, 1>,
                Eigen::aligned_allocator<Eigen::Matrix<Scalar, Rows, 1>>>;

namespace ReUseX {

class CellComplex
    : public boost::adjacency_list<boost::vecS,        // edge container
                                   boost::vecS,        // vertex container
                                   boost::undirectedS, // cells–faces are
                                   // bidirectional
                                   // boost::directedS,
                                   VerteciesData, // vertex bundle
                                   EdgeData       // edge bundle
                                   >,
      public Registry {
    protected:
  using Graph = boost::adjacency_list<boost::vecS, boost::vecS,
                                      boost::undirectedS, VertexData, EdgeData>;

    public:
  using Vertex = boost::graph_traits<Graph>::vertex_descriptor;
  using GraphIter = boost::graph_traits<CellComplex>::vertex_iterator;
  using AdjacencyIter = boost::graph_traits<CellComplex>::adjacency_iterator;

    protected:
  template <NodeType T> struct Is_Type {
    const CellComplex *g;
    Is_Type() = delete;
    Is_Type(const CellComplex *g) : g(g) {}
    bool operator()(CellComplex::vertex_descriptor vd) const {
      auto const &prop = (*g)[vd];
      return prop.type == T;
    }
  };
  using IsVertex = Is_Type<NodeType::Vertex>;
  using IsCell = Is_Type<NodeType::Cell>;
  using IsFace = Is_Type<NodeType::Face>;
  struct IsFaceBetweenCells {
    const CellComplex *g;
    IsFaceBetweenCells() = delete;
    IsFaceBetweenCells(const CellComplex *g) : g(g) {}
    bool operator()(CellComplex::vertex_descriptor vd) const {
      auto const &prop = (*g)[vd];
      int n_adj = 0;
      auto [begin, end] = boost::adjacent_vertices(vd, *g);
      for (auto it = begin; it != end; ++it)
        if ((*g)[*it].type == NodeType::Cell)
          ++n_adj;
      return prop.type == NodeType::Face && n_adj >= 2;
    }
  };

  using VertexIterator = boost::filter_iterator<IsVertex, GraphIter>;
  using FaceIterator = boost::filter_iterator<IsFace, GraphIter>;
  using FaceBetweenCellIterator =
      boost::filter_iterator<IsFaceBetweenCells, GraphIter>;
  using CellIterator = boost::filter_iterator<IsCell, GraphIter>;
  using VertexOnFaceIterator = boost::filter_iterator<IsVertex, AdjacencyIter>;
  using FaceOnCellIterator = boost::filter_iterator<IsFace, AdjacencyIter>;
  using CellOnFaceIterator = boost::filter_iterator<IsCell, AdjacencyIter>;

    private:
  int vertex_count = 0;
  int face_count = 0;
  int cell_count = 0;

    public:
  size_t n_rooms, n_walls;

  CellComplex() = delete;

  CellComplex(
      std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>
          &planes,
      std::vector<size_t> &verticals, std::vector<size_t> &horizontals,
      std::vector<std::pair<size_t, size_t>> &pairs,
      std::array<double, 2> min_xy, std::array<double, 2> max_xy,
      std::optional<
          std::function<void(size_t, std::vector<std::array<double, 3>> const &,
                             std::vector<int> const &)>>
          viz_func = std::nullopt);

  // TODO: We might want to return something here
  template <typename PointT = pcl::PointXYZ>
  auto compute_face_coverage(pcl::PointCloud<PointT>::ConstPtr cloud,
                             EigenVectorContainer<double, 4> &planes,
                             std::vector<pcl::IndicesPtr> &inliers,
                             const double grid_size = 0.2) -> void;

  template <typename PointT = pcl::PointXYZ, typename PointN = pcl::Normal,
            typename PointL = pcl::Label>
  auto compute_room_probabilities(pcl::PointCloud<PointT>::ConstPtr cloud,
                                  pcl::PointCloud<PointN>::ConstPtr normals,
                                  pcl::PointCloud<PointL>::ConstPtr labels,
                                  const double grid_size = 0.2) -> void;

  inline Vertex add_vertex(Eigen::Vector3d pos) {
    auto v = boost::add_vertex(*this);
    (*this)[v].type = NodeType::Vertex;
    (*this)[v].id = vertex_count++;
    (*this)[v].pos = pos;
    (*this)[v].data = VertexData{};
    return v;
  };

  template <typename T>
  inline Vertex add_face(Eigen::Vector3d pos, T begin, T end,
                         int plane_id = -1) {
    auto f = boost::add_vertex(*this);
    (*this)[f].type = NodeType::Face;
    (*this)[f].id = face_count++;
    (*this)[f].pos = pos;
    (*this)[f].data = FaceData{plane_id};
    for (auto it = begin; it != end; ++it)
      boost::add_edge(*it, f, *this);
    return f;
  };
  template <typename Range>
  inline Vertex add_face(Eigen::Vector3d pos, Range vertices,
                         int plane_id = -1) {
    return add_face(pos, std::begin(vertices), std::end(vertices), plane_id);
  };

  template <typename T>
  inline Vertex add_cell(Eigen::Vector3d pos, T begin, T end) {
    auto c = boost::add_vertex(*this);
    (*this)[c].type = NodeType::Cell;
    (*this)[c].id = cell_count++;
    (*this)[c].pos = pos;
    (*this)[c].data = CellData{};
    for (auto it = begin; it != end; ++it)
      boost::add_edge(*it, c, *this);
    return c;
  };
  template <typename Range>
  inline Vertex add_cell(Eigen::Vector3d pos, Range faces) {
    return add_cell(pos, std::begin(faces), std::end(faces));
  };

  size_t num_vertices() const;
  size_t num_faces() const;
  size_t num_cells() const;

  std::ostream &operator<<(std::ostream &os) const;

  auto vertices_begin() const -> VertexIterator;
  auto vertices_end() const -> VertexIterator;

  auto faces_begin() const -> FaceIterator;
  auto faces_end() const -> FaceIterator;

  auto faces_between_cells_begin() const -> FaceBetweenCellIterator;
  auto faces_between_cells_end() const -> FaceBetweenCellIterator;

  auto cells_begin() const -> CellIterator;
  auto cells_end() const -> CellIterator;

  // TODO: Consider renameing to x_around_y_{begin/end} e.g.
  // vertices_around_face_begin
  auto vertices_begin(Vertex f) const -> VertexOnFaceIterator;
  auto vertices_end(Vertex f) const -> VertexOnFaceIterator;

  // auto cells_begin(Vertex f) const -> CellOnFaceIterator;
  // auto cells_end(Vertex f) const -> CellOnFaceIterator;

  auto faces_begin(Vertex c) const -> FaceOnCellIterator;
  auto faces_end(Vertex c) const -> FaceOnCellIterator;

  auto get_a(Vertex f) const -> Vertex;
  auto get_b(Vertex f) const -> Vertex;
};
} // namespace ReUseX

template <> struct fmt::formatter<ReUseX::CellComplex> {
  // Parse function (optional)
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) {
    return ctx.begin();
  }

  // Format function
  template <typename FormatContext>
  auto format(const ReUseX::CellComplex &obj, FormatContext &ctx) {
    return fmt::format_to(ctx.out(), "[{}c {}f {}v {}r  {}w]", obj.num_cells(),
                          obj.num_faces(), obj.num_vertices(), obj.n_rooms,
                          obj.n_walls);
  }
};

// Implementation files
#include "./CellComplexFaceCoverage.hpp"
#include "./CellComplexRoomProbabilites.hpp"
