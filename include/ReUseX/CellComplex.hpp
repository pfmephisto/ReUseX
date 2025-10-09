// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/iterator/filter_iterator.hpp>

#include <spdlog/spdlog.h>

#include <fmt/format.h>

#include <filesystem>
#include <set>
#include <string>
#include <vector>

namespace fs = std::filesystem;

// Tags to distinguish node types
enum class NodeType { Cell, Face, Vertex };

struct VertexData {
  int id;
};
struct FaceData {
  int id;
  int plane_id;
  // double area;
};
struct CellData {
  int id;
  std::set<int> wall_ids{};
};

// Generic vertex bundle
struct VerteciesData {
  // TODO: Consider moving the ID field to here
  NodeType type;
  std::variant<VertexData, FaceData, CellData> data;
};

struct EdgeData {
  bool is_main = false;
};

namespace ReUseX {
class Registry {
    private:
  // Keyed by name and type_index
  std::map<std::pair<std::string, std::type_index>, std::shared_ptr<void>>
      registry;

    public:
  virtual ~Registry() = default;
  template <typename Key, typename T>
  std::pair<boost::associative_property_map<std::map<Key, T>>, bool>
  add_property_map(const std::string &name) {
    auto key = std::make_pair(name, std::type_index(typeid(T)));
    auto it = registry.find(key);
    if (it != registry.end()) {
      // Already exists: recover the map and wrap it in a property_map
      auto map_ptr = std::static_pointer_cast<std::map<Key, T>>(it->second);
      return {boost::associative_property_map<std::map<Key, T>>(*map_ptr),
              false};
      // already exists
      /*
      auto ptr = std::static_pointer_cast<
          boost::associative_property_map<std::map<Key, T>>>(it->second);
      return {*ptr, false};
      */
    } else {
      // Create new map and store in registry
      auto m = std::make_shared<std::map<Key, T>>();
      registry[key] = m;
      return {boost::associative_property_map<std::map<Key, T>>(*m), true};
      /*
      auto m = std::make_shared<std::map<Key, T>>();
      auto pm =
          std::make_shared<boost::associative_property_map<std::map<Key, T>>>(
              *m);
      registry[key] = pm;
      return {*pm, true};
      */
    }
  }

  template <typename Key, typename T>
  const boost::associative_property_map<std::map<Key, T>>
  property_map(const std::string &name) const {
    auto key = std::make_pair(name, std::type_index(typeid(T)));
    auto it = registry.find(key);
    if (it == registry.end()) {
      spdlog::error("Property map not found: {}", name);
      throw std::runtime_error("Property map not found");
    }
    auto map_ptr = std::static_pointer_cast<std::map<Key, T>>(it->second);
    return boost::associative_property_map<std::map<Key, T>>(*map_ptr);
    /*
    return *std::static_pointer_cast<
        boost::associative_property_map<std::map<Key, T>>>(it->second);
            */
  }
};

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

  inline Vertex add_vertex(std::optional<int> id = std::nullopt) {
    auto v = boost::add_vertex(*this);
    (*this)[v].type = NodeType::Vertex;
    (*this)[v].data =
        id.has_value() ? VertexData{*id} : VertexData{vertex_count++};
    return v;
  };

  template <typename T>
  inline Vertex add_face(T begin, T end, int plane_id = -1) {
    auto f = boost::add_vertex(*this);
    (*this)[f].type = NodeType::Face;
    (*this)[f].data = FaceData{face_count++, plane_id};
    for (auto it = begin; it != end; ++it)
      boost::add_edge(*it, f, *this);
    return f;
  };
  template <typename Range>
  inline Vertex add_face(Range vertices, int plane_id = -1) {
    return add_face(std::begin(vertices), std::end(vertices), plane_id);
  };

  template <typename T> inline Vertex add_cell(T begin, T end) {
    auto c = boost::add_vertex(*this);
    (*this)[c].type = NodeType::Cell;
    (*this)[c].data = CellData{cell_count++};
    for (auto it = begin; it != end; ++it)
      boost::add_edge(*it, c, *this);
    return c;
  };
  template <typename Range> inline Vertex add_cell(Range faces) {
    return add_cell(std::begin(faces), std::end(faces));
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
