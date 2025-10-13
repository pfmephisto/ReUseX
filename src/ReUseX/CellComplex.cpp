// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ReUseX/CellComplex.hpp"

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/graph/adj_list_serialize.hpp>

namespace ReUseX {

template <NodeType P> size_t _count(ReUseX::CellComplex const *c) {
  size_t count = 0;
  auto [begin, end] = boost::vertices(*c);
  for (auto it = begin; it != end; ++it) {
    if ((*c)[*it].type == P) {
      ++count;
    }
  }
  return count;
}
size_t CellComplex::num_vertices() const {
  return _count<NodeType::Vertex>(this);
}
size_t CellComplex::num_faces() const { return _count<NodeType::Face>(this); }
size_t CellComplex::num_cells() const { return _count<NodeType::Cell>(this); }

std::ostream &CellComplex::operator<<(std::ostream &os) const {
  spdlog::warn("This << operator mith chage in the future");
  os << fmt::format("[{}c {}f {}v {}r  {}w]", num_cells(), num_faces(),
                    num_vertices(), n_rooms, n_walls);
  return os;
}

auto CellComplex::vertices_begin() const -> CellComplex::VertexIterator {
  auto [begin, end] = boost::vertices(*this);
  return boost::make_filter_iterator(IsVertex{this}, begin, end);
}

auto CellComplex::vertices_end() const -> CellComplex::VertexIterator {
  auto end = boost::vertices(*this).second;
  return boost::make_filter_iterator(IsVertex{this}, end, end);
}

auto CellComplex::faces_begin() const -> CellComplex::FaceIterator {
  auto [begin, end] = boost::vertices(*this);
  return boost::make_filter_iterator(IsFace{this}, begin, end);
}

auto CellComplex::faces_end() const -> CellComplex::FaceIterator {
  auto end = boost::vertices(*this).second;
  return boost::make_filter_iterator(IsFace{this}, end, end);
}

auto CellComplex::faces_between_cells_begin() const
    -> CellComplex::FaceBetweenCellIterator {
  auto [begin, end] = boost::vertices(*this);
  return boost::make_filter_iterator(IsFaceBetweenCells{this}, begin, end);
}

auto CellComplex::faces_between_cells_end() const
    -> CellComplex::FaceBetweenCellIterator {
  auto end = boost::vertices(*this).second;
  return boost::make_filter_iterator(IsFaceBetweenCells{this}, end, end);
}

auto CellComplex::cells_begin() const -> CellComplex::CellIterator {
  auto [begin, end] = boost::vertices(*this);
  return boost::make_filter_iterator(IsCell{this}, begin, end);
}

auto CellComplex::cells_end() const -> CellComplex::CellIterator {
  auto end = boost::vertices(*this).second;
  return boost::make_filter_iterator(IsCell{this}, end, end);
}

auto CellComplex::vertices_begin(Vertex f) const
    -> CellComplex::VertexOnFaceIterator {
  auto [begin, end] = boost::adjacent_vertices(f, *this);
  return boost::make_filter_iterator(IsVertex{this}, begin, end);
}

auto CellComplex::vertices_end(Vertex f) const
    -> CellComplex::VertexOnFaceIterator {
  auto end = boost::adjacent_vertices(f, *this).second;
  return boost::make_filter_iterator(IsVertex{this}, end, end);
}

/*
auto CellComplex::cells_begin(Vertex f) const
    -> CellComplex::CellOnFaceIterator {
  auto [begin, end] = boost::adjacent_vertices(f, *this);
  return boost::make_filter_iterator(IsCell{this}, begin, end);
}

auto CellComplex::cells_end(Vertex f) const -> CellComplex::CellOnFaceIterator {
  auto end = boost::adjacent_vertices(f, *this).second;
  return boost::make_filter_iterator(IsCell{this}, end, end);
}
*/

auto CellComplex::faces_begin(Vertex c) const
    -> CellComplex::FaceOnCellIterator {
  auto [begin, end] = boost::adjacent_vertices(c, *this);
  return boost::make_filter_iterator(IsFace{this}, begin, end);
}

auto CellComplex::faces_end(Vertex c) const -> CellComplex::FaceOnCellIterator {
  auto end = boost::adjacent_vertices(c, *this).second;
  return boost::make_filter_iterator(IsFace{this}, end, end);
}

auto CellComplex::get_a(Vertex f) const -> CellComplex::Vertex {
  auto [begin, end] = boost::out_edges(f, *this);
  for (auto it = begin; it != end; ++it)
    if ((*this)[*it].is_main) { // in front of the plane
      auto u = boost::source(*it, *this);
      auto v = boost::target(*it, *this);
      auto c = (f == u) ? v : u;             // opposite vertex
      if ((*this)[c].type != NodeType::Cell) // skip if not a cell
        continue;
      return c;
    }
  auto id = (*this)[f].id;
  throw std::runtime_error(
      fmt::format("Face does not have an adjacent cell on side A", id));
}

auto CellComplex::get_b(Vertex f) const -> CellComplex::Vertex {
  auto [begin, end] = boost::out_edges(f, *this);
  for (auto it = begin; it != end; ++it)
    if (!(*this)[*it].is_main) { // behind of the plane
      auto u = boost::source(*it, *this);
      auto v = boost::target(*it, *this);
      auto c = (f == u) ? v : u;             // opposite vertex
      if ((*this)[c].type != NodeType::Cell) // skip if not a cell
        continue;
      return c;
    }
  auto id = (*this)[f].id;
  throw std::runtime_error(
      fmt::format("Face {} does not have an adjacent cell on side B", id));
}

} // namespace ReUseX
