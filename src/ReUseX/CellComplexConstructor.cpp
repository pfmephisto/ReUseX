// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ReUseX/CellComplex.hpp"
#include "ReUseX/helpers.hpp"

#include <CGAL/Arr_linear_traits_2.h>
#include <CGAL/Arrangement_with_history_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/circulator.h>

#include <range/v3/to_container.hpp>
#include <range/v3/view/transform.hpp>

#include <map>

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using NT = Kernel::FT;

using Plane_3 = Kernel::Plane_3;
using Point_3 = Kernel::Point_3;
using Vector_3 = Kernel::Vector_3;

using Polygon_2 = CGAL::Polygon_2<Kernel>;
using Traits = CGAL::Arr_linear_traits_2<Kernel>;
using Point_2 = Traits::Point_2;
using Line_2 = Traits::Line_2;
using Curve_2 = Traits::Curve_2;
using Direction_2 = Traits::Direction_2;

using Arrangement = CGAL::Arrangement_with_history_2<Traits>;

using Vertex_handle = Arrangement::Vertex_handle;
using Halfedge_handle = Arrangement::Halfedge_handle;
using Face_handle = Arrangement::Face_handle;
using Curve_handle = Arrangement::Curve_handle;

using Vd = ReUseX::CellComplex::Vertex;
using Fd = ReUseX::CellComplex::Vertex;
using Cd = ReUseX::CellComplex::Vertex;

using VertexMap =
    std::unordered_map<Arrangement::Vertex_handle, std::vector<Vd>>;
using FaceMap = std::unordered_map<Arrangement::Face_handle, std::vector<Fd>>;

template <typename Scalar, int Rows>
using EigenVectorContainer =
    std::vector<Eigen::Matrix<Scalar, Rows, 1>,
                Eigen::aligned_allocator<Eigen::Matrix<Scalar, Rows, 1>>>;

namespace ReUseX {
CellComplex::CellComplex(
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>
        &planes_vec,
    std::vector<size_t> &verticals, std::vector<size_t> &horizontals,
    std::vector<std::pair<size_t, size_t>> &pairs,
    std::optional<
        std::function<void(size_t, std::vector<std::array<double, 3>> const &,
                           std::vector<int> const &)>>
        viz_func)
    : n_rooms(0), n_walls(pairs.size()) {

  auto planes = planes_vec | ranges::views::transform([](auto const &p) {
                  return Plane_3(p[0], p[1], p[2], p[3]);
                }) |
                ranges::to<std::vector>();

  auto volumes = this->add_property_map<Cd, double>("c:volume").first;
  auto areas = this->add_property_map<Fd, double>("f:area").first;

  spdlog::trace("Constructing arrangement with {} vertical planes",
                verticals.size());

  Arrangement arr;
  std::unordered_map<Curve_handle, int> ch_map{};
  const Plane_3 ground_plane(0, 0, 1, 0);
  for (auto const id : verticals) {
    auto const &plane = planes[id];

    //  Intersect with ground plane to get line
    auto intersection_1 = CGAL::intersection(ground_plane, plane);
    if (!intersection_1) {
      spdlog::warn("Wall plane does not intersect ground plane");
      continue;
    }
    const auto line_3d = std::get<Kernel::Line_3>(intersection_1.value());
    const Point_2 p(line_3d.point().x(), line_3d.point().y());
    const Direction_2 d(line_3d.direction().dx(), line_3d.direction().dy());
    const Line_2 line_2d(p, d);
    auto ch = insert(arr, (Curve_2)line_2d);
    ch_map[ch] = id;
  }

  // Visualize arrangement
  if (viz_func) {
    spdlog::trace("Visualizing arrangement");
    size_t count = 0;
    for (auto fit = arr.faces_begin(); fit != arr.faces_end(); ++fit, ++count) {
      if (fit->is_unbounded())
        continue;

      std::vector<std::array<double, 3>> pts;
      for (auto vit = fit->outer_ccb(), done = vit;;) {
        auto v = vit->source();
        pts.push_back({CGAL::to_double(v->point().x()),
                       CGAL::to_double(v->point().y()), 0.0});
        if (++vit == done)
          break;
      }

      std::vector<int> indices(pts.size() + 1);
      std::iota(indices.begin(), indices.end(), 0);
      indices.back() = 0,

      viz_func.value()(count, pts, indices);
    }
  }

  auto sorted_floors = horizontals;
  std::sort(sorted_floors.begin(), sorted_floors.end(),
            [&planes](auto a, auto b) {
              const auto &Pa = planes[a];
              const auto &Pb = planes[b];
              const double h1 = -Pa.d() * Pa.c();
              const double h2 = -Pb.d() * Pb.c();
              return h1 < h2;
            });

  // INFO:Looging
  // for (size_t i = 0; i < sorted_floors.size(); ++i) {
  //  const auto id = sorted_floors[i];
  //  const auto plane = planes[id];
  //  auto height = -plane.d() * plane.c();
  //  spdlog::trace("Horizontal plane {} at height {:.3f} ", i, height);
  //}

  VertexMap point_map{};
  spdlog::trace("Initialize vertex map for {} floors", sorted_floors.size());
  for (auto vit = arr.vertices_begin(); vit != arr.vertices_end(); ++vit) {
    point_map[vit] = std::vector<Vd>(sorted_floors.size());
  }
  // Initialize face map
  FaceMap face_map{};
  spdlog::trace("Initialize face map for {} floors", sorted_floors.size());
  for (auto fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
    if (fit->is_unbounded())
      continue;
    face_map[fit] = std::vector<Fd>(sorted_floors.size());
  }

  spdlog::trace("Create vertices");
  for (size_t i = 0; i < sorted_floors.size(); ++i) {
    const auto id = sorted_floors[i];
    const auto plane = planes[id];
    const double height = -plane.d() * plane.c();

    for (auto vit = arr.vertices_begin(); vit != arr.vertices_end(); ++vit)
      point_map[vit][i] = this->add_vertex(
          Eigen::Vector3d(CGAL::to_double(vit->point().x()),
                          CGAL::to_double(vit->point().y()), height));
  }

  // Construct planar faces for each floor
  spdlog::trace("Create  horizontal faces");
  for (auto fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
    if (fit->is_unbounded())
      continue;

    for (size_t i = 0; i < sorted_floors.size(); ++i) {

      // Walk around the outer boundary
      std::vector<Vd> verts{};
      NT cx = 0, cy = 0;
      // Eigen::Vector3d center(0, 0, 0);
      // double cx = 0, cy = 0;
      std::vector<Point_2> pts;

      for (auto vit = fit->outer_ccb(), done = vit;;) {
        auto v = vit->source();
        cx += v->point().x();
        cy += v->point().y();

        verts.push_back(point_map[v][i]);
        // auto pos = (*this)[point_map[v][i]].pos;
        // center += pos;
        pts.emplace_back(v->point().x(), v->point().y());

        if (++vit == done)
          break;
      }
      // center /= static_cast<double>(verts.size());
      cx /= static_cast<NT>(verts.size());
      cy /= static_cast<NT>(verts.size());

      NT area_nt;
      CGAL::area_2(pts.begin(), pts.end(), area_nt);
      double area = CGAL::to_double(area_nt);

      // Set center of face at z = 0, will be updated later
      auto f = this->add_face(
          Eigen::Vector3d(CGAL::to_double(cx), CGAL::to_double(cy), 0.0), verts,
          sorted_floors[i] /* plane id */);
      face_map[fit][i] = f;
      areas[f] = area;
    }
  }

  spdlog::trace("Creating cells");
  for (size_t i = 0; i < sorted_floors.size() - 1; ++i) {
    const auto id1 = sorted_floors[i];
    const auto id2 = sorted_floors[i + 1];
    const auto plane1 = planes[id1];
    const auto plane2 = planes[id2];
    const double h1 = -plane1.d() * plane1.c();
    const double h2 = -plane2.d() * plane2.c();
    const double dist = h2 - h1;
    const double h = (h1 + h2) / 2.0;
    spdlog::trace("Horizontal section thickness is {:.3}", dist);

    std::unordered_map<Arrangement::Halfedge_handle, Fd> face_cache{};

    for (auto fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
      if (fit->is_unbounded())
        continue;
      std::vector<Fd> fc{};
      fc.reserve(10);
      fc.push_back(face_map[fit][i]);
      fc.push_back(face_map[fit][i + 1]);

      // Update z coordinate of horizontal faces
      (*this)[face_map[fit][i]].pos[2] = h1;
      (*this)[face_map[fit][i + 1]].pos[2] = h2;

      NT cx = 0, cy = 0;

      for (auto he : CGAL::Container_from_circulator(fit->outer_ccb())) {

        auto self = he.twin()->twin();
        auto twin = he.twin();
        assert(twin != self);

        cx += he.source()->point().x();
        cy += he.source()->point().y();

        // INFO: Reuse vertical faces
        if (face_cache.contains(self)) {
          fc.push_back(face_cache[self]);
          continue;
        }

        const auto pts = std::array{
            point_map[he.source()][i],
            point_map[he.target()][i],
            point_map[he.target()][i + 1],
            point_map[he.source()][i + 1],
        };

        // Compute center of vertical face
        Eigen::Vector3d center =
            ((*this)[pts[0]].pos + (*this)[pts[1]].pos) / 2.0;
        center[2] = h;

        int plane_id = -1;
        for (auto cit = arr.originating_curves_begin(self);
             cit != arr.originating_curves_end(self); ++cit) {
          plane_id = ch_map[cit];
          break;
        }

        // Create vertical face
        auto f = this->add_face(center, pts, plane_id);

        // Eigen vector length
        areas[f] = dist * ((*this)[pts[0]].pos - (*this)[pts[1]].pos).norm();
        // areas[f] = dist * std::sqrt(CGAL::squared_distance(
        //                       he.source()->point(), he.target()->point()));

        face_cache[twin] = f; // Cache twin halfedge
        fc.push_back(f);
      }

      // Compute center of cell
      cx /= static_cast<NT>(fc.size() - 2);
      cy /= static_cast<NT>(fc.size() - 2);
      Eigen::Vector3d center(CGAL::to_double(cx), CGAL::to_double(cy), h);
      // const auto Fb = face_map[fit][i];
      // auto center = (*this)[Fb].pos;
      // center[2] = h;

      // Create the cell
      auto c = this->add_cell(center, fc);
      volumes[c] = dist * areas[face_map[fit][i]];
    }
  }

  // INFO: Assign wall ids to each cell
  // TODO: Inconsisten use of Plane_3 and Eigen::Vector4d for planes
  spdlog::info("Assigning wall ids to each cell");
  for (size_t i = 0; i < pairs.size(); ++i) {
    const auto &[id1, id2] = pairs[i];
    auto plane1 = planes[id1];
    auto plane2 = planes[id2];

    for (auto cit = this->cells_begin(); cit != this->cells_end(); ++cit) {
      auto p = (*this)[*cit].pos;
      auto dist1 = planePointDist(
          Eigen::Vector4d(plane1.a(), plane1.b(), plane1.c(), plane1.d()), p);
      if (dist1 > 0)
        continue;
      auto dist2 = planePointDist(
          Eigen::Vector4d(plane2.a(), plane2.b(), plane2.c(), plane2.d()), p);
      if (dist2 > 0)
        continue;

      std::get<CellData>((*this)[*cit].data).wall_ids.insert(i);
    }
  }

  // INFO: Set main cell for each face
  spdlog::info("Setting main cell for each face");
  for (auto fit = this->faces_begin(); fit != this->faces_end(); ++fit) {
    // const auto face_id = (*this)[*fit].id;
    const auto plane_id = std::get<FaceData>((*this)[*fit].data).plane_id;

    if (plane_id < 0) {
      spdlog::warn("Face {} has no associated plane", (*this)[*fit].id);
      continue;
    }

    const auto plane = planes[plane_id];
    Eigen::Vector4d plane_vec(plane.a(), plane.b(), plane.c(), plane.d());

    auto [begin, end] = boost::out_edges(*fit, *this);
    for (auto eit = begin; eit != end; ++eit) {

      // Get the adjacent cell
      const auto t = boost::target(*eit, *this);
      const auto s = boost::source(*eit, *this);
      auto c = (*fit == t) ? s : t;
      if ((*this)[c].type != NodeType::Cell)
        continue;

      // spdlog::trace("Face {} is adjacent to cell {}",
      //               (*this)[*fit].id,
      //               (*this)[c].id);

      // const int cell_id = (*this)[c].id;
      // const PointT &center = cell_centers->points[cell_id];
      const auto &center = (*this)[c].pos;

      const auto dist = planePointDist(plane_vec, center);
      (*this)[*eit].is_main = dist >= 0;
      // auto status = (*this)[*eit].is_main
      //                   ? fmt::format(fmt::fg(fmt::rgb(0, 255, 0)), "true")
      //                   : fmt::format(fmt::fg(fmt::rgb(255, 0, 0)),
      //                   "false");
      // spdlog::trace("Face {:<3} cell {:>3} is main {}",
      //               (*this)[*fit].id,
      //               (*this)[c].id, status);
    }
  }
}
} // namespace ReUseX
