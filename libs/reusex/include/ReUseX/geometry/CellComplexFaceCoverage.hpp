// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <ReUseX/core/logging.hpp>
#include <ReUseX/geometry/CellComplex.hpp>
#include <ReUseX/geometry/utils.hpp>
#include <spdmon/spdmon.hpp>

#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/circulator.h>

#include <range/v3/range/concepts.hpp>
#include <range/v3/to_container.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/zip.hpp>

template <class Kernel>
static double
compute_grid_coverage(typename CGAL::Polygon_2<Kernel> const &polygon,
                      std::vector<typename Kernel::Point_2> const &points,
                      const double cell_size = 0.2) {
  // Bounding box
  double min_x = polygon[0].x(), max_x = polygon[0].x();
  double min_y = polygon[0].y(), max_y = polygon[0].y();
  for (auto &p : polygon) {
    min_x = std::min(min_x, p.x());
    max_x = std::max(max_x, p.x());
    min_y = std::min(min_y, p.y());
    max_y = std::max(max_y, p.y());
  }

  int nx = static_cast<int>(std::ceil((max_x - min_x) / cell_size));
  int ny = static_cast<int>(std::ceil((max_y - min_y) / cell_size));

  int covered_cells = 0;
  int total_cells = 0;

  for (int i = 0; i < nx; ++i) {
    for (int j = 0; j < ny; ++j) {
      const double cx = min_x + (i + 0.5) * cell_size;
      const double cy = min_y + (j + 0.5) * cell_size;
      typename Kernel::Point_2 cell_center(cx, cy);
      if (polygon.bounded_side(cell_center) == CGAL::ON_UNBOUNDED_SIDE)
        continue;

      ++total_cells;
      for (auto &pt : points) {
        if (std::abs(pt.x() - cx) <= cell_size / 2 &&
            std::abs(pt.y() - cy) <= cell_size / 2) {
          ++covered_cells;
          break;
        }
      }
    }
  }
  if (total_cells == 0)
    return 0.0;

  return static_cast<double>(covered_cells) / total_cells;
}
namespace ReUseX::geometry {
template <typename PointT>
auto CellComplex::compute_face_coverage(pcl::PointCloud<PointT>::ConstPtr cloud,
                                        EigenVectorContainer<double, 4> &planes,
                                        std::vector<pcl::IndicesPtr> &inliers,
                                        const double grid_size) -> void {

  using EPICK = CGAL::Exact_predicates_inexact_constructions_kernel;

  using Point_3 = EPICK::Point_3;
  using Plane_3 = EPICK::Plane_3;

  using Polygon_2 = CGAL::Polygon_2<EPICK>;
  using Point_2 = Polygon_2::Point_2;

  // INFO: Compute face probabilities
  ReUseX::core::trace("calling compute_face_coverage");
  auto f_sp =
      this->add_property_map<Vertex, double>("f:support_probability").first;

  {
    auto logger =
        spdmon::LoggerProgress("Computing face coverage", this->num_faces());

    std::vector<ReUseX::geometry::CellComplex::Vertex> face_list;
    face_list.reserve(this->num_faces());
    for (auto fit = this->faces_begin(); fit != this->faces_end(); ++fit)
      face_list.push_back(*fit);

#pragma omp parallel for schedule(dynamic)
    for (size_t face_idx = 0; face_idx < face_list.size(); ++face_idx) {
      auto fit = face_list[face_idx];

      // for (size_t face_idx = 0; face_idx < this->num_faces(); ++face_idx) {
      // auto fit = this->faces_begin();

      const int plane_id = std::get<FaceData>((*this)[fit].data).plane_id;

      auto get_comverage = [&](const int id) {
        // if (id < 0) {
        //   ReUseX::core::warn("Face {} has no associated plane",
        //   (*this)[*fit].id); f_sp[*fit] = -1.0;
        //   ++logger;
        //   continue;
        // }

        const auto plane_vec = planes[id];
        const auto indices = inliers[id];

        const Plane_3 plane(plane_vec[0], plane_vec[1], plane_vec[2],
                            plane_vec[3]);

        if (indices->empty())
          return 0.0;

        Polygon_2 polygon{};
        std::transform(this->vertices_begin(fit), this->vertices_end(fit),
                       std::back_inserter(polygon), [&](Vertex v) {
                         const auto pos = (*this)[v].pos;
                         return plane.to_2d(Point_3(pos[0], pos[1], pos[2]));
                       });

        // if (!polygon.is_simple()) {
        //   ReUseX::core::warn("Face {} polygon not is simple",
        //   (*this)[fit].id); f_sp[fit] = -1.0;
        //   ++logger;
        //   continue;
        // }

        const auto inliers = *indices | ranges::views::transform([&](int idx) {
          const PointT &p = cloud->points[idx];
          return plane.to_2d(Point_3(p.x, p.y, p.z));
        }) | ranges::views::filter([&](const Point_2 &p) {
          return polygon.bounded_side(p) == CGAL::ON_BOUNDED_SIDE;
        }) | ranges::to<std::vector>();

        if (inliers.empty())
          return 0.0;

        return compute_grid_coverage<EPICK>(polygon, inliers, grid_size);
      };

      auto val = get_comverage(plane_id);

#pragma omp critical
      f_sp[fit] = val;

      ++logger;
    }
  }
}
} // namespace ReUseX::geometry
