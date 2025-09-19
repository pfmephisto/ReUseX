// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "rux/cellcomplex.hpp"

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>

#include <ReUseX/io.hpp>

#include <CGAL/Exact_integer.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Extended_homogeneous.h>
#include <CGAL/Homogeneous.h>
#include <CGAL/IO/Nef_polyhedron_iostream_3.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Polyhedron_3.h>

#ifdef CGAL_USE_BASIC_VIEWER
#include <CGAL/draw_nef_3.h>
#endif

namespace fs = std::filesystem;

// using Kernel = CGAL::Exact_predicates_exact_constructions_kernel; // 0
// volumes
// using Kernel =
//     CGAL::Extended_homogeneous<CGAL::Exact_integer>; // add_corners < 2
//                                                        // assertion violtion
using Kernel = CGAL::Homogeneous<CGAL::Exact_integer>; // Constructor not
// available for Nef_polyhedron_3 and Plane_3
using Nef_polyhedron = CGAL::Nef_polyhedron_3<Kernel>;
using Polyhedron = CGAL::Polyhedron_3<Kernel>;
using RT = Kernel::RT;
using Plane_3 = Nef_polyhedron::Plane_3;
using Point_3 = Kernel::Point_3;
// using Iso_cuboid_3 = Kernel::Iso_cuboid_3;

using Volume_const_iterator = Nef_polyhedron::Volume_const_iterator;
using Shell_entry_const_iterator = Nef_polyhedron::Shell_entry_const_iterator;
using SFace_const_iterator = Nef_polyhedron::SFace_const_iterator;
using SFace_cycle_const_iterator = Nef_polyhedron::SFace_cycle_const_iterator;
using SHalfedge_const_iterator = Nef_polyhedron::SHalfedge_const_iterator;
using SVertex_const_iterator = Nef_polyhedron::SVertex_const_iterator;

using Vertex_const_handle = Nef_polyhedron::Vertex_const_handle;
using Halfedge_const_handle = Nef_polyhedron::Halfedge_const_handle;
using Halffacet_const_handle = Nef_polyhedron::Halffacet_const_handle;
using SHalfedge_const_handle = Nef_polyhedron::SHalfedge_const_handle;
using SHalfloop_const_handle = Nef_polyhedron::SHalfloop_const_handle;
using SFace_const_handle = Nef_polyhedron::SFace_const_handle;
using Volume_const_iterator = Nef_polyhedron::Volume_const_iterator;
using Shell_entry_const_iterator = Nef_polyhedron::Shell_entry_const_iterator;

class Shell_explorer {
  bool first;
  Vertex_const_handle v_min;

    public:
  Shell_explorer() : first(true) {}

  void visit(Vertex_const_handle v) {
    if (first ||
        CGAL::lexicographically_xyz_smaller(v->point(), v_min->point())) {
      v_min = v;
      first = false;
    }
  }

  void visit(Halfedge_const_handle) {}
  void visit(Halffacet_const_handle) {}
  void visit(SHalfedge_const_handle) {}
  void visit(SHalfloop_const_handle) {}
  void visit(SFace_const_handle) {}

  Vertex_const_handle &minimal_vertex() { return v_min; }
  void reset_minimal_vertex() { first = true; }
};

void setup_subcommand_cellcomplex(CLI::App &app) {

  auto opt = std::make_shared<SubcommandCellcomplexOptions>();
  auto *sub = app.add_subcommand(
      "cellcomplex",
      "This tool computes the best fit volumes form a set of input planes.");

  sub->add_option("input", opt->input, "Path to the input file.")
      ->required()
      ->check(CLI::ExistingFile);
  sub->add_option("output", opt->output, "Path to the output file.")
      ->default_val(opt->output);

  sub->callback([opt]() {
    spdlog::trace("calling run_subcommand_cellcomplex");
    return run_subcommand_cellcomplex(*opt);
  });
};

int run_subcommand_cellcomplex(SubcommandCellcomplexOptions const &opt) {

  // TODO: Make sure the input planse are cleaned
  // - Horizontal and vertical snapping
  // - Merge plance that are close and almost parallel
  // - optinally apply shape regularization to enforce orthogonality
  //   or set angles
  //   [link](https://doc.cgal.org/latest/Shape_regularization/index.html#Chapter_Shape_Regularization)

  spdlog::trace("Reading input file: {}", opt.input.string());
  std::vector<pcl::ModelCoefficients> planes = {};
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>
      centroids;
  std::vector<std::shared_ptr<pcl::Indices>> inliers = {};
  ReUseX::read(opt.input.string(), planes, centroids, inliers);

  spdlog::trace("Number of planes read: {}", planes.size());

  Nef_polyhedron space(Nef_polyhedron::COMPLETE); // EMPTY, COMPLETE

  spdlog::trace("Building Nef polyhedron from planes");
  for (size_t i = 0; i < planes.size(); ++i) {
    const auto &p = planes[i];
    // space = space.intersection(Nef_polyhedron(
    //     Plane_3(p.values[0], p.values[1], p.values[2], p.values[3]),
    //     Nef_polyhedron::INCLUDED));

    // space = space.intersection(Nef_polyhedron(
    //     Plane_3(p.values[0], p.values[1], p.values[2], p.values[3]),
    //     Nef_polyhedron::EXCLUDED));

    space = space.intersection(Plane_3(RT(p.values[0]), RT(p.values[1]),
                                       RT(p.values[2]), RT(p.values[3])),
                               Nef_polyhedron::Intersection_mode::PLANE_ONLY);
  }
// Nef_polyhedron::Intersection_mode::PLANE_ONLY
// CLOSED_HALFSPACE
// OPEN_HALFSPACE
// PLANE_ONLY
#ifdef CGAL_USE_BASIC_VIEWER
  CGAL::draw(space);
#endif

  int ic = 0;
  Volume_const_iterator c;
  Shell_explorer SE;
  CGAL_forall_volumes(c, space) {
    std::cout << "Volume " << ic++ << std::endl;
    int is = 0;
    Shell_entry_const_iterator it;
    CGAL_forall_shells_of(it, c) {
      SE.reset_minimal_vertex();
      space.visit_shell_objects(SFace_const_handle(it), SE);
      Point_3 p(SE.minimal_vertex()->point());
      std::cout << "  minimal vertex of shell " << is++ << " is at " << p
                << std::endl;
    }
  }

  return 0;
}
