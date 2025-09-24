// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "rux/cellcomplex.hpp"

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>

#include <ReUseX/io.hpp>

#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Iso_cuboid_3.h>
#include <CGAL/Iso_rectangle_2.h>
#include <CGAL/Polygon_2.h>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <fmt/core.h>
#include <fmt/ranges.h>

#include <iostream>
#include <ranges>
#include <string>
#include <vector>

using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using Plane_3 = Kernel::Plane_3;
using Point_3 = Kernel::Point_3;
using Point_2 = Kernel::Point_2;
using Line_3 = Kernel::Line_3;
using Line_2 = Kernel::Line_2;
using Segment_3 = Kernel::Segment_3;
using Segment_2 = Kernel::Segment_2;
using Vector_3 = Kernel::Vector_3;
using Vector_2 = Kernel::Vector_2;
using Direction_3 = Kernel::Direction_3;
using Direction_2 = Kernel::Direction_2;
using Iso_cuboid_3 = Kernel::Iso_cuboid_3;
using Iso_rectangle_2 = Kernel::Iso_rectangle_2;

using Traits_2 = CGAL::Arr_segment_traits_2<Kernel>;
using Arrangement_2 = CGAL::Arrangement_2<Traits_2>;
using Face_handle = Arrangement_2::Face_handle;
using Polygon_2 = CGAL::Polygon_2<Kernel>;

#ifdef CGAL_USE_SCIP
#include <CGAL/SCIP_mixed_integer_program_traits.h>
using MIP_Solver = CGAL::SCIP_mixed_integer_program_traits<double>;
#elif defined(CGAL_USE_GLPK)
#include <CGAL/GLPK_mixed_integer_program_traits.h>
using MIP_Solver = CGAL::GLPK_mixed_integer_program_traits<double>;
#endif

typedef typename MIP_Solver::Variable Variable;
typedef typename MIP_Solver::Linear_objective Linear_objective;
typedef typename MIP_Solver::Linear_constraint Linear_constraint;

using PointT = pcl::PointXYZRGB;
using Cloud = pcl::PointCloud<PointT>;
using CloudPtr = Cloud::Ptr;

namespace fs = std::filesystem;

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

  sub->add_flag("-d, --display", opt->display,
                "Display the result in a PCL visualizer.")
      ->default_val(opt->display);

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

  pcl::visualization::PCLVisualizer::Ptr viewer;
  if (opt.display) {
    viewer = pcl::visualization::PCLVisualizer::Ptr(
        new pcl::visualization::PCLVisualizer("MCL Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
  }

  fs::path input_path = opt.input;

  fs::path cloud_path = input_path.replace_extension(".pcd");
  fs::path plane_path = input_path.replace_extension(".planes");

  if (!fs::exists(cloud_path)) {
    spdlog::error("Input cloud file does not exist: {}", cloud_path.string());
    return 1;
  }
  if (!fs::exists(plane_path)) {
    spdlog::error("Input plane file does not exist: {}", plane_path.string());
    return 1;
  }

  CloudPtr cloud(new Cloud);
  pcl::io::loadPCDFile(cloud_path.string(), *cloud);
  if (viewer) {
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    viewer->addPointCloud<PointT>(cloud, rgb, "input cloud");
  }

  std::vector<pcl::ModelCoefficients> planes = {};
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>
      centroids;
  std::vector<std::shared_ptr<pcl::Indices>> inliers = {};
  ReUseX::read(plane_path, planes, centroids, inliers);

  spdlog::trace("Number of planes read: {}", planes.size());

  std::vector<Plane_3> walls;
  std::vector<Plane_3> floors;
  std::vector<Plane_3> ceilings;

  const Vector_3 up(0, 0, 1);

  for (size_t i = 0; i < planes.size(); ++i) {
    const auto &p = planes[i];
    spdlog::trace("Plane {:>3}: [{:>3}]", i, fmt::join(p.values, ","));

    auto plane = Plane_3(p.values[0], p.values[1], p.values[2], p.values[3]);
    auto val =
        CGAL::to_double(CGAL::scalar_product(plane.orthogonal_vector(), up));
    if (std::abs(val) < 1e-6) {
      walls.push_back(plane);
    } else if (val > 0.5) {
      floors.push_back(plane);
    } else if (val < -0.5) {
      ceilings.push_back(plane);
    } else {
      const auto normal = plane.orthogonal_vector();
      spdlog::warn("Unknown plane orientation");
      std::cout << "Normal: " << normal << std::endl;
    }
  }

  spdlog::debug("Number of walls:    {:>3}", walls.size());
  spdlog::debug("Number of floors:   {:>3}", floors.size());
  spdlog::debug("Number of ceilings: {:>3}", ceilings.size());

  PointT min_pt, max_pt;
  pcl::getMinMax3D(*cloud, min_pt, max_pt);

  // TODO: There is no need to use a cuboid, a rectangle would suffice
  Iso_cuboid_3 bbox(Point_3(min_pt.x, min_pt.y, min_pt.z),
                    Point_3(max_pt.x, max_pt.y, max_pt.z));

  Arrangement_2 arr;
  for (const auto &plane : walls) {

    const Plane_3 cut_plane(bbox.min(), up);

    // Create a line segment representation of the plane
    const auto intersection = CGAL::intersection(plane, cut_plane);
    if (!intersection.has_value() ||
        !holds_alternative<Line_3>(intersection.value())) {
      spdlog::warn("No intersection between planes");
      continue;
    }
    const Line_3 line_3d = get<Line_3>(intersection.value());
    const auto intersection_2 = CGAL::intersection(bbox, line_3d);

    if (!intersection_2.has_value() ||
        !holds_alternative<Segment_3>(intersection_2.value())) {
      spdlog::warn("Line does not intersect bounding box");
      std::cout << "Line: " << line_3d << std::endl;
      std::cout << "Box: " << bbox << std::endl;
      continue;
    }
    const Segment_3 line_segment_3d = get<Segment_3>(intersection_2.value());
    Point_2 p1, p2;
    p1 = Point_2(line_segment_3d.source().x(), line_segment_3d.source().y());
    p2 = Point_2(line_segment_3d.target().x(), line_segment_3d.target().y());
    const Segment_2 segment(p1, p2);

    // Insert the plane into the arrangement
    CGAL::insert(arr, segment);
  }

  spdlog::debug("Number of arrangement faces: {}", arr.number_of_faces());

  /*
  int cell_count = 0;
  for (auto fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
    Face_handle face = fit;

    if (!face->is_unbounded()) {
      Polygon_2 poly;

      // Iterate over outer boundary of the face
      auto ccb = face->outer_ccb();
      auto he = ccb;
      do {
        poly.push_back(he->source()->point());
        he = he->next();
      } while (he != ccb);

      std::vector<std::string> tags{};
      std::transform(poly.begin(), poly.end(), std::back_inserter(tags),
                     [](auto const &p) {
                       return fmt::format("({:>2}, {:>2})",
                                          CGAL::to_double(p.x()),
                                          CGAL::to_double(p.y()));
                     });
      spdlog::trace("Cell {:>2}: {:>9}", cell_count, fmt::join(tags, " - "));
      ++cell_count;
    }
  }
  spdlog::info("Cell count: {}", cell_count);
  */

  std::vector<Plane_3> selected_floors{};
  const double height_threshold = 0.1;
  double min_z = std::numeric_limits<double>::max();
  std::sort(floors.begin(), floors.end(),
            [](const Plane_3 &a, const Plane_3 &b) {
              return CGAL::to_double(a.d()) < CGAL::to_double(b.d());
            });
  for (const auto &floor : floors)
    if ((CGAL::to_double(floor.d()) + height_threshold) < min_z) {
      selected_floors.push_back(floor);
      min_z = CGAL::to_double(floor.d());
    }
  spdlog::debug("Number of selected floors:   {:>3}", selected_floors.size());

  std::vector<Plane_3> selected_ceilings{};
  double max_z = std::numeric_limits<double>::lowest();
  std::sort(ceilings.begin(), ceilings.end(),
            [](const Plane_3 &a, const Plane_3 &b) {
              return CGAL::to_double(a.d()) > CGAL::to_double(b.d());
            });
  for (const auto &ceiling : ceilings)
    if ((CGAL::to_double(ceiling.d()) - height_threshold) > max_z) {
      selected_ceilings.push_back(ceiling);
      max_z = CGAL::to_double(ceiling.d());
    }
  std::reverse(selected_ceilings.begin(), selected_ceilings.end());
  spdlog::debug("Number of selected ceilings: {:>3}", selected_ceilings.size());

  if (viewer) {
    std::vector<CloudPtr> polygons{};
    for (auto fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
      Face_handle face = fit;

      if (!face->is_unbounded()) {
        polygons.push_back(CloudPtr(new Cloud));

        // Iterate over outer boundary of the face
        auto ccb = face->outer_ccb();
        auto he = ccb;
        do {
          PointT p;
          p.x = CGAL::to_double(he->source()->point().x());
          p.y = CGAL::to_double(he->source()->point().y());
          p.z = min_pt.z; // Place the polygon at the bottom of the bbox
          p.r = 255;
          p.g = 255;
          p.b = 255;
          polygons.back()->points.push_back(p);
          he = he->next();
        } while (he != ccb);
      }
    }

    for (size_t i = 0; i < polygons.size(); ++i) {
      const auto name = "polygon_" + std::to_string(i);
      const auto color = pcl::GlasbeyLUT::at(i);
      pcl::Vertices indices{};
      indices.vertices.resize(polygons[i]->points.size());
      std::iota(indices.vertices.begin(), indices.vertices.end(), 0);
      indices.vertices.push_back(0); // Close the polygon
      // std::views::iota(0, polygons[i]->points.size())

      // viewer->addPolygon<PointT>(polygons[i],
      //                            static_cast<double>(color.r) / 254.0,
      //                            static_cast<double>(color.g) / 254.0,
      //                            static_cast<double>(color.b) / 254.0, name);
      viewer->addPolygonMesh<PointT>(polygons[i], {indices}, name);
      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR,
          static_cast<double>(color.r) / 254.0,
          static_cast<double>(color.g) / 254.0,
          static_cast<double>(color.b) / 254.0, name);
    }
  }

  // TODO: Create solver
  // - Variables
  // - Constraints
  //   - Constraints 1: Each cell c must be assigned exactly one label from Ro
  //   - Constraints 2: At boundary faces of room interiors, the room label may
  //   only occur on the positive side of the face
  //   - Constraints 3: Wall labels may only occur in cells which are assigned
  //   the outside label
  //   - Constraints 4: The boundary faces of room interiors must also be the
  //   boundary faces of an active wall
  //   - Constraints 5: At wall boundaries that are inner faces, the wall label
  //   must be on the negative side of the respective faces
  //   - Constraints 6: A wall may end at an inner face only if this face is a
  //   boundary face of at least one other active wall

  MIP_Solver solver;

  // Variable x1
  Variable *x1 = solver.create_variable(Variable::CONTINUOUS, 0, 40, "x1");

  // Variable x2
  // You can create first (using default parameters) and then assign values.
  Variable *x2 = solver.create_variable();
  x2->set_name("x2"); // This is optional (a default will be given)

  // Variable x3
  Variable *x3 = solver.create_variable(); // Uses all default parameters
  x3->set_name("x3");

  // Variable x4
  Variable *x4 = solver.create_variable(Variable::INTEGER, 2, 3, "x4");

  // Objective.
  // Be careful this is "MAXIMIZE"
  Linear_objective *obj = solver.create_objective(Linear_objective::MAXIMIZE);
  obj->add_coefficient(x1, 1.0);
  obj->add_coefficient(x2, 2.0);
  obj->add_coefficient(x3, 3.0);
  obj->add_coefficient(x4, 1.0);

  // Constraint c1: -x1 + x2 + x3 + 10 x4 <= 20
  Linear_constraint *c1 =
      solver.create_constraint(-Linear_constraint::infinity(), 20, "c1");
  c1->add_coefficient(x1, -1);
  c1->add_coefficient(x2, 1);
  c1->add_coefficient(x3, 1);
  c1->add_coefficient(x4, 10);

  // Constraint c2: x1 - 3 x2 + x3 <= 30
  Linear_constraint *c2 =
      solver.create_constraint(-Linear_constraint::infinity(), 30, "c2");
  c2->add_coefficient(x1, 1);
  c2->add_coefficient(x2, -3);
  c2->add_coefficient(x3, 1);

  // Constraint c3:  x2 - 3.5 x4 = 0
  Linear_constraint *c3 = solver.create_constraint(0, 0, "c3");
  c3->add_coefficient(x2, 1);
  c3->add_coefficient(x4, -3.5);

  // Solve
  if (!solver.solve()) {
    spdlog::error("Solving problem failed");
    return EXIT_FAILURE;
  }

  spdlog::info("Result: ");
  const std::vector<double> &results = solver.solution();
  for (std::size_t i = 0; i < results.size(); ++i)
    spdlog::info("\tx{}: {}", i + 1, results[i]);

  if (viewer)
    while (!viewer->wasStopped()) {
      viewer->spinOnce(100);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

  return 0;
}
