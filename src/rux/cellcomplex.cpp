// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "rux/cellcomplex.hpp"

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>

#include <fmt/std.h>

#include <ReUseX/CellComplex.hpp>
#include <ReUseX/Solidifier.hpp>
#include <ReUseX/io.hpp>

#include <CGAL/Arr_non_caching_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_2_algorithms.h>

#include <pcl/common/colors.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <embree4/rtcore.h>

#include <algorithm>
#include <filesystem>
#include <ranges>

/*
#include <pcl/common/pca.h>
// #include <pcl/correspondence.h>
// #include <pcl/filters/filter.h>
// #include <pcl/filters/uniform_sampling.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
*/

namespace fs = std::filesystem;
namespace views = std::views;

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Number_type = Kernel::FT;

using Traits = CGAL::Arr_non_caching_segment_traits_2<Kernel>;
using Point_2 = Traits::Point_2;
using Line_2 = Traits::Line_2;
using Direction_2 = Traits::Direction_2;
using Segment = Traits::X_monotone_curve_2;

// The map-extended dcel vertex.
template <typename Point_2>
class Arr_map_vertex : public CGAL::Arr_vertex_base<Point_2> {
    public:
  // std::string name, type;
};

// The map-extended dcel halfedge.
template <typename X_monotone_curve_2>
class Arr_map_halfedge : public CGAL::Arr_halfedge_base<X_monotone_curve_2> {
    public:
  int source_id = -1;
  // std::string name, type;
};

// The map-extended dcel face.
class Arr_map_face : public CGAL::Arr_face_base {
    public:
  // std::string name, type;
};

// The map-extended dcel.
template <typename Traits>
class DCEL
    : public CGAL::Arr_dcel_base<
          Arr_map_vertex<typename Traits::Point_2>,
          Arr_map_halfedge<typename Traits::X_monotone_curve_2>, Arr_map_face> {
};

using Arrangement = CGAL::Arrangement_2<Traits, DCEL<Traits>>;
// using Arrangement = CGAL::Arrangement_2<Traits>;

using Vertex_handle = Arrangement::Vertex_handle;
using Halfedge_handle = Arrangement::Halfedge_handle;
using Face_handle = Arrangement::Face_handle;

using Plane_3 = Kernel::Plane_3;
using Point_3 = Kernel::Point_3;
using Vector_3 = Kernel::Vector_3;
using Iso_cuboid_3 = Kernel::Iso_cuboid_3;
using Iso_rectangle_2 = CGAL::Iso_rectangle_2<Kernel>;
using Polygon_2 = CGAL::Polygon_2<Kernel>;

using Indices = pcl::Indices;
using IndicesPtr = pcl::IndicesPtr;
using IndicesConstPtr = pcl::IndicesConstPtr;

using PointT = pcl::PointXYZRGBL;
using NormalT = pcl::Normal;
using LabelT = pcl::PointXYZRGBL;

using Cloud = pcl::PointCloud<PointT>;
using CloudPtr = typename Cloud::Ptr;
using CloudConstPtr = typename Cloud::ConstPtr;

using CloudN = pcl::PointCloud<NormalT>;
using CloudNPtr = typename CloudN::Ptr;
using CloudNConstPtr = typename CloudN::ConstPtr;

using CloudL = pcl::PointCloud<LabelT>;
using CloudLPtr = typename CloudL::Ptr;
using CloudLConstPtr = typename CloudL::ConstPtr;

using Plane_3Ptr = std::shared_ptr<Plane_3>;
using PlaneIndicesPair = std::pair<Plane_3Ptr, IndicesPtr>;
using PlaneIndicesPairs = std::vector<PlaneIndicesPair>;

using PCA = pcl::PCA<PointT>;

using Planes = std::vector<std::tuple<Plane_3Ptr, IndicesPtr, Eigen::Vector4f>>;

using RTCVertex = struct RTCVertex {
  float x, y, z, r; // x, y, z coordinates and radius
};

using RTCNormal = struct RTCNormal {
  float x, y, z; // x, y, z components of the normal vector
};

namespace collect_impl {
template <typename Container>
struct collect /*: std::range_adaptor_closure<collect<Container>>*/ {
  auto operator()(std::ranges::input_range auto &&range) const {
    auto r = range | std::views::common;
    return Container(std::ranges::begin(r), std::ranges::end(r));
  }

  friend auto operator|(std::ranges::input_range auto &&range, collect c) {
    return c(range);
  }
};

template <template <typename...> typename Template, typename... T>
concept well_formed_template = requires() {
  typename Template<T...>;
};

template <template <typename...> typename Container>
struct auto_collect /*: std::range_adaptor_closure<auto_collect<Container>>*/
{
  template <std::ranges::input_range Rng> auto operator()(Rng &&range) const {
    auto r = range | std::views::common;
    using element_t = std::ranges::range_value_t<Rng>;
    if constexpr (well_formed_template<Container, element_t>) {
      return Container<element_t>(std::ranges::begin(r), std::ranges::end(r));
    } else {
      using first_t = decltype(std::declval<element_t>().first);
      using second_t = decltype(std::declval<element_t>().second);
      static_assert(well_formed_template<Container, first_t, second_t>);
      return Container<first_t, second_t>(std::ranges::begin(r),
                                          std::ranges::end(r));
    }
  }

  friend auto operator|(std::ranges::input_range auto &&range, auto_collect c) {
    return c(range);
  }
};
} // namespace collect_impl

template <template <typename...> typename Container>
inline constexpr auto collect() {
  return collect_impl::auto_collect<Container>{};
}
template <typename Container> inline constexpr auto collect() {
  return collect_impl::collect<Container>{};
}

auto separate_planes(const Planes &planes,
                     const Vector_3 &up = Vector_3(0, 0, 1),
                     const double epsilon = 0.1) {
  spdlog::trace("Separate planes into vertical and horizontal planes");

  // using value_type = Planes::value_type;

  std::vector<size_t> vertical{};
  std::vector<size_t> horizontal{};

  for (size_t i = 0; i < planes.size(); ++i) {
    auto plane = std::get<0>(planes[i]);

    const Vector_3 normal = plane->orthogonal_vector();
    const double scalar = CGAL::to_double(normal * up);

    if (std::abs(scalar) < epsilon) // Wall
      vertical.push_back(i);
    else if (std::abs(scalar - 1.0) < epsilon) // Floor
      horizontal.push_back(i);
    else if (std::abs(scalar + 1.0) < epsilon) // Ceiling
      horizontal.push_back(i);
    else
      spdlog::warn("Plane {} is not vertical or horizontal", i);
  }
  return std::make_tuple(vertical, horizontal);
};

auto dist_to_plane(const Eigen::Vector4f &plane, const Eigen::Vector3f &p) {
  const auto n = plane.head<3>();
  const auto d = plane[3];
  return (n.dot(p) + d) / n.squaredNorm(); // signed distance / |n|^2
}

auto make_pairs(Planes &planes, const double threshold = 0.6,
                const double new_plane_offset = 0.5) {
  spdlog::trace("Find plane pairs with threshold {} and add new planes at a "
                "distance of {}",
                threshold, new_plane_offset);

  // using value_type = Planes::value_type;

  std::vector<std::tuple<size_t, size_t>> pairs{};
  std::vector<bool> paired(planes.size(), false);

  for (size_t i = 0; i < planes.size(); ++i) {
    // Skip if already paired
    if (paired[i])
      continue;

    auto const &plane_i_ = std::get<0>(planes[i]);
    const Eigen::Vector4f plane_i(plane_i_->a(), plane_i_->b(), plane_i_->c(),
                                  plane_i_->d());

    auto candidates =
        std::views::iota(i + 1, planes.size()) // indices after i
        | std::views::transform([&](size_t j) {
            // INFO: Compute plane angle
            auto const &plane_j_ = std::get<0>(planes[j]);
            const Eigen::Vector4f plane_j(plane_j_->a(), plane_j_->b(),
                                          plane_j_->c(), plane_j_->d());
            const double scalar = plane_i.head<3>().dot(plane_j.head<3>());
            return std::pair{j, scalar};
          }) |
        views::filter([](auto p) {
          return std::abs(p.second + 1.0) < 0.1; // Opposite normals
        }) |
        views::transform([&](auto p) {
          const size_t j = p.first;

          auto const &plane_j_ = std::get<0>(planes[j]);
          const Eigen::Vector4f plane_j(plane_j_->a(), plane_j_->b(),
                                        plane_j_->c(), plane_j_->d());

          // Dist distance c_i to plane j
          const auto c_i = std::get<2>(planes[i]);
          auto const dist_j = dist_to_plane(plane_j, c_i.head<3>());

          // Dist distance c_j to plane i
          const auto c_j = std::get<2>(planes[j]);
          auto const dist_i = dist_to_plane(plane_i, c_j.head<3>());

          const auto diff = (std::abs(dist_i) + std::abs(dist_j)) / 2.0;

          return std::pair{j, diff};
        }) |
        views::filter([&threshold](auto p) { return p.second < threshold; }) |
        collect<std::vector>();

    std::ranges::sort(candidates, {}, &std::pair<size_t, double>::second);

    if (candidates.size() >= 1) {
      pairs.push_back(std::make_tuple(i, candidates[0].first));
      paired[candidates[0].first] = true;
    } else {
      Eigen::Vector4f new_plane = -plane_i;
      new_plane[3] -= new_plane_offset;
      Eigen::Vector3f p = std::get<2>(planes[i]).head<3>();
      float dist = dist_to_plane(new_plane, p);
      Eigen::Vector4f centroid = Eigen::Vector4f::Zero();
      centroid.head<3>() = p - dist * new_plane.head<3>();

      planes.emplace_back(std::make_shared<Plane_3>(new_plane[0], new_plane[1],
                                                    new_plane[2], new_plane[3]),
                          IndicesPtr(new Indices{}), centroid);
      pairs.push_back(std::make_tuple(i, planes.size() - 1));
      paired.push_back(true);
    }
  }

  return pairs;
}

size_t compute_number_of_inliers(Eigen::Vector4f const plane,
                                 CloudConstPtr cloud, IndicesConstPtr indices,
                                 const float threshold = 0.2) {
  const Eigen::Vector3f n = plane.head<3>();
  const float d = plane[3];
  // If normal not guaranteed normalized, uncomment:
  // const float invNorm = 1.0f / n.norm();
  // const float scaledThreshold = threshold * invNorm;

  return std::ranges::count_if(*indices, [&](int idx) {
    const auto &p = cloud->points[idx].getVector3fMap(); // directly 3f map
    float dist = n.dot(p) + d;
    return std::abs(dist) < threshold; // or scaledThreshold
  });
}

Planes merge_plaes(Planes &planes, CloudPtr const cloud,
                   const double angle_threshold = 0.1,
                   const double distance_threshold = 0.5,
                   const double min_overlap = 0.8) {
  spdlog::trace(
      "Merge planes with angle threshold {} and distance threshold {}",
      angle_threshold, distance_threshold);

  PCA pca;
  pca.setInputCloud(cloud);

  Planes merged_planes{};
  for (size_t i = 0; i < planes.size(); ++i) {
    auto const &plane_i_ = std::get<0>(planes[i]);
    auto const &plane_i = Eigen::Vector4f(plane_i_->a(), plane_i_->b(),
                                          plane_i_->c(), plane_i_->d());
    auto const &inliers_i = std::get<1>(planes[i]);

    bool match = false;

    for (size_t j = 0; j < merged_planes.size(); ++j) {
      auto const &plane_j_ = std::get<0>(merged_planes[j]);
      auto const &plane_j = Eigen::Vector4f(plane_j_->a(), plane_j_->b(),
                                            plane_j_->c(), plane_j_->d());
      auto const &inliers_j = std::get<1>(merged_planes[j]);

      if (std::abs(plane_i.head<3>().dot(plane_j.head<3>()) - 1.0) >
          angle_threshold)
        continue; // Not same orientation

      size_t overlap_j = compute_number_of_inliers(plane_i, cloud, inliers_j,
                                                   distance_threshold);

      size_t overlap_i = compute_number_of_inliers(plane_j, cloud, inliers_i,
                                                   distance_threshold);

      const double overlap =
          static_cast<double>(overlap_i + overlap_j) /
          static_cast<double>(inliers_i->size() + inliers_j->size());

      if (overlap < min_overlap)
        continue; // Not enough inlier overlap

      IndicesPtr merged_inliers(new Indices);
      merged_inliers->reserve(inliers_i->size() + inliers_j->size());
      merged_inliers->insert(merged_inliers->end(), inliers_i->begin(),
                             inliers_i->end());
      merged_inliers->insert(merged_inliers->end(), inliers_j->begin(),
                             inliers_j->end());
      pca.setIndices(merged_inliers);
      Eigen::Vector3f normal = pca.getEigenVectors().col(2);

      if (normal.dot(plane_i.head<3>()) < 0)
        normal = -normal;

      Eigen::Vector4f merged_centroid = pca.getMean();
      Plane_3Ptr merged_plane =
          std::make_shared<Plane_3>(normal[0], normal[1], normal[2],
                                    -normal.dot(merged_centroid.head<3>()));

      merged_planes[j] =
          std::make_tuple(merged_plane, merged_inliers, merged_centroid);
      match = true;
      break;
    }
    if (!match)
      merged_planes.push_back(planes[i]);
  }

  return merged_planes;
}

Planes force_orthogonal_planes(Planes &planes, const double threshold = 0.1,
                               const Vector_3 &up = Vector_3(0, 0, 1)) {
  spdlog::trace("Force planes to be orthogonal");

  for (auto &plane_tuple : planes) {
    auto &plane = std::get<0>(plane_tuple);
    // auto &centroid = std::get<2>(plane_tuple);

    auto normal = plane->orthogonal_vector();
    auto scalar = CGAL::to_double(normal * up);

    if (std::abs(scalar) < 0.1) {
      // Wall
      Vector_3 new_normal = normal - (normal * up) * up;
      new_normal = new_normal / std::sqrt(new_normal.squared_length());
      plane = std::make_shared<Plane_3>(new_normal.x(), new_normal.y(),
                                        new_normal.z(), plane->d());
    } else if (std::abs(scalar - 1.0) < threshold) {
      // Floor
      plane = std::make_shared<Plane_3>(up.x(), up.y(), up.z(), plane->d());
    } else if (std::abs(scalar + 1.0) < threshold) {
      // Ceiling
      plane = std::make_shared<Plane_3>(-up.x(), -up.y(), -up.z(), plane->d());
    } else {
      spdlog::warn("Plane with normal ({}, {}, {}) is not vertical or "
                   "horizontal",
                   normal.x(), normal.y(), normal.z());
    }
  }

  return planes;
}

double compute_grid_coverage(const Polygon_2 &polygon,
                             const std::vector<Point_2> &points,
                             double cell_size = 0.2) {
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
      Point_2 cell_center(cx, cy);
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

  return static_cast<double>(covered_cells) / total_cells;
}

// Spherical Fibonacci
std::vector<Eigen::Vector3d> sampleSphericalFibonacci(size_t N) {
  std::vector<Eigen::Vector3d> out;
  out.reserve(N);
  if (N == 0)
    return out;
  constexpr double golden_angle =
      M_PI * (3.0 - std::sqrt(5.0)); // ~2.3999632297

  for (size_t i = 0; i < N; ++i) {
    double t = (double)i;
    double z = 1.0 - 2.0 * t / (double)(N - 1); // maps to [1, -1] if N>1
    double phi = golden_angle * t;
    double r = std::sqrt(std::max(0.0, 1.0 - z * z));
    double x = std::cos(phi) * r;
    double y = std::sin(phi) * r;
    out.emplace_back(x, y, z);
  }
  return out;
}

void setup_subcommand_cellcomplex(CLI::App &app) {

  auto opt = std::make_shared<SubcommandCellcomplexOptions>();
  auto *sub = app.add_subcommand(
      "cellcomplex",
      "This tool computes the best fit volumes form a set of input planes.");

  sub->add_option("input", opt->input, "Path to the input file.")
      ->required()
      ->check(CLI::ExistingFile);
  sub->add_option("labels", opt->labels_path, "Path to the input label file.")
      ->required()
      ->check(CLI::ExistingFile); // ->default_val(opt->labels);
  sub->add_option("output", opt->output, "Path to the output file.")
      ->default_val(opt->output);

  sub->add_flag("-d, --visualize", opt->display, "Display the result.")
      ->default_val(opt->display);

  sub->callback([opt]() {
    spdlog::trace("calling run_subcommand_cellcomplex");
    return run_subcommand_cellcomplex(*opt);
  });
};

int run_subcommand_cellcomplex(SubcommandCellcomplexOptions const &opt) {

  // TODO: Make sure the input planse are cleaned
  // - optinally apply shape regularization to enforce orthogonality
  //   [link](https://doc.cgal.org/latest/Shape_regularization/index.html#Chapter_Shape_Regularization)

  pcl::visualization::PCLVisualizer::Ptr viewer;
  if (opt.display) {
    viewer = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
  }

  fs::path cloud_path = fs::path(opt.input).replace_extension(".pcd");
  fs::path planes_path = fs::path(opt.input).replace_extension(".planes");

  assert(fs::exists(cloud_path) && "Input point cloud file does not exist");
  assert(fs::exists(planes_path) && "Input planes file does not exist");
  assert(fs::exists(opt.labels_path) && "Input labels file does not exist");

  CloudPtr cloud(new Cloud);

  spdlog::trace("Reading input file: {}", cloud_path);
  pcl::io::load<PointT>(cloud_path.string(), *cloud);

  if (opt.display) {
    spdlog::trace("Displaying point cloud with {} points", cloud->size());
    const std::string cloud_name = "cloud";
    viewer->addPointCloud<PointT>(cloud, cloud_name);
  }

  CloudLPtr room_labels(new CloudL);
  pcl::io::load<LabelT>(opt.labels_path.string(), *room_labels);
  if (room_labels->size() != cloud->size()) {
    spdlog::error("Labels size does not match point cloud size");
    return EXIT_FAILURE;
  }

  spdlog::trace("Reading input file: {}", planes_path);
  // TODO: Rewrite reader function to be more portable
  std::vector<pcl::ModelCoefficients> planes_coeff = {};
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>
      centroids;
  std::vector<IndicesPtr> inliers = {};
  ReUseX::read(planes_path, planes_coeff, centroids, inliers);

  Planes planes = {};
  for (size_t i = 0; i < planes_coeff.size(); ++i) {
    planes.push_back(std::make_tuple(
        std::make_shared<Plane_3>(
            planes_coeff[i].values[0], planes_coeff[i].values[1],
            planes_coeff[i].values[2], planes_coeff[i].values[3]),
        inliers[i], centroids[i]));
  }
  spdlog::trace("Number of planes read: {}", planes.size());

  // Create normal cloud
  CloudNPtr normals(new CloudN);
  normals->resize(cloud->size());
  for (size_t i = 0; i < planes.size(); ++i) {
    const auto &coeff = planes_coeff[i];
    for (const auto &index : *inliers[i]) {
      normals->points[index].normal_x = coeff.values[0];
      normals->points[index].normal_y = coeff.values[1];
      normals->points[index].normal_z = coeff.values[2];
    }
  }

  planes = merge_plaes(planes, cloud);
  spdlog::debug("Number of planes after merging: {}", planes.size());

  planes = force_orthogonal_planes(planes);
  spdlog::debug("Number of planes after forcing orthogonality: {}",
                planes.size());

  const double threshold = 1.5;
  auto pairs = make_pairs(planes, threshold);
  auto [walls, floors] = separate_planes(planes);

  // auto [walls, floors] = separate_planes(planes);
  spdlog::debug("Number of horizonal planes: {}", walls.size());
  spdlog::debug("Number of vertical planes: {}", floors.size());

  if (opt.display) {
    size_t count = 0;
    for (size_t i = 0; i < walls.size(); ++i) {
      const size_t id = walls[i];
      const auto wall = planes[id];
      const auto plane = std::get<0>(wall);
      const auto origin = std::get<2>(wall);
      std::string name = fmt::format("wall_{}-planes", i);
      pcl::ModelCoefficients coeff;
      coeff.values = {
          static_cast<float>(plane->a()), static_cast<float>(plane->b()),
          static_cast<float>(plane->c()), static_cast<float>(plane->d())};
      viewer->addPlane(coeff, origin[0], origin[1], origin[2], name);
      auto color = pcl::GlasbeyLUT::at(count);
      viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR,
          static_cast<double>(color.r) / 255.0,
          static_cast<double>(color.g) / 255.0,
          static_cast<double>(color.b) / 255.0, name);
      const PointT p(origin[0], origin[1], origin[2]);
      viewer->addText3D(name, p, 0.2, 1.0, 1.0, 1.0,
                        fmt::format("text_{}", name));
      ++count;
    }
    for (size_t i = 0; i < floors.size(); ++i) {
      const size_t id = floors[i];
      const auto floor = planes[id];
      const auto plane = std::get<0>(floor);
      const auto origin = std::get<2>(floor);
      std::string name = fmt::format("floor_{}-planes", i);
      pcl::ModelCoefficients coeff;
      coeff.values = {
          static_cast<float>(plane->a()), static_cast<float>(plane->b()),
          static_cast<float>(plane->c()), static_cast<float>(plane->d())};
      viewer->addPlane(coeff, origin[0], origin[1], origin[2], name);
      auto color = pcl::GlasbeyLUT::at(count);
      viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR,
          static_cast<double>(color.r) / 255.0,
          static_cast<double>(color.g) / 255.0,
          static_cast<double>(color.b) / 255.0, name);
      const PointT p(origin[0], origin[1], origin[2]);
      viewer->addText3D(name, p, 0.2, 1.0, 1.0, 1.0,
                        fmt::format("text_{}", name));
      ++count;
    }
    for (const auto &[i, j] : pairs) {
      auto origin_i = std::get<2>(planes[i]);
      auto origin_j = std::get<2>(planes[j]);

      PointT p1, p2;
      std::string name = fmt::format("pair_{}-{}", i, j);
      p1.getVector3fMap() = origin_i.head<3>();
      p2.getVector3fMap() = origin_j.head<3>();
      viewer->addLine<PointT>(p1, p2, 0.0, 0.0, 1.0, name);
    }
  }

  Arrangement arr;
  PointT min, max;
  pcl::getMinMax3D(*cloud, min, max);
  const double offset = 1.0;
  const Iso_rectangle_2 brec(min.x - offset, min.y - offset, max.x + offset,
                             max.y + offset);
  const Plane_3 ground_plane(0, 0, 1, -min.z);
  for (auto const id : walls) {
    auto const &wall = planes[id];
    const auto plane = std::get<0>(wall);
    // Intersect with ground plane to get line
    auto intersection_1 = CGAL::intersection(ground_plane, *plane);
    if (!intersection_1) {
      spdlog::warn("Wall plane does not intersect ground plane");
      continue;
    }
    const auto line_3d = std::get<Kernel::Line_3>(intersection_1.value());
    const Point_2 p(line_3d.point().x(), line_3d.point().y());
    const Direction_2 d(line_3d.direction().dx(), line_3d.direction().dy());
    const Line_2 line_2d(p, d);
    auto intersection_2 = CGAL::intersection(brec, line_2d);
    if (!intersection_2) {
      spdlog::warn(
          "Wall plane intersection does not intersect bounding rectangle");
      continue;
    }
    const auto segment = std::get<Segment>(intersection_2.value());
    CGAL::insert(arr, segment);
    // TODO: This seems like a hacky way to assign ids to halfedges
    for (auto heit = arr.halfedges_begin(); heit != arr.halfedges_end();
         ++heit) {
      if (heit->source_id == -1) {
        heit->source_id = id;
        heit->twin()->source_id = id;
      }
    }
  }

  if (opt.display) {
    constexpr double z_offset = -0.7;
    constexpr uint8_t color[3] = {0, 0, 0};

    auto make_point = [&](const auto &p) {
      PointT pt;
      pt.x = CGAL::to_double(p.x());
      pt.y = CGAL::to_double(p.y());
      pt.z = min.z + z_offset;
      pt.r = color[0];
      pt.g = color[1];
      pt.b = color[2];
      return pt;
    };

    size_t count = 0;
    for (auto fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
      if (fit->is_unbounded())
        continue;

      auto points = CloudPtr(new Cloud);

      // Walk around the outer boundary
      for (auto vit = fit->outer_ccb(), done = vit;;) {
        points->push_back(make_point(vit->source()->point()));
        if (++vit == done)
          break;
      }

      // Create indices [0..N-1]
      pcl::Vertices face;
      face.vertices.resize(points->size());
      std::iota(face.vertices.begin(), face.vertices.end(), 0);
      face.vertices.push_back(0);

      // Add polygon to viewer
      const std::string name = fmt::format("face_{}", count);
      auto c = pcl::GlasbeyLUT::at(count);
      viewer->addPolygonMesh<PointT>(points, {face}, name);
      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR,
          static_cast<double>(c.r) / 255.0, static_cast<double>(c.g) / 255.0,
          static_cast<double>(c.b) / 255.0, name);
      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, name);

      ++count;
    }
  }

  if (opt.display) {
    auto map_val = [&min, &max, &offset](const double val) {
      const double min_in = min.z - offset;
      const double max_in = max.z + offset;
      constexpr double min_out = 0.0;
      const double max_out = static_cast<double>(pcl::ViridisLUT::size());
      return static_cast<size_t>(
          (val - min_in) / (max_in - min_in) * (max_out - min_out) + min_out);
    };

    for (size_t i = 0; i < floors.size(); ++i) {
      const auto id = floors[i];
      const auto &floor = planes[id];
      const double height = std::get<2>(floor)[2];
      pcl::PointCloud<pcl::PointXYZ>::Ptr points(
          new pcl::PointCloud<pcl::PointXYZ>);
      points->push_back(pcl::PointXYZ(min.x - offset, min.y - offset, height));
      points->push_back(pcl::PointXYZ(max.x + offset, min.y - offset, height));
      points->push_back(pcl::PointXYZ(max.x + offset, max.y + offset, height));
      points->push_back(pcl::PointXYZ(min.x - offset, max.y + offset, height));

      pcl::Vertices face;
      face.vertices = {0, 1, 2, 3, 0};

      auto color = pcl::ViridisLUT::at(map_val(height));

      std::string name = fmt::format("floor_{}-layer", i);
      viewer->addPolygonMesh<pcl::PointXYZ>(points, {face}, name);
      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR,
          static_cast<double>(color.r) / 255.0,
          static_cast<double>(color.g) / 255.0,
          static_cast<double>(color.b) / 255.0, name);
      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, name);
    }
  }

  using Vd = ReUseX::CellComplex::Vertex;
  using Fd = ReUseX::CellComplex::Vertex;
  using Cd = ReUseX::CellComplex::Vertex;
  using VertexMap =
      std::unordered_map<Arrangement::Vertex_handle, std::vector<Vd>>;
  using FaceMap = std::unordered_map<Arrangement::Face_handle, std::vector<Fd>>;

  VertexMap point_map{};
  FaceMap face_map{};

  std::shared_ptr<ReUseX::CellComplex> cc =
      std::make_shared<ReUseX::CellComplex>();

  auto sorted_floors = floors;
  std::sort(sorted_floors.begin(), sorted_floors.end(),
            [&planes](auto a, auto b) {
              const auto &Pa = planes[a];
              const auto &Pb = planes[b];
              const double h1 = -std::get<0>(Pa)->d() * std::get<0>(Pa)->c();
              const double h2 = -std::get<0>(Pb)->d() * std::get<0>(Pb)->c();
              return h1 < h2;
            });

  for (size_t i = 0; i < sorted_floors.size(); ++i) {
    const auto id = sorted_floors[i];
    const auto plane = std::get<0>(planes[id]);
    auto height = -plane->d() * plane->c();
    spdlog::trace("Horizontal plane {} at height {:.3f} ", i, height);
  }

  spdlog::trace("Initialize vertex map for {} floors", sorted_floors.size());
  for (auto vit = arr.vertices_begin(); vit != arr.vertices_end(); ++vit) {
    point_map[vit] = std::vector<Vd>(sorted_floors.size());
  }

  spdlog::trace("Create {} vertices",
                arr.number_of_vertices() * sorted_floors.size());
  CloudPtr points(new Cloud);
  for (size_t i = 0; i < sorted_floors.size(); ++i) {
    const auto id = sorted_floors[i];
    const auto plane = std::get<0>(planes[id]);
    const double height = -plane->d() * plane->c();

    for (auto vit = arr.vertices_begin(); vit != arr.vertices_end(); ++vit) {

      point_map[vit][i] = cc->add_vertex();

      auto p = vit->point();
      PointT pt;
      pt.x = CGAL::to_double(p.x());
      pt.y = CGAL::to_double(p.y());
      pt.z = height;
      points->push_back(pt);
    }
  }

  if (opt.display) {
    const std::string name = "cellcomplex_vertices";
    pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler(
        points, 0, 0, 255);
    viewer->addPointCloud<PointT>(points, color_handler, name);
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, name);
  }

  // Initialize face map
  spdlog::trace("Initialize face map for {} floors", sorted_floors.size());
  for (auto fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
    if (fit->is_unbounded())
      continue;
    face_map[fit] = std::vector<Fd>(sorted_floors.size());
  }

  auto areas = cc->add_property_map<Fd, double>("f:area").first;
  auto face_area = [](Face_handle fit) {
    std::vector<Point_2> pts;
    auto vit = fit->outer_ccb();
    auto start = vit;
    do {
      pts.push_back(vit->source()->point());
    } while (++vit != start);
    Number_type area_nt;
    CGAL::area_2(pts.begin(), pts.end(), area_nt);
    return CGAL::to_double(area_nt);
  };

  // Construct planar faces for each floor
  spdlog::trace("Create {} horizontal faces",
                arr.number_of_faces() * sorted_floors.size());
  for (auto fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
    if (fit->is_unbounded())
      continue;

    const double area = face_area(fit);

    for (size_t i = 0; i < sorted_floors.size(); ++i) {
      std::vector<Vd> verts{};
      for (auto vit = fit->outer_ccb(), done = vit;;) {
        verts.push_back(point_map[vit->source()][i]);
        if (++vit == done)
          break;
      }
      auto f = cc->add_face(verts, sorted_floors[i] /* plane id */);
      face_map[fit][i] = f;
      areas[f] = area;
    }
  }

  auto face_center_xy = [](Face_handle fit) {
    Number_type cx = 0, cy = 0;
    size_t count = 0;
    auto vit = fit->outer_ccb();
    auto start = vit;
    do {
      cx += vit->source()->point().x();
      cy += vit->source()->point().y();
      ++count;
    } while (++vit != start);
    cx /= static_cast<Number_type>(count);
    cy /= static_cast<Number_type>(count);

    return std::make_pair(CGAL::to_double(cx), CGAL::to_double(cy));
  };

  CloudPtr cell_centers(new Cloud);
  std::unordered_map<size_t, Cd> cell_center_map{};
  // std::unordered_map<PointT, Cd, ReUseX::PointHash> cell_center_map{};

  spdlog::trace("Create {} cells",
                arr.number_of_faces() * (sorted_floors.size() - 1));
  auto volumes = cc->add_property_map<Cd, double>("c:volume").first;
  for (size_t i = 0; i < sorted_floors.size() - 1; ++i) {
    const auto id1 = sorted_floors[i];
    const auto id2 = sorted_floors[i + 1];
    const auto plane1 = std::get<0>(planes[id1]);
    const auto plane2 = std::get<0>(planes[id2]);
    const double h1 = -plane1->d() * plane1->c();
    const double h2 = -plane2->d() * plane2->c();
    const double dist = h2 - h1;
    spdlog::trace("Horizontal section thickness is {:.3f}", dist);

    for (auto fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
      if (fit->is_unbounded())
        continue;
      std::vector<Fd> fc{};
      fc.reserve(10);
      fc.push_back(face_map[fit][i]);
      fc.push_back(face_map[fit][i + 1]);

      // Walk around the outer boundary
      for (auto heit = fit->outer_ccb(), done = heit;;) {
        const auto pts = {
            point_map[heit->source()][i],
            point_map[heit->target()][i],
            point_map[heit->target()][i + 1],
            point_map[heit->source()][i + 1],
        };
        auto f = cc->add_face(pts, heit->source_id);
        face_map[fit][i] = f;
        areas[f] =
            dist * std::sqrt(CGAL::squared_distance(heit->source()->point(),
                                                    heit->target()->point()));

        fc.push_back(f);

        if (++heit == done)
          break;
      }

      // Create the cell
      auto c = cc->add_cell(fc);
      volumes[c] = dist * areas[face_map[fit][i]];

      double cx, cy, cz;
      std::tie(cx, cy) = face_center_xy(fit);
      // cz = h1 + d_half;
      cz = (h1 + h2) / 2.0;
      PointT pt;
      pt.x = cx;
      pt.y = cy;
      pt.z = cz;
      cell_center_map[cell_centers->size()] = c;
      cell_centers->push_back(pt);
    }
  }

  auto get_cell_indices = [](CloudConstPtr cloud, Eigen::Vector4f const &p1,
                             Eigen::Vector4f const &p2) {
    pcl::PlaneClipper3D<PointT> clipper(p1);

    Indices indices{};
    clipper.clipPointCloud3D(*cloud, indices);

    clipper.setPlaneParameters(p2);
    IndicesPtr result(new Indices);
    clipper.clipPointCloud3D(*cloud, *result, indices);
    return result;
  };

  for (size_t i = 0; i < pairs.size(); ++i) {
    const auto &[id1, id2] = pairs[i];
    auto plane1 = std::get<0>(planes[id1]);
    auto plane2 = std::get<0>(planes[id2]);

    Eigen::Vector4f plane1_vec(plane1->a(), plane1->b(), plane1->c(),
                               plane1->d());
    Eigen::Vector4f plane2_vec(plane2->a(), plane2->b(), plane2->c(),
                               plane2->d());
    auto indices = get_cell_indices(cell_centers, -plane1_vec, -plane2_vec);

    spdlog::trace("Plane pair {}-{} has {} inliers", id1, id2, indices->size());
    for (auto idx : *indices) {
      auto c = cell_center_map[idx];
      std::get<CellData>((*cc)[c].data).wall_ids.insert(i);
    }
  }
  cc->n_walls = pairs.size();

  if (opt.display) {
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red_color(
        cell_centers, 255, 0, 0);
    const std::string name = "cell_centers";
    viewer->addPointCloud<PointT>(cell_centers, red_color, name);
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
  }

  // TODO: Compute face probabilities
  auto f_sp = cc->add_property_map<Fd, double>("f:support_probability").first;
  for (auto fit = cc->faces_begin(); fit != cc->faces_end(); ++fit) {
    const int plane_id = std::get<FaceData>((*cc)[*fit].data).plane_id;
    if (plane_id < 0) {
      spdlog::warn("Face {} has no associated plane",
                   std::get<FaceData>((*cc)[*fit].data).id);
      continue; // Horizontal face
    }
    const auto plane = std::get<0>(planes[plane_id]);
    const Eigen::Vector4f plane_vec(plane->a(), plane->b(), plane->c(),
                                    plane->d());
    // TODO: Set the main cell (positive side of the plane)
    auto [begin, end] = boost::out_edges(*fit, *cc);
    for (auto eit = begin; eit != end; ++eit) {

      // Get the adjacent cell
      const auto t = boost::target(*eit, *cc);
      const auto s = boost::source(*eit, *cc);
      auto c = (*fit == t) ? s : t;
      if ((*cc)[c].type != NodeType::Cell)
        continue;

      // spdlog::trace("Face {} is adjacent to cell {}",
      //               std::get<FaceData>((*cc)[*fit].data).id,
      //               std::get<CellData>((*cc)[c].data).id);

      const int cell_id = std::get<CellData>((*cc)[c].data).id;
      const PointT &center = cell_centers->points[cell_id];

      const auto dist = dist_to_plane(plane_vec, center.getVector3fMap());
      if (dist < 0)
        continue; // Negative side of the plane
      (*cc)[*eit].is_main = true;
    }

    const auto indices = std::get<1>(planes[plane_id]);
    if (indices->empty()) {
      // spdlog::warn("Plane {} has no inliers", plane_id);
      f_sp[*fit] = 0.0;
      continue;
    }

    Polygon_2 polygon{};
    std::transform(cc->vertices_begin(*fit), cc->vertices_end(*fit),
                   std::back_inserter(polygon), [&](Vd v) {
                     const auto id = std::get<VertexData>((*cc)[v].data).id;
                     const PointT &p = points->points[id];
                     return plane->to_2d(Point_3(p.x, p.y, p.z));
                   });

    auto inliers = *indices | std::views::transform([&](int idx) {
      const PointT &p = cloud->points[idx];
      return plane->to_2d(Point_3(p.x, p.y, p.z));
    }) | std::views::filter([&](const Point_2 &p) {
      return polygon.bounded_side(p) == CGAL::ON_BOUNDED_SIDE;
    }) | collect<std::vector>();

    if (inliers.empty()) {
      f_sp[*fit] = 0.0;
      continue;
    }
    f_sp[*fit] = compute_grid_coverage(polygon, inliers, 0.2);
  }

  // TODO: Compute room probabilities
  const double grid_size_ = 0.2;
  const double radius_ = grid_size_ * M_SQRT2;
  const float epsilon_ = 0.01f;
  unsigned int old_mxcsr = _mm_getcsr(); // save current flagsq

  _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
  _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

  spdlog::trace("Creating device and scene for ray tracing");
  RTCDevice device_ = rtcNewDevice("verbose=0"); // 0-3
  RTCScene scene_ = rtcNewScene(device_);
  rtcSetSceneFlags(scene_, RTC_SCENE_FLAG_ROBUST);
  rtcSetSceneBuildQuality(scene_, RTC_BUILD_QUALITY_HIGH);
  assert(device_ != nullptr && "Error creating Embree device");
  assert(scene_ != nullptr && "Error creating Embree scene");
  // rtcSetDeviceErrorFunction(device_, rtcCheckError, nullptr);

  // TODO: Get Subsampled indices for point cloud

  // Get unique labels
  std::set<unsigned int> labels_set{};
  for (const auto &l : room_labels->points)
    labels_set.insert(l.label);

  std::vector<unsigned int> labels(labels_set.begin(), labels_set.end());

  auto c_rp =
      cc->add_property_map<Cd, std::vector<double>>("c:room_probabilities")
          .first;

  for (auto cit = cc->cells_begin(); cit != cc->cells_end(); ++cit)
    c_rp[*cit] = std::vector<double>(labels.size(), 0.0);

  // Intersect with ground plane to get line/ INFO: Create Embree scene
  spdlog::trace("Creating scene geometry for ray tracing");
  pcl::PassThrough<LabelT> pass;
  pass.setInputCloud(room_labels);
  pass.setFilterFieldName("label");

  pcl::UniformSampling<LabelT> us;
  us.setInputCloud(room_labels);
  us.setRadiusSearch(grid_size_);
  struct RTCData {
    size_t label_index;
  };
  using RTCDataPtr = std::shared_ptr<RTCData>;
  std::vector<RTCDataPtr> rtc_data_vec{}; // To keep data alive
  for (size_t i = 0; i < labels.size(); ++i) {

    IndicesPtr indices(new Indices);
    pass.setFilterLimits(static_cast<float>(labels[i]),
                         static_cast<float>(labels[i]));
    pass.filter(*indices);
    us.setIndices(indices);
    us.filter(*indices);

    RTCGeometry geometry_ =
        rtcNewGeometry(device_, RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT);

    RTCVertex *vb = (RTCVertex *)rtcSetNewGeometryBuffer(
        geometry_, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4,
        sizeof(RTCVertex), indices->size());

    RTCNormal *nb = (RTCNormal *)rtcSetNewGeometryBuffer(
        geometry_, RTC_BUFFER_TYPE_NORMAL, 0, RTC_FORMAT_FLOAT3,
        sizeof(RTCNormal), indices->size());

    for (size_t i = 0; i < indices->size(); ++i) {
      const auto &p = cloud->points[indices->at(i)];
      const auto &n = normals->points[indices->at(i)];

      vb[i].x = static_cast<float>(p.x);
      vb[i].y = static_cast<float>(p.y);
      vb[i].z = static_cast<float>(p.z);
      vb[i].r = static_cast<float>(radius_) * 1.05f;

      nb[i].x = static_cast<float>(n.normal_x);
      nb[i].y = static_cast<float>(n.normal_y);
      nb[i].z = static_cast<float>(n.normal_z);
    }

    rtcSetGeometryMask(geometry_, 0xFFFFFFFF);

    rtc_data_vec.emplace_back(new RTCData{i});
    rtcSetGeometryUserData(geometry_, rtc_data_vec.back().get());

    rtcCommitGeometry(geometry_);
    rtcAttachGeometry(scene_, geometry_);
    rtcReleaseGeometry(geometry_);
  }

  spdlog::trace("Committing scene");
  rtcCommitScene(scene_);

  const auto dirs = sampleSphericalFibonacci(100);
  /*
   #pragma omp parallel for reduction(mergeTriplets : triplets) \
    schedule(dynamic, 4)
  */
  for (auto cit = cc->cells_begin(); cit != cc->cells_end(); ++cit) {
    // For each cell create n random rays
    // Count the number of intersections per id nad normalize
    for (size_t i = 0; i < dirs.size(); ++i) {
      const auto dir = dirs[i];

      const size_t idx = std::get<CellData>((*cc)[*cit].data).id;
      Eigen::Vector3f point_s = cell_centers->points[idx].getVector3fMap();

      RTCRayHit rayhit;
      rayhit.ray.org_x = point_s.x();
      rayhit.ray.org_y = point_s.y();
      rayhit.ray.org_z = point_s.z();
      rayhit.ray.dir_x = dir.x();
      rayhit.ray.dir_y = dir.y();
      rayhit.ray.dir_z = dir.z();

      rayhit.ray.tnear = epsilon_;
      // rayhit.ray.tfar = dir_norm - epsilon_;

      // rayhit.ray.time = 0.0f;       // motion blur time, not used here
      // rayhit.ray.mask = 0xFFFFFFFF; // mask for ray types, not used here
      // rayhit.ray.id = 0;            // ray ID, not used here
      // rayhit.ray.flags = 0;         // ray flags, not used here

      rtcIntersect1(scene_, &rayhit);
      if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
        // Check if backside
        const auto normal =
            Eigen::Vector3f(rayhit.hit.Ng_x, rayhit.hit.Ng_y, rayhit.hit.Ng_z)
                .normalized();
        const Eigen::Vector3f dir_vec(dir.x(), dir.y(), dir.z());
        if (normal.dot(dir_vec) < 0)
          continue; // Backside

        auto geometry = rtcGetGeometry(scene_, rayhit.hit.geomID);
        rtcGetGeometryUserData(geometry);
        RTCData *data = (RTCData *)rtcGetGeometryUserData(geometry);

        // #pragma omp critical
        c_rp[*cit][data->label_index] += 1;
      }

      // Compute probabilities
      double sum = std::accumulate(c_rp[*cit].begin(), c_rp[*cit].end(), 0.0);
      if (sum > 0)
        for (size_t j = 0; j < c_rp[*cit].size(); ++j)
          c_rp[*cit][j] /= sum;
    }
  }
  rtcReleaseScene(scene_);
  rtcReleaseDevice(device_);

  _mm_setcsr(old_mxcsr); // restore old flags

  spdlog::debug("Cell complex: {}", *cc);

  spdlog::trace("Initializing Solidifier");
  ReUseX::Solidifier solidifier(cc);
  auto results = solidifier.solve();
  if (results.has_value()) {
    auto values = results.value();
    for (size_t i = 0; i < values.size(); ++i)
      spdlog::trace("x_{}= {}", i, values[i]);
  }

  if (opt.display)
    while (!viewer->wasStopped()) {
      viewer->spinOnce(100);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

  return 0;
}
