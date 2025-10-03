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
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2_algorithms.h>

#include <pcl/common/colors.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

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
using Arrangement = CGAL::Arrangement_2<Traits>;

using Vertex_handle = Arrangement::Vertex_handle;
using Halfedge_handle = Arrangement::Halfedge_handle;
using Face_handle = Arrangement::Face_handle;

using Plane_3 = Kernel::Plane_3;
using Point_3 = Kernel::Point_3;
using Vector_3 = Kernel::Vector_3;
using Iso_cuboid_3 = Kernel::Iso_cuboid_3;
using Iso_rectangle_2 = CGAL::Iso_rectangle_2<Kernel>;

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
  spdlog::trace("Separate planes into walls and floors/ceilings");

  using value_type = Planes::value_type;

  std::vector<value_type> walls{};
  std::vector<value_type> floors{};

  for (size_t i = 0; i < planes.size(); ++i) {
    auto plane = std::get<0>(planes[i]);

    const Vector_3 normal = plane->orthogonal_vector();
    const double scalar = CGAL::to_double(normal * up);

    if (std::abs(scalar) < epsilon) // Wall
      walls.push_back(planes[i]);
    else if (std::abs(scalar - 1.0) < epsilon) // Floor
      floors.push_back(planes[i]);
    else if (std::abs(scalar + 1.0) < epsilon) // Ceiling
      floors.push_back(planes[i]);
    else
      spdlog::warn("Plane {} is not vertical or horizontal", i);
  }

  return std::make_tuple(walls, floors);
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

  CloudPtr cloud(new Cloud);

  spdlog::trace("Reading input file: {}", cloud_path);
  pcl::io::load<PointT>(cloud_path.string(), *cloud);

  if (opt.display) {
    spdlog::trace("Displaying point cloud with {} points", cloud->size());
    const std::string cloud_name = "cloud";
    viewer->addPointCloud<PointT>(cloud, cloud_name);
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

  planes = merge_plaes(planes, cloud);
  spdlog::debug("Number of planes after merging: {}", planes.size());

  planes = force_orthogonal_planes(planes);
  spdlog::debug("Number of planes after forcing orthogonality: {}",
                planes.size());

  // TODO: First make pairs, the split
  // auto pairs = make_pairs(planes);
  // auto [walls, floors] = separate_planes(planes);

  auto [walls, floors] = separate_planes(planes);
  spdlog::debug("Number of walls: {}", walls.size());
  spdlog::debug("Number of floors: {}", floors.size());

  const double threshold = 1.5;
  auto wall_pairs = make_pairs(walls, threshold);
  spdlog::debug("Number of wall pairs: {} of {}", wall_pairs.size(),
                walls.size());

  auto floor_pairs = make_pairs(floors, threshold);
  spdlog::debug("Number of floor pairs: {} of {}", floor_pairs.size(),
                floors.size());

  if (opt.display) {
    size_t count = 0;
    for (size_t i = 0; i < walls.size(); ++i) {
      auto plane = std::get<0>(walls[i]);
      auto origin = std::get<2>(walls[i]);
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
      auto plane = std::get<0>(floors[i]);
      auto origin = std::get<2>(floors[i]);
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

    for (const auto &[i, j] : wall_pairs) {
      auto origin_i = std::get<2>(walls[i]);
      auto origin_j = std::get<2>(walls[j]);

      PointT p1, p2;
      std::string name = fmt::format("wall_pair_{}-{}", i, j);
      p1.getVector3fMap() = origin_i.head<3>();
      p2.getVector3fMap() = origin_j.head<3>();
      viewer->addLine<PointT>(p1, p2, 1.0, 0.0, 0.0, name);
    }
    for (const auto &[i, j] : floor_pairs) {
      auto origin_i = std::get<2>(floors[i]);
      auto origin_j = std::get<2>(floors[j]);

      PointT p1, p2;
      std::string name = fmt::format("floor_pair_{}-{}", i, j);
      p1.getVector3fMap() = origin_i.head<3>();
      p2.getVector3fMap() = origin_j.head<3>();
      viewer->addLine<PointT>(p1, p2, 0.0, 1.0, 0.0, name);
    }
  }

  Arrangement arr;
  PointT min, max;
  pcl::getMinMax3D(*cloud, min, max);
  const double offset = 1.0;
  const Iso_rectangle_2 brec(min.x - offset, min.y - offset, max.x + offset,
                             max.y + offset);
  const Plane_3 ground_plane(0, 0, 1, -min.z);
  for (auto const &wall : walls) {
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
    size_t n_floors = 0;
    auto map_val = [&min, &max, &offset](const double val) {
      const double min_in = min.z - offset;
      const double max_in = max.z + offset;
      constexpr double min_out = 0.0;
      const double max_out = static_cast<double>(pcl::ViridisLUT::size());
      return static_cast<size_t>(
          (val - min_in) / (max_in - min_in) * (max_out - min_out) + min_out);
    };

    for (auto floor : floors) {
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

      std::string name = fmt::format("floor_{}-layer", n_floors);
      viewer->addPolygonMesh<pcl::PointXYZ>(points, {face}, name);
      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR,
          static_cast<double>(color.r) / 255.0,
          static_cast<double>(color.g) / 255.0,
          static_cast<double>(color.b) / 255.0, name);
      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, name);
      ++n_floors;
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
  std::sort(sorted_floors.begin(), sorted_floors.end(), [](auto a, auto b) {
    const double h1 = -std::get<0>(a)->d() * std::get<0>(a)->c();
    const double h2 = -std::get<0>(b)->d() * std::get<0>(b)->c();
    return h1 < h2;
  });

  for (size_t i = 0; i < sorted_floors.size(); ++i) {
    const auto plane = std::get<0>(sorted_floors[i]);
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
    const auto plane = std::get<0>(sorted_floors[i]);
    const double height = -plane->d() * plane->c();
    // const double height = std::get<0>(sorted_floors[i])->d();

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
      auto f = cc->add_face(verts);
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
    const auto plane1 = std::get<0>(sorted_floors[i]);
    const auto plane2 = std::get<0>(sorted_floors[i + 1]);
    const double h1 = -plane1->d() * plane1->c();
    const double h2 = -plane2->d() * plane2->c();
    // const double h1 = std::get<0>(sorted_floors[i])->d();
    // const double h2 = std::get<0>(sorted_floors[i + 1])->d();
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
        auto f = cc->add_face(pts);
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

  // TODO: When pairs have been merged, merge these two loops
  size_t n_wall_pairs = 0;
  for (const auto &[i, j] : wall_pairs) {
    auto plane_i = std::get<0>(walls[i]);
    auto plane_j = std::get<0>(walls[j]);
    Eigen::Vector4f plane_i_vec(plane_i->a(), plane_i->b(), plane_i->c(),
                                plane_i->d());
    Eigen::Vector4f plane_j_vec(plane_j->a(), plane_j->b(), plane_j->c(),
                                plane_j->d());

    auto indices = get_cell_indices(cell_centers, -plane_i_vec, -plane_j_vec);

    spdlog::trace("Wall pair {}-{} has {} inliers", i, j, indices->size());

    for (auto idx : *indices) {
      auto c = cell_center_map[idx];
      std::get<CellData>((*cc)[c].data).wall_ids.insert(n_wall_pairs);
    }

    ++n_wall_pairs;
  }

  if (opt.display) {
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red_color(
        cell_centers, 255, 0, 0);
    const std::string name = "cell_centers";
    viewer->addPointCloud<PointT>(cell_centers, red_color, name);
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
  }

  for (const auto &[i, j] : floor_pairs) {
    auto plane_i = std::get<0>(floors[i]);
    auto plane_j = std::get<0>(floors[j]);

    Eigen::Vector4f plane_i_vec(plane_i->a(), plane_i->b(), plane_i->c(),
                                plane_i->d());
    Eigen::Vector4f plane_j_vec(plane_j->a(), plane_j->b(), plane_j->c(),
                                plane_j->d());

    auto indices = get_cell_indices(cell_centers, -plane_i_vec, -plane_j_vec);

    spdlog::trace("Horizontal pair {}-{} has {} inliers", i, j,
                  indices->size());

    for (auto idx : *indices) {
      auto c = cell_center_map[idx];
      std::get<CellData>((*cc)[c].data).wall_ids.insert(n_wall_pairs);
    }
    ++n_wall_pairs;
  }
  cc->n_walls = n_wall_pairs;

  // TODO: Compute face and room probabilities
  auto f_sp = cc->add_property_map<Fd, double>("f:support_probability").first;
  auto c_rp =
      cc->add_property_map<Cd, std::vector<double>>("c:room_probabilities")
          .first;

  spdlog::debug("Cell complex has {} vertices, {} faces, and {} cells",
                cc->num_vertices(), cc->num_faces(), cc->num_cells());

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
