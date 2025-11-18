// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ReUseX/geometry/mesh.hpp"
#include "ReUseX/geometry/CellComplex.hpp"
#include "ReUseX/geometry/Solidifier.hpp"
#include "ReUseX/geometry/regularization.hpp"
// #include "ReUseX/geometry/utils.hpp"
// #include "ReUseX/io/reusex.hpp"

#include <pcl/visualization/pcl_visualizer.h>

// #include <algorithm>
// #include <filesystem>
// #include <latch>
// #include <mutex>
// #include <thread>

#include <fmt/color.h>
#include <fmt/std.h>
#include <spdlog/spdlog.h>

namespace ReUseX::geometry {

pcl::PolygonMeshPtr
mesh(CloudConstPtr cloud, CloudNConstPtr normals,
     EigenVectorContainer<double, 4> &planes,
     EigenVectorContainer<double, 3> &centroids,
     std::vector<IndicesPtr> &inliers, CloudLConstPtr rooms,
     MeshOptions const opt,
     std::shared_ptr<ReUseX::visualize::Visualizer> viewer) {

  std::shared_ptr<const std::vector<int>> vps = nullptr;

  if (viewer) {
    vps = viewer->getViewports();
    if (vps->size() < 4) {
      spdlog::warn("Visualizer has less than 4 viewports defined, disabling "
                   "visualization");
      viewer = nullptr;
    }
  }

  if (viewer) {
    viewer->addPointCloud(cloud, "cloud", vps->at(0));
    viewer->resetCameraViewpoint("cloud");
  }

  planes = regularizePlanes<double, PointT>(planes, cloud, inliers, 10.0);

  std::tie(planes, inliers, centroids) =
      merge_planes(planes, inliers, centroids, cloud);
  spdlog::debug("Number of planes after merging: {}", planes.size());

  planes = force_orthogonal_planes(planes);
  spdlog::debug("Number of planes after forcing orthogonality: {}",
                planes.size());

  auto pairs = make_pairs(planes, inliers, centroids, opt.search_threshold,
                          opt.new_plane_offset);
  spdlog::debug("Number of plane pairs: {}", pairs.size());

  auto [vertical, horizontal] = separate_planes(planes);
  spdlog::debug("Number indices in inliers [{}]",
                fmt::join(inliers | ranges::views::transform([](auto const &i) {
                            return i->size();
                          }),
                          ", "));
  spdlog::debug("Number of horizonal planes: {}", horizontal.size());
  spdlog::debug("Number of vertical planes: {}", vertical.size());

  spdlog::debug("Vertical planes [{}]", fmt::join(vertical, ", "));
  spdlog::debug("Horizontal planes [{}]", fmt::join(horizontal, ", "));

  if (viewer) {
    auto sel = [&](size_t idx) {
      return std::make_pair(planes[idx], centroids[idx]);
    };

    auto vertical_planes =
        vertical | ranges::views::transform(sel) | ranges::to<std::vector>();
    viewer->addPlanes(vertical_planes, "vertical_planes", vps->at(0));

    auto horizontal_planes =
        horizontal | ranges::views::transform(sel) | ranges::to<std::vector>();
    viewer->addPlanes(horizontal_planes, "horizontal_planes", vps->at(0));

    auto plane_pairs = pairs | ranges::views::transform([&](auto const &p) {
                         return std::make_pair(sel(p.first), sel(p.second));
                       }) |
                       ranges::to<std::vector>();
    viewer->addPlanePairs(plane_pairs, "plain_pairs", vps->at(0));
  }

  PointT min, max;
  pcl::getMinMax3D(*cloud, min, max);

  // INFO: Display floors
  if (viewer) {
    auto heights =
        horizontal |
        ranges::views::transform([&](auto idx) { return planes[idx][2]; }) |
        ranges::to<std::vector>();
    Eigen::Vector3d min_vec(min.x, min.y, min.z);
    Eigen::Vector3d max_vec(max.x, max.y, max.z);
    min_vec -= Eigen::Vector3d::Ones();
    max_vec += Eigen::Vector3d::Ones();
    viewer->addFloors(heights, min_vec, max_vec, "floor", vps->at(0));
  }

  // // auto viz_callback = [&queue_mutex, &task_queue,
  // //                      vp_2](size_t idx,
  // //                            std::vector<std::array<double, 3>> const
  // &pts,
  // //                            std::vector<int> const &indices) {
  // //   std::lock_guard<std::mutex> lock(queue_mutex);
  // //   task_queue.push([idx, vp_2, &pts, &indices](VisualizerPtr viewer) {
  // //     auto points = CloudPtr(new Cloud);
  // //     for (const auto &p : pts) {
  // //       PointT pt;
  // //       pt.x = static_cast<float>(p[0]);
  // //       pt.y = static_cast<float>(p[1]);
  // //       pt.z = static_cast<float>(p[2]);
  // //       pt.r = 0;
  // //       pt.g = 0;
  // //       pt.b = 0;
  // //       points->push_back(pt);
  // //     }

  // //    pcl::Vertices face;
  // //    face.vertices = indices;
  // //    auto color = pcl::GlasbeyLUT::at(idx);

  // //    // Add polygon to viewer
  // //    const std::string name = fmt::format("temp_{}", idx);
  // //    viewer->addPolygonMesh<PointT>(points, {face}, name, vp_2);
  // //    viewer->setPointCloudRenderingProperties(
  // //        pcl::visualization::PCL_VISUALIZER_COLOR,
  // //        static_cast<double>(color.r) / 255.0,
  // //        static_cast<double>(color.g) / 255.0,
  // //        static_cast<double>(color.b) / 255.0, name, vp_2);
  // //    // viewer->setPointCloudRenderingProperties(
  // //    //     pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, name, vp_2);
  // //  });
  // //};

  std::shared_ptr<CellComplex> cc = std::make_shared<CellComplex>(
      planes, vertical, horizontal, pairs,
      std::array<double, 2>{min.x - 1, min.y - 1},
      std::array<double, 2>{max.x + 1, max.y + 1}, std::nullopt);

  spdlog::debug("Cell complex: {}", *cc);

  // INFO: Display cell complex
  if (viewer)
    viewer->addCellComplex(cc, "cell_complex", vps->at(1));

  cc->compute_room_probabilities<PointT, NormalT, LabelT>(cloud, normals,
                                                          rooms);
  // INFO: Display face support probabilities
  if (false && viewer)
    viewer->addSupportProbabilities(cc, "face_support_probablilites",
                                    vps->at(2));

  cc->compute_face_coverage<PointT>(cloud, planes, inliers);
  // INFO: Display room probabilities
  if (viewer)
    viewer->addRoomProbabilities(cc, "room_probablilites", vps->at(2));

  spdlog::debug("Cell complex: {}", *cc);

  spdlog::trace("Initializing Solidifier");
  ReUseX::geometry::Solidifier solidifier(cc);
  auto results = solidifier.solve();

  if (!results)
    spdlog::warn("Solidification failed to find a solution");

  // INFO: Display results
  if (results.has_value() && viewer)
    viewer->addRooms(cc, results.value(), "rooms", vps->at(3));

  pcl::PolygonMeshPtr mesh(new pcl::PolygonMesh());
  if (results) {
    // TODO: Convert to mesh and return
    auto [room_labels, wall_labels] = results.value();
    auto [vertices, faces] = solidifier.toMesh(
        [&](const CellComplex::Vertex v) { return room_labels[v] > 0; });
    CloudLocPtr mesh_vertices(new CloudLoc);
    mesh_vertices->points.resize(vertices.rows());
    mesh_vertices->width = static_cast<uint32_t>(vertices.rows());
    mesh_vertices->height = 1;
    for (int i = 0; i < vertices.rows(); ++i) {
      mesh_vertices->points[i].x = static_cast<float>(vertices(i, 0));
      mesh_vertices->points[i].y = static_cast<float>(vertices(i, 1));
      mesh_vertices->points[i].z = static_cast<float>(vertices(i, 2));
    }
    pcl::toPCLPointCloud2(*mesh_vertices, mesh->cloud);
    mesh->polygons.resize(faces.rows());
    for (int i = 0; i < faces.rows(); ++i) {
      pcl::Vertices polygon;
      for (int j = 0; j < faces.cols(); ++j) {
        polygon.vertices.push_back(faces(i, j));
      }
      mesh->polygons[i] = polygon;
    }
    // spdlog::debug("Generated mesh with {} polygons", mesh->polygons.size());
    // return mesh;
  }

  return mesh;
}
} // namespace ReUseX::geometry
