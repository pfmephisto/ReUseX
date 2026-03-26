// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "geometry/mesh.hpp"
#include "core/logging.hpp"
#include "core/processing_observer.hpp"
#include "geometry/CellComplex.hpp"
#include "geometry/Solidifier.hpp"
#include "geometry/regularization.hpp"
// #include "geometry/utils.hpp"
// #include "io/reusex.hpp"

// #include <algorithm>
// #include <filesystem>
// #include <latch>
// #include <mutex>
// #include <thread>

#include <fmt/color.h>
#include <fmt/std.h>

namespace ReUseX::geometry {

pcl::PolygonMeshPtr mesh(CloudConstPtr cloud, CloudNConstPtr normals,
                         EigenVectorContainer<double, 4> &planes,
                         EigenVectorContainer<double, 3> &centroids,
                         std::vector<IndicesPtr> &inliers, CloudLConstPtr rooms,
                         MeshOptions const opt) {

  auto observer = ReUseX::core::get_visual_observer();
  constexpr auto stage = ReUseX::core::Stage::MeshGeneration;

  observer->viewer_add_geometry("input_cloud", cloud, stage);

  planes = regularizePlanes<double, PointT>(planes, cloud, inliers, 10.0);

  std::tie(planes, inliers, centroids) =
      merge_planes(planes, inliers, centroids, cloud);
  ReUseX::core::debug("Number of planes after merging: {}", planes.size());

  planes = force_orthogonal_planes(planes);
  ReUseX::core::debug("Number of planes after forcing orthogonality: {}",
                      planes.size());

  auto pairs = make_pairs(planes, inliers, centroids, opt.search_threshold,
                          opt.new_plane_offset);
  ReUseX::core::debug("Number of plane pairs: {}", pairs.size());

  auto [vertical, horizontal] = separate_planes(planes);
  ReUseX::core::debug(
      "Number indices in inliers [{}]",
      fmt::join(inliers | ranges::views::transform(
                              [](auto const &i) { return i->size(); }),
                ", "));
  ReUseX::core::debug("Number of horizonal planes: {}", horizontal.size());
  ReUseX::core::debug("Number of vertical planes: {}", vertical.size());

  ReUseX::core::debug("Vertical planes [{}]", fmt::join(vertical, ", "));
  ReUseX::core::debug("Horizontal planes [{}]", fmt::join(horizontal, ", "));

  auto sel = [&](size_t idx) {
    return std::make_pair(planes[idx], centroids[idx]);
  };

  auto vPlanes =
      vertical | ranges::views::transform(sel) | ranges::to<std::vector>();

  auto hPlanes =
      horizontal | ranges::views::transform(sel) | ranges::to<std::vector>();

  auto pPairs = pairs | ranges::views::transform([&](auto const &p) {
                  return std::make_pair(sel(p.first), sel(p.second));
                }) |
                ranges::to<std::vector>();

  observer->viewer_add_geometries("vertical_planes", vPlanes, stage);
  observer->viewer_add_geometries("horizontal_planes", hPlanes, stage);
  observer->viewer_add_geometries("plane_pairs", pPairs, stage);

  // viewer->addPlanes(vertical_planes, "vertical_planes", vps->at(0));
  // viewer->addPlanes(horizontal_planes, "horizontal_planes", vps->at(0));
  // viewer->addPlanePairs(plane_pairs, "plain_pairs", vps->at(0));

  PointT min, max;
  pcl::getMinMax3D(*cloud, min, max);

  // Floors are visualized via the "horizontal_planes" observer call above.
  // The rux VizualizationObserver renders horizontal planes as floor quads.

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

  ReUseX::core::debug("Cell complex: {}", *cc);

  // INFO: Display cell complex
  // if (viewer)
  //  viewer->addCellComplex(cc, "cell_complex", vps->at(1));

  cc->compute_room_probabilities<PointT, NormalT, LabelT>(cloud, normals,
                                                          rooms);
  // // INFO: Display face support probabilities
  // if (false && viewer)
  //   viewer->addSupportProbabilities(cc, "face_support_probablilites",
  //   vps->at(2));

  cc->compute_face_coverage<PointT>(cloud, planes, inliers);
  // INFO: Display room probabilities
  observer->viewer_add_geometry("cell_complex", *cc, stage);
  //   viewer->addRoomProbabilities(cc, "room_probablilites", vps->at(2));

  ReUseX::core::debug("Cell complex: {}", *cc);

  ReUseX::core::trace("Initializing Solidifier");
  ReUseX::geometry::Solidifier solidifier(cc);
  auto results = solidifier.solve();

  if (!results)
    ReUseX::core::warn("Solidification failed to find a solution");

  // INFO: Display results
  if (results.has_value())
    observer->viewer_add_geometry("rooms", results.value(), stage);
  // viewer->addRooms(cc, results.value(), "rooms", vps->at(3));

  pcl::PolygonMeshPtr mesh(new pcl::PolygonMesh());
  if (results) {
    // TODO: Refactor mesh conversion to return value instead of out parameter
    // category=Geometry estimate=1h
    // Current code builds mesh in-place but could return it directly for
    // cleaner API. Consider returning pcl::PolygonMeshPtr from this branch
    // instead of using the mesh variable declared above. Improves readability
    // and follows modern C++ patterns.
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
    // ReUseX::core::debug("Generated mesh with {} polygons",
    // mesh->polygons.size()); return mesh;
  }

  return mesh;
}
} // namespace ReUseX::geometry
