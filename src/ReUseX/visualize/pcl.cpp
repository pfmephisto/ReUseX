// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ReUseX/visualize/pcl.hpp"
#include "ReUseX/utils/math.hpp"

#include <range/v3/to_container.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/transform.hpp>
namespace ReUseX::visualize {
void addPlane(std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
              const Eigen::Vector4d &plane, const Eigen::Vector3d &origin,
              const pcl::RGB &color, const std::string_view &name, int vp) {

  pcl::ModelCoefficients coeff;
  coeff.values = {static_cast<float>(plane[0]), static_cast<float>(plane[1]),
                  static_cast<float>(plane[2]), static_cast<float>(plane[3])};

  viewer->addPlane(coeff, origin[0], origin[1], origin[2], std::string{name},
                   vp);

  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                      static_cast<double>(color.r) / 255.0,
                                      static_cast<double>(color.g) / 255.0,
                                      static_cast<double>(color.b) / 255.0,
                                      std::string{name}, vp);
  // Add text next to plane
  // const PointT p(origin[0], origin[1], origin[2]);
  // viewer->addText3D(name, p, 0.2, 1.0, 1.0, 1.0,
  //                   fmt::format("text_{}", name), vp);
}
void addPlanes(std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
               const std::vector<Pair> &vertical_planes,
               const std::string_view &name, int vp) {
  // spdlog::trace("Displaying planes");
  for (auto [idx, pair] : ranges::views::enumerate(vertical_planes)) {
    auto [plane, origin] = pair;
    auto color = pcl::GlasbeyLUT::at(idx);
    addPlane(viewer, plane, origin, color, fmt::format("{}_{}", name, idx), vp);
  }
}

void addPair(std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
             const PlanePair &pair, const std::string_view &name, int vp) {
  auto [plane_i, origin_i] = pair.first;
  auto [plane_j, origin_j] = pair.second;

  PointT p1, p2;
  p1.getVector3fMap() = origin_i.head<3>().cast<float>();
  p2.getVector3fMap() = origin_j.head<3>().cast<float>();
  viewer->addLine<PointT>(p1, p2, 0.0, 0.0, 1.0, fmt::format("{}_line", name),
                          vp);

  // auto mid = 0.5 * (origin_i + origin_j);
  // auto text_name = fmt::format("{}_pair" name);
  // PointT p;
  // p.getArray3fMap() = mid.cast<float>();
  // viewer->addText3D(fmt::format("P{}", i), p, 0.2, 1.0, 1.0, 1.0, text_name,
  //                   vp);

  // viewer->addText3D(fmt::format("{}", pairs[i].first),
  //                   PointT{static_cast<float>(p1[0]),
  //                   static_cast<float>(p1[1]),
  //                          static_cast<float>(p1[2])},
  //                   0.1, 1.0, 1.0, 1.0,
  //                   fmt::format("text_{}_{}", text_name, pairs[i].first),
  //                   vp);
  // viewer->addText3D(fmt::format("{}", pairs[i].second),
  //                   PointT{static_cast<float>(p2[0]),
  //                   static_cast<float>(p2[1]),
  //                          static_cast<float>(p2[2])},
  //                   0.1, 1.0, 1.0, 1.0,
  //                   fmt::format("text_{}_{}", text_name, pairs[i].second),
  //                   vp);
}

void addPlanePairs(std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                   const std::vector<PlanePair> &plane_pairs,
                   const std::string_view &name, int vp) {
  for (auto [idx, pair] : ranges::views::enumerate(plane_pairs)) {
    addPair(viewer, pair, std::string_view(fmt::format("{}_{}", name, idx)),
            vp);
  }
}

void addFloor(std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
              const double height, const Eigen::Vector3d &min,
              const Eigen::Vector3d &max, const std::string_view &name,
              int vp) {

  const auto color =
      pcl::ViridisLUT::at(static_cast<int>(ReUseX::utils::remap<double>(
          height, min.z(), max.z(), 0.0, pcl::ViridisLUT::size())));

  pcl::PointCloud<pcl::PointXYZ>::Ptr points(
      new pcl::PointCloud<pcl::PointXYZ>);
  points->push_back(pcl::PointXYZ(min.x(), min.y(), height));
  points->push_back(pcl::PointXYZ(max.x(), min.y(), height));
  points->push_back(pcl::PointXYZ(max.x(), max.y(), height));
  points->push_back(pcl::PointXYZ(min.x(), max.y(), height));

  pcl::Vertices face;
  face.vertices = {0, 1, 2, 3, 0};

  std::string name_str{name};

  viewer->addPolygonMesh<pcl::PointXYZ>(points, {face}, name_str, vp);

  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_COLOR,
      static_cast<double>(color.r) / 255.0,
      static_cast<double>(color.g) / 255.0,
      static_cast<double>(color.b) / 255.0, name_str);
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, name_str, vp);
}

void addFloors(std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
               const std::vector<double> &heights, const Eigen::Vector3d &min,
               const Eigen::Vector3d &max, const std::string_view &name,
               int vp) {
  // spdlog::trace("Displaying floors");
  for (auto [idx, height] : ranges::views::enumerate(heights)) {
    addFloor(viewer, height, min, max, fmt::format("{}_{}", name, idx), vp);
  }
}

void addCellComplex(std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                    const std::shared_ptr<ReUseX::geometry::CellComplex> &cc,
                    const std::string_view &name, int vp) {
  spdlog::trace("Displaying cell complex vertices");
  // INFO: Display vertices
  auto vertices = CloudPtr(new Cloud);
  vertices->points.resize(cc->num_vertices());
  for (auto vit = cc->vertices_begin(); vit != cc->vertices_end(); ++vit) {
    const auto id = (*cc)[*vit].id;
    const auto pos = (*cc)[*vit].pos;
    vertices->points[id].x = pos[0];
    vertices->points[id].y = pos[1];
    vertices->points[id].z = pos[2];
  }
  const std::string v_name = fmt::format("{}_vertices", name);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> v_color_handler(
      vertices, 0, 0, 255);
  viewer->addPointCloud<PointT>(vertices, v_color_handler, v_name, vp);
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, v_name, vp);

  // INFO: Display faces
  spdlog::trace("Displaying cell complex faces");
  auto faces = CloudPtr(new Cloud);
  faces->points.resize(cc->num_faces());
  for (auto fit = cc->faces_begin(); fit != cc->faces_end(); ++fit) {
    const auto id = (*cc)[*fit].id;
    const auto pos = (*cc)[*fit].pos;
    faces->points[id].x = pos[0];
    faces->points[id].y = pos[1];
    faces->points[id].z = pos[2];
  }
  const std::string f_name = fmt::format("{}_faces", name);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> f_color_handler(
      faces, 0, 255, 0);
  viewer->addPointCloud<PointT>(faces, f_color_handler, f_name, vp);
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, f_name, vp);

  // INFO: Display cell centers
  spdlog::trace("Displaying cell complex cell centers");
  auto cells = CloudPtr(new Cloud);
  cells->points.resize(cc->num_cells());
  for (auto cit = cc->cells_begin(); cit != cc->cells_end(); ++cit) {
    const auto id = (*cc)[*cit].id;
    const auto pos = (*cc)[*cit].pos;
    cells->points[id].x = pos[0];
    cells->points[id].y = pos[1];
    cells->points[id].z = pos[2];
  }
  pcl::visualization::PointCloudColorHandlerCustom<PointT> c_color_handler(
      cells, 255, 0, 0);
  const std::string c_name = fmt::format("{}_centers", name);
  viewer->addPointCloud<PointT>(cells, c_color_handler, c_name, vp);
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, c_name, vp);

  // INFO: Display Cell Face Correspondences
  spdlog::trace("Displaying cell face correspondences");
  std::string cf_name = "correspondences_cf";
  pcl::CorrespondencesPtr cf(new pcl::Correspondences);
  for (auto cit = cc->cells_begin(); cit != cc->cells_end(); ++cit) {
    const auto cid = (*cc)[*cit].id;
    for (auto fit = cc->faces_begin(*cit); fit != cc->faces_end(*cit); ++fit) {
      const auto fid = (*cc)[*fit].id;
      cf->emplace_back(static_cast<int>(cid), static_cast<int>(fid), 0.0);
    }
  }
  viewer->addCorrespondences<PointT>(cells, faces, *cf, cf_name, vp);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                      1.0, 1.0, 1.0, cf_name, vp);

  // INFO: Display Face Vertex Correspondences
  spdlog::trace("Displaying face vertex correspondences");
  std::string fv_name = "correspondences_fv";
  pcl::CorrespondencesPtr fv(new pcl::Correspondences);
  for (auto fit = cc->faces_begin(); fit != cc->faces_end(); ++fit) {
    const auto fid = (*cc)[*fit].id;
    for (auto vit = cc->vertices_begin(*fit); vit != cc->vertices_end(*fit);
         ++vit) {
      const auto vid = (*cc)[*vit].id;
      fv->emplace_back(static_cast<int>(fid), static_cast<int>(vid), 0.0);
    }
  }
  viewer->addCorrespondences<PointT>(faces, vertices, *fv, fv_name, vp);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                      0.5, 0.5, 0.5, fv_name, vp);
}

void addRoomProbabilities(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const std::shared_ptr<ReUseX::geometry::CellComplex> &cc,
    const std::string_view &name, int vp) {
  spdlog::trace("Displaying room probabilities");
  auto c_rp = cc->property_map<ReUseX::geometry::CellComplex::Vertex,
                               std::vector<double>>("c:room_probabilities");
  // Create a Lut for the room labels
  std::vector<pcl::RGB> lut(cc->n_rooms + 1);
  for (size_t i = 0; i < lut.size(); ++i)
    lut[i] = pcl::GlasbeyLUT::at(i);

  for (auto cit = cc->cells_begin(); cit != cc->cells_end(); ++cit) {
    const auto id = (*cc)[*cit].id;
    const auto &vec = c_rp[*cit];

    // Mix the colors based on the probabilities
    pcl::RGB color{0, 0, 0};
    for (size_t i = 0; i < vec.size(); ++i) {
      const auto c = lut[i];
      color.r += static_cast<uint8_t>(c.r * vec[i]);
      color.g += static_cast<uint8_t>(c.g * vec[i]);
      color.b += static_cast<uint8_t>(c.b * vec[i]);
    }
    // color.r /= static_cast<uint8_t>(vec.size());
    // color.g /= static_cast<uint8_t>(vec.size());
    // color.b /= static_cast<uint8_t>(vec.size());

    std::string name_ = fmt::format("{}_cell_{}-prob", name, id);
    PointT p;
    p.x = (*cc)[*cit].pos[0];
    p.y = (*cc)[*cit].pos[1];
    p.z = (*cc)[*cit].pos[2];
    // TODO: Scale sphere size based on if its a room or not
    auto prob_outside = vec[0];
    double r = 0.4 * (1.0 - prob_outside) + 0.1;

    if (prob_outside > 0.9)
      continue; // Skip mostly outside cells

    viewer->addSphere(p, r, static_cast<double>(color.r) / 255.0,
                      static_cast<double>(color.g) / 255.0,
                      static_cast<double>(color.b) / 255.0, name_, vp);
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, name_, vp);

    // PointT p = cell_centers->points[id];
    // viewer->addText3D(fmt::format("C{}:R{}", id, prob_str), p,
    // 0.05, 1.0, 1.0, 1.0, name_);
  }
}

void addSupportProbabilities(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const std::shared_ptr<ReUseX::geometry::CellComplex> &cc,
    const std::string_view &name, int vp) {
  spdlog::trace("Displaying face support probabilities");
  auto vertices = CloudPtr(new Cloud);
  vertices->points.resize(cc->num_vertices());
  for (auto vit = cc->vertices_begin(); vit != cc->vertices_end(); ++vit) {
    const auto id = (*cc)[*vit].id;
    const auto pos = (*cc)[*vit].pos;
    vertices->points[id].x = pos[0];
    vertices->points[id].y = pos[1];
    vertices->points[id].z = pos[2];
  }

  auto f_sp = cc->property_map<ReUseX::geometry::CellComplex::Vertex, double>(
      "f:support_probability");
  for (auto fit = cc->faces_begin(); fit != cc->faces_end(); ++fit) {
    const auto id = (*cc)[*fit].id;
    const auto prob = f_sp[*fit] == -1.0 ? 1.0 : f_sp[*fit];

    const auto c = f_sp[*fit] == -1.0 ? pcl::RGB(255, 0, 0)
                                      : pcl::ViridisLUT::at(static_cast<size_t>(
                                            prob * pcl::ViridisLUT::size()));

    std::string name_ = fmt::format("{}_face_{}-prob", name, id);

    // Create face
    pcl::Vertices face{};
    for (auto vit = cc->vertices_begin(*fit); vit != cc->vertices_end(*fit);
         ++vit)
      face.vertices.push_back(static_cast<int>((*cc)[*vit].id));

    // Close the face
    if (!face.vertices.empty())
      face.vertices.push_back(face.vertices[0]);
    else
      continue;

    // std::reverse(face.vertices.begin(), face.vertices.end());
    // ranges::reverse(face.vertices);

    viewer->addPolygonMesh<PointT>(vertices, {face}, name_, vp);
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR,
        static_cast<double>(c.r) / 255.0, static_cast<double>(c.g) / 255.0,
        static_cast<double>(c.b) / 255.0, name_, vp);

    auto opacity = f_sp[*fit] == -1.0 ? 1.0 : prob * 0.8 + 0.2;
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, name_, vp);

    if (f_sp[*fit] != -1.0)
      continue; // Only display text for unsupported faces

    auto plane_id = std::get<FaceData>((*cc)[*fit].data).plane_id;

    viewer->addText3D(fmt::format("P{}", plane_id),
                      PointT{static_cast<float>((*cc)[*fit].pos[0]),
                             static_cast<float>((*cc)[*fit].pos[1]),
                             static_cast<float>((*cc)[*fit].pos[2])},
                      0.3, 1.0, 1.0, 1.0, fmt::format("text_face_{}", id), vp);
  }
}

void addRooms(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const std::shared_ptr<ReUseX::geometry::CellComplex> &cc,
    const std::pair<
        std::unordered_map<ReUseX::geometry::CellComplex::Vertex, int>,
        std::unordered_map<ReUseX::geometry::CellComplex::Vertex,
                           std::set<int>>> &results,
    const std::string_view &name, int vp) {
  spdlog::trace("Displaying results");

  // TODO: Make view base constructor form CellComplex
  auto points = CloudPtr(new Cloud);
  points->points.resize(cc->num_vertices());
  for (auto vit = cc->vertices_begin(); vit != cc->vertices_end(); ++vit) {
    const auto id = (*cc)[*vit].id;
    const auto pos = (*cc)[*vit].pos;
    points->points[id].x = pos[0];
    points->points[id].y = pos[1];
    points->points[id].z = pos[2];
  }

  auto [res_room_labels, res_wall_labels] = results;

  for (auto cit = cc->cells_begin(); cit != cc->cells_end(); ++cit) {
    const auto id = (*cc)[*cit].id;
    const auto pos = (*cc)[*cit].pos;
    const auto name_ = fmt::format("{}_cell_{}", name, id);
    const auto label = res_room_labels[*cit];
    const auto color = pcl::GlasbeyLUT::at(label);

    PointT c;
    c.x = pos[0];
    c.y = pos[1];
    c.z = pos[2];

    switch (label) {
    case 0:
      viewer->addText3D(fmt::format("R{} W[{}]", label,
                                    fmt::join(res_wall_labels[*cit], ",")),
                        c, 0.1, 1.0, 1.0, 1.0, name_, vp);
      break;
    default:
      std::vector<pcl::Vertices> faces{};
      size_t count = 0;

      for (auto fit = cc->faces_begin(*cit); fit != cc->faces_end(*cit);
           ++fit, ++count) {
        pcl::Vertices face{};

        for (auto vit = cc->vertices_begin(*fit); vit != cc->vertices_end(*fit);
             ++vit) {
          const auto vid = (*cc)[*vit].id;
          face.vertices.push_back(static_cast<int>(vid));
        }

        // Close the face
        if (!face.vertices.empty())
          face.vertices.push_back(face.vertices[0]);

        faces.push_back(face);
      }
      viewer->addPolygonMesh<PointT>(points, faces, name_, vp);
      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR,
          static_cast<double>(color.r) / 255.0,
          static_cast<double>(color.g) / 255.0,
          static_cast<double>(color.b) / 255.0, name_, vp);
    }
  }
}
} // namespace ReUseX::visualize
