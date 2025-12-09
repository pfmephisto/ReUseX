// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ReUseX/io/rhino.hpp"

#include <range/v3/to_container.hpp>
#include <range/v3/view/common.hpp>
#include <range/v3/view/transform.hpp>

namespace ReUseX::io {

auto configure_rhino_model() -> std::unique_ptr<ONX_Model> {

  auto model = std::make_unique<ONX_Model>();
  // Properties
  spdlog::trace("setting up model properties");
  model->m_properties.m_RevisionHistory.NewRevision();
  model->m_properties.m_Application.m_application_name = "ReUseX";
  model->m_properties.m_Application.m_application_URL =
      L"https://github.com/pfmephisto/ReUseX";
  model->m_properties.m_Application.m_application_details =
      "ReUseX Export annotated pcl point cloud to Rhino 3dm file";
  model->m_properties.m_Notes.m_notes =
      "This file was made with the ReUseX Export tool";
  model->m_properties.m_Notes.m_bVisible = true;
  // model.m_sStartSectionComments = "ReUseX Export";

  // Settings
  spdlog::trace("setting up model settings");
  model->m_settings.m_ModelUnitsAndTolerances.m_unit_system =
      ON::LengthUnitSystem::Meters;
  model->m_settings.m_ModelUnitsAndTolerances.m_absolute_tolerance = 0.01;
  model->m_settings.m_ModelUnitsAndTolerances.m_angle_tolerance = ON_PI / 180.0;
  model->m_settings.m_ModelUnitsAndTolerances.m_relative_tolerance = 0.01;

  return model;
}

auto make_rhino_pointcloud(CloudConstPtr cloud)
    -> std::unique_ptr<ON_PointCloud> {

  const size_t num_points = cloud->points.size();
  auto rhino_cloud = std::make_unique<ON_PointCloud>();

  // INFO: Reserve the memory for the point cloud
  rhino_cloud->m_P.Reserve(num_points);
  rhino_cloud->m_C.Reserve(num_points);
  rhino_cloud->m_N.Reserve(num_points);

  // INFO: Set the point cloud properties
  rhino_cloud->m_P.SetCount(num_points);
  rhino_cloud->m_C.SetCount(num_points);
  rhino_cloud->m_N.SetCount(num_points);

  // INFO: Adding points to the rhino point cloud
#pragma omp parallel for shared(rhino_cloud, cloud)
  for (int j = 0; j < (int)num_points; j++) {
    ON_3dPoint pt(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z);
    ON_Color color(cloud->points[j].r, cloud->points[j].g, cloud->points[j].b);
    ON_3dVector dir(0.0, 0.0, 1.0); // Default normal direction

    rhino_cloud->m_P[j] = pt;
    rhino_cloud->m_C[j] = color;
    rhino_cloud->m_N[j] = dir; // Default normal direction
  }

  return rhino_cloud;
}

auto create_rhino_layers(ONX_Model &model,
                         const std::vector<std::string> &layer_names,
                         std::optional<std::vector<ON_Color>> layer_colors,
                         const ON_Layer *base_layer) -> std::vector<int> {
  spdlog::trace("creating rhino layers: {}", fmt::join(layer_names, ", "));

  std::vector<int> layer_map(layer_names.size(), ON_UNSET_INT_INDEX);

  std::vector<ON_Color> colors(layer_names.size(), ON_Color::Black);
  if (layer_colors)
    colors = *layer_colors;

  for (size_t i = 0; i < layer_names.size(); i++) {
    spdlog::trace("  creating layer: {}", layer_names[i]);
    spdlog::trace("    color: R={}, G={}, B={}", colors[i].Red(),
                  colors[i].Green(), colors[i].Blue());
    ON_wString name(layer_names[i].c_str());
    layer_map[i] = model.AddLayer(name, colors[i]);

    ON_Layer *layer = const_cast<ON_Layer *>(
        ON_Layer::Cast(model.LayerFromIndex(layer_map[i]).ModelComponent()));

    if (base_layer)
      layer->SetParentLayerId(base_layer->Id());
  }
  return layer_map;
}

auto save_rhino_pointcloud(CloudConstPtr pcl_cloud, CloudLConstPtr pcl_labels)
    -> std::unique_ptr<ONX_Model> {

  auto model = configure_rhino_model();

  if (pcl_cloud->size() != pcl_labels->size()) {
    spdlog::error(
        "Point cloud and labels have different sizes (cloud: {}, labels: "
        "{})",
        pcl_cloud->size(), pcl_labels->size());
    return model;
  }

  spdlog::trace("creating set of labels");
  std::set<uint32_t> labels_set{};
  for (const auto &point : pcl_labels->points)
    labels_set.insert(point.label);

  spdlog::trace("creating sorted vector of labels");
  std::vector<uint32_t> labels(labels_set.begin(), labels_set.end());
  std::sort(labels.begin(), labels.end());

  spdlog::debug("Found {} unique labels [{}]", labels.size(),
                fmt::join(labels, ", "));

  std::vector<CloudPtr, Eigen::aligned_allocator<CloudPtr>> clouds(
      labels.size());
  spdlog::trace("filter points by labels");
  for (int i = 0; i < (int)labels.size(); i++) {
    clouds[i] = CloudPtr(new Cloud());
    for (int j = 0; j < (int)pcl_cloud->points.size(); j++) {
      if (pcl_labels->points[j].label == labels[i])
        clouds[i]->points.push_back(pcl_cloud->points[j]);
    }
  }
  std::vector<size_t> sizes(clouds.size());
  std::transform(clouds.begin(), clouds.end(), sizes.begin(),
                 [](const CloudPtr &c) { return c->size(); });
  spdlog::debug("Filtered {} clouds of sizes [{}]", clouds.size(),
                fmt::join(sizes, ", "));

  spdlog::trace("creating document layers");
  // model.AddDefaultLayer(nullptr, ON_Color::UnsetColor);
  ON_wString base_layer_name = L"ReUseX Export";
  auto base_layer_id = model->AddLayer(base_layer_name, ON_Color::Black);
  spdlog::trace("base layer id: {}", base_layer_id);
  const ON_Layer *base_layer =
      ON_Layer::Cast(model->LayerFromIndex(base_layer_id).ModelComponent());
  spdlog::trace("base layer pointer: {}", (void *)base_layer);

  auto layer_names =
      labels | ranges::views::transform([](uint32_t label) {
        spdlog::trace("label: {}", label);
        return label == static_cast<uint32_t>(-1)
                   ? "Semantic class: 0 (unlabeled)"
                   : fmt::format("Semantic class: {} ({})", label,
                                 ReUseX::vision::Yolov8_className[label]);
      }) |
      ranges::to<std::vector<std::string>>();
  spdlog::trace("layer names: {}", fmt::join(layer_names, ", "));
  auto colors = labels | ranges::views::transform([](uint32_t label) {
                  auto c = pcl::GlasbeyLUT::at(label % pcl::GlasbeyLUT::size());
                  return ON_Color(c.r, c.g, c.b);
                }) |
                ranges::to<std::vector<ON_Color>>();
  spdlog::trace("layer colors:");
  for (const auto &color : colors) {
    spdlog::trace("  R={}, G={}, B={}", color.Red(), color.Green(),
                  color.Blue());
  }

  auto layer_map = create_rhino_layers(*model, layer_names, colors, base_layer);

  spdlog::trace("creating rhino point clouds");
  for (int i = 0; i < (int)clouds.size(); i++) {

    auto rhino_cloud = make_rhino_pointcloud(clouds[i]);

    // INFO: Creating attributes for the point cloud
    ON_3dmObjectAttributes *attribute = new ON_3dmObjectAttributes();
    // TODO: Add actual label name to the attribute name
    // See Layer names for reference
    attribute->m_name = L"ReUseX Point Cloud Semantic"; // + labels[i];
    attribute->m_layer_index = layer_map[i];
    attribute->m_color = ON_Color::UnsetColor;

    // INFO: Adding the point cloud to the model
    model->AddManagedModelGeometryComponent(rhino_cloud.release(), attribute);
  }

  return model;
}
} // namespace ReUseX::io
