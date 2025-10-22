// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "rux/export.hpp"
#include "ReUseX/Yolo.hpp"
#include "ReUseX/fmt_formatter.hpp"
#include "spdmon/spdmon.hpp"

#include <fmt/format.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <pcl/common/colors.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>

#include <opennurbs_public.h>

#include <opennurbs_layer.h>
#include <opennurbs_pointcloud.h>

#include <filesystem>
#include <ranges>
namespace fs = std::filesystem;
using namespace ReUseX;

void setup_subcommand_export(CLI::App &app) {

  auto opt = std::make_shared<SubcommandExportOptions>();
  auto *sub = app.add_subcommand(
      "export",
      "This tool exports a rtab-map database to a pcl point cloud for "
      "use in reusex.");

  sub->add_option("cloud", opt->cloud_path_in,
                  "Path to the input point cloud file.")
      ->required()
      ->check(CLI::ExistingFile);

  sub->add_option("labels", opt->labels_path_in,
                  "Path to the input point cloud file.")
      ->required()
      ->check(CLI::ExistingFile);

  sub->add_option("output", opt->path_out,
                  "Path to the output point cloud file")
      ->default_val(opt->path_out);

  sub->callback([opt]() {
    spdlog::trace("calling export subcommand");
    return run_subcommand_export(*opt);
  });
}

int run_subcommand_export(SubcommandExportOptions const &opt) {

  spdlog::trace("creating rhino model");
  ONX_Model model;

  // Properties
  spdlog::trace("setting up model properties");
  model.m_properties.m_RevisionHistory.NewRevision();
  model.m_properties.m_Application.m_application_name = "ReUseX";
  model.m_properties.m_Application.m_application_URL =
      L"https://github.com/pfmephisto/ReUseX";
  model.m_properties.m_Application.m_application_details =
      "ReUseX Export annotated pcl point cloud to Rhino 3dm file";
  model.m_properties.m_Notes.m_notes =
      "This file was made with the ReUseX Export tool";
  model.m_properties.m_Notes.m_bVisible = true;
  // model.m_sStartSectionComments = "ReUseX Export";

  // Settings
  spdlog::trace("setting up model settings");
  model.m_settings.m_ModelUnitsAndTolerances.m_unit_system =
      ON::LengthUnitSystem::Meters;
  model.m_settings.m_ModelUnitsAndTolerances.m_absolute_tolerance = 0.01;
  model.m_settings.m_ModelUnitsAndTolerances.m_angle_tolerance = ON_PI / 180.0;
  model.m_settings.m_ModelUnitsAndTolerances.m_relative_tolerance = 0.01;

  spdlog::trace("load pcl point cloud from file: {}", opt.cloud_path_in);
  CloudPtr pcl_cloud(new Cloud);
  pcl::io::loadPCDFile<PointT>(opt.cloud_path_in.c_str(), *pcl_cloud);

  spdlog::trace("load pcl labels from file: {}", opt.labels_path_in);
  CloudLPtr pcl_labels(new CloudL);
  pcl::io::loadPCDFile<LabelT>(opt.labels_path_in.c_str(), *pcl_labels);

  if (pcl_cloud->empty()) {
    spdlog::error("Point cloud is empty, nothing to export.");
    return 1;
  }

  if (pcl_cloud->size() != pcl_labels->size()) {
    spdlog::error(
        "Point cloud and labels have different sizes (cloud: {}, labels: "
        "{})",
        pcl_cloud->size(), pcl_labels->size());
    return 1;
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
  auto base_layer_id = model.AddLayer(base_layer_name, ON_Color::Black);
  const ON_Layer *base_layer =
      ON_Layer::Cast(model.LayerFromIndex(base_layer_id).ModelComponent());

  std::vector<int> layer_map(labels.size(), ON_UNSET_INT_INDEX);
  for (size_t i = 0; i < labels.size(); i++) {
    ON_wString name;
    // name.Format(L"Semantic id: %d", labels[i]);
    std::string class_name;
    if (labels[i] == 0) {
      class_name = "unlabeled";
    } else {
      class_name = Yolov8Seg::GetClassName(labels[i] - 1);
    }

    name.Format(L"Semantic class: %d (%s)", labels[i], class_name.c_str());

    auto c = pcl::GlasbeyLUT::at(labels[i] % pcl::GlasbeyLUT::size());
    ON_Color color(c.r, c.g, c.b);
    layer_map[i] = model.AddLayer(name, color);

    ON_Layer *layer = const_cast<ON_Layer *>(
        ON_Layer::Cast(model.LayerFromIndex(layer_map[i]).ModelComponent()));
    layer->SetParentLayerId(base_layer->Id());
  }

  spdlog::trace("creating rhino point clouds");
  for (int i = 0; i < (int)clouds.size(); i++) {
    size_t num_points = clouds[i]->points.size();

    ON_PointCloud *rhino_cloud = new ON_PointCloud();

    // INFO: Reserve the memory for the point cloud
    rhino_cloud->m_P.Reserve(num_points);
    rhino_cloud->m_C.Reserve(num_points);
    rhino_cloud->m_N.Reserve(num_points);

    // INFO: Set the point cloud properties
    rhino_cloud->m_P.SetCount(num_points);
    rhino_cloud->m_C.SetCount(num_points);
    rhino_cloud->m_N.SetCount(num_points);

    // INFO: Adding points to the rhino point cloud
#pragma omp parallel for shared(rhino_cloud, clouds, i)
    for (int j = 0; j < (int)num_points; j++) {
      ON_3dPoint pt(clouds[i]->points[j].x, clouds[i]->points[j].y,
                    clouds[i]->points[j].z);
      ON_Color color(clouds[i]->points[j].r, clouds[i]->points[j].g,
                     clouds[i]->points[j].b);
      ON_3dVector dir(0.0, 0.0, 1.0); // Default normal direction

      rhino_cloud->m_P[j] = pt;
      rhino_cloud->m_C[j] = color;
      rhino_cloud->m_N[j] = dir; // Default normal direction
    }

    // INFO: Creating attributes for the point cloud
    ON_3dmObjectAttributes *attribute = new ON_3dmObjectAttributes();
    // TODO: Add actual label name to the attribute name
    // See Layer names for reference
    attribute->m_name = L"ReUseX Point Cloud Semantic"; // + labels[i];
    attribute->m_layer_index = layer_map[i];
    attribute->m_color = ON_Color::UnsetColor;

    // INFO: Adding the point cloud to the model
    model.AddManagedModelGeometryComponent(rhino_cloud, attribute);
  }

  spdlog::trace("writeing model to file: {}", opt.path_out.string());
  int version = 0;
  // const char *comment = __FILE__ "write_layers_example()" __DATE__;
  if (!model.Write(opt.path_out.c_str(), version
                   /*, comment, &error_log*/)) {
    spdlog::error("Failed to write model to file: {}", opt.path_out.string());
    return 1;
  }
  spdlog::info("Model successfully written to {}", opt.path_out.string());

  return 0;
}
