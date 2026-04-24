// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "io/export_scene.hpp"
#include "core/logging.hpp"

#include <reusex/core/MaterialPassport.hpp>
#include <reusex/core/ProjectDB.hpp>

#include <pcl/common/colors.h>

#include <cstdlib>
#include <regex>

namespace reusex::io {

namespace {

/// Parse instance label_definitions format: "SM{semantic_class}-{instance_id}
/// ({size}p)"
/// Returns the semantic class ID, or -1 on failure.
int parse_instance_semantic_class(const std::string &def) {
  // Match "SM<digits>-<digits> (<digits>p)"
  static const std::regex re(R"(SM(\d+)-\d+\s+\(\d+p\))");
  std::smatch match;
  if (std::regex_match(def, match, re))
    return std::stoi(match[1].str());
  return -1;
}

/// Extract translation from a row-major 4x4 SE(3) matrix.
void extract_position(const std::array<double, 16> &pose, double &x, double &y,
                      double &z) {
  x = pose[3];
  y = pose[7];
  z = pose[11];
}

} // namespace

ExportScene gather_export_scene(const ProjectDB &db) {
  ExportScene scene;

  // --- Cloud layer ---
  if (db.has_point_cloud("cloud")) {
    reusex::debug("ExportScene: loading 'cloud'");
    ExportScene::CloudLayer layer;
    layer.cloud = db.point_cloud_xyzrgb("cloud");

    if (db.has_point_cloud("normals"))
      layer.normals = db.point_cloud_normal("normals");

    scene.cloud = std::move(layer);
    reusex::debug("ExportScene: cloud has {} points",
                  scene.cloud->cloud->size());
  }

  // --- Semantic layer ---
  if (db.has_point_cloud("instances") && db.has_point_cloud("cloud")) {
    // Instance-based export
    reusex::debug("ExportScene: loading instance-based semantic data");

    auto instance_labels = db.point_cloud_label("instances");
    auto instance_defs = db.label_definitions("instances");

    // Get semantic name mapping from "labels" definitions
    std::map<int, std::string> semantic_names;
    if (db.has_point_cloud("labels"))
      semantic_names = db.label_definitions("labels");

    auto cloud = db.point_cloud_xyzrgb("cloud");

    // Group instance IDs by semantic class
    // instance_id -> semantic_class_id
    std::map<uint32_t, int> instance_to_semantic;
    for (const auto &[inst_id, def] : instance_defs) {
      int sem_class = parse_instance_semantic_class(def);
      if (sem_class >= 0)
        instance_to_semantic[static_cast<uint32_t>(inst_id)] = sem_class;
    }

    // semantic_class_id -> list of instance_ids
    std::map<int, std::vector<uint32_t>> semantic_to_instances;
    for (const auto &[inst_id, sem_class] : instance_to_semantic)
      semantic_to_instances[sem_class].push_back(inst_id);

    // Build per-instance point clouds
    // instance_id -> CloudPtr
    std::map<uint32_t, CloudPtr> instance_clouds;
    for (size_t i = 0; i < instance_labels->size(); ++i) {
      uint32_t inst = instance_labels->points[i].label;
      if (inst == 0)
        continue; // skip unlabeled
      if (instance_clouds.find(inst) == instance_clouds.end())
        instance_clouds[inst] = CloudPtr(new Cloud());
      instance_clouds[inst]->points.push_back(cloud->points[i]);
    }

    // Build SemanticCategory entries
    for (const auto &[sem_class, inst_ids] : semantic_to_instances) {
      ExportScene::SemanticCategory cat;
      cat.label_id = sem_class;

      // Look up name from "labels" label_definitions
      auto it = semantic_names.find(sem_class);
      cat.name = (it != semantic_names.end()) ? it->second
                                              : "class_" + std::to_string(sem_class);

      // Glasbey color
      auto c =
          pcl::GlasbeyLUT::at(sem_class % pcl::GlasbeyLUT::size());
      cat.color = {c.r, c.g, c.b};

      // Add instances
      for (uint32_t inst_id : inst_ids) {
        auto cloud_it = instance_clouds.find(inst_id);
        if (cloud_it == instance_clouds.end() ||
            cloud_it->second->empty())
          continue;

        ExportScene::SemanticInstance si;
        si.instance_id = inst_id;
        si.points = cloud_it->second;
        cat.instances.push_back(std::move(si));
      }

      if (!cat.instances.empty())
        scene.semantic.push_back(std::move(cat));
    }

    reusex::debug("ExportScene: {} semantic categories (instance-based)",
                  scene.semantic.size());

  } else if (db.has_point_cloud("labels") && db.has_point_cloud("cloud")) {
    // Label-based export (no instances)
    reusex::debug("ExportScene: loading label-based semantic data");

    auto labels = db.point_cloud_label("labels");
    auto label_defs = db.label_definitions("labels");
    auto cloud = db.point_cloud_xyzrgb("cloud");

    // Group points by label
    std::map<uint32_t, CloudPtr> label_clouds;
    for (size_t i = 0; i < labels->size(); ++i) {
      uint32_t lbl = labels->points[i].label;
      if (lbl == 0)
        continue; // skip unlabeled/background
      if (label_clouds.find(lbl) == label_clouds.end())
        label_clouds[lbl] = CloudPtr(new Cloud());
      label_clouds[lbl]->points.push_back(cloud->points[i]);
    }

    for (const auto &[lbl, lbl_cloud] : label_clouds) {
      ExportScene::SemanticCategory cat;
      cat.label_id = static_cast<int>(lbl);

      auto it = label_defs.find(static_cast<int>(lbl));
      cat.name = (it != label_defs.end()) ? it->second
                                          : "class_" + std::to_string(lbl);

      auto c = pcl::GlasbeyLUT::at(lbl % pcl::GlasbeyLUT::size());
      cat.color = {c.r, c.g, c.b};

      ExportScene::SemanticInstance si;
      si.instance_id = 0;
      si.points = lbl_cloud;
      cat.instances.push_back(std::move(si));

      scene.semantic.push_back(std::move(cat));
    }

    reusex::debug("ExportScene: {} semantic categories (label-based)",
                  scene.semantic.size());
  }

  // --- Mesh layer ---
  for (const auto &name : db.list_meshes()) {
    reusex::debug("ExportScene: loading mesh '{}'", name);
    auto mesh = db.mesh(name);
    if (mesh && !mesh->polygons.empty())
      scene.meshes.push_back({name, mesh});
  }

  // --- 360 Panoramas ---
  for (const auto &pano : db.list_panoramic_images()) {
    ExportScene::PanoEntry entry;
    entry.image_name = pano.filename;

    if (pano.node_id >= 0 && db.has_sensor_frame(pano.node_id)) {
      auto pose = db.sensor_frame_pose(pano.node_id);
      extract_position(pose, entry.x, entry.y, entry.z);
    }

    scene.panoramas.push_back(std::move(entry));
  }

  if (!scene.panoramas.empty())
    reusex::debug("ExportScene: {} panoramic images", scene.panoramas.size());

  // --- Materials ---
  for (const auto &passport : db.all_material_passports()) {
    ExportScene::MaterialEntry entry;
    entry.name = passport.description.designation.empty()
                     ? passport.metadata.document_guid
                     : passport.description.designation;

    // Try to get stored properties
    try {
      entry.properties =
          db.passport_stored_properties(passport.metadata.document_guid);
    } catch (...) {
      // No stored properties - that's OK
    }

    scene.materials.push_back(std::move(entry));
  }

  if (!scene.materials.empty())
    reusex::debug("ExportScene: {} material passports", scene.materials.size());

  return scene;
}

} // namespace reusex::io
