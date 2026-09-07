// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "view/project_loader.hpp"
#include "filter_utils.hpp"

#include <reusex/core/ProjectDB.hpp>

#include <fmt/format.h>
#include <fmt/std.h>
#include <pcl/filters/extract_indices.h>
#include <spdlog/spdlog.h>

#include <set>

namespace rux::view {

namespace {

/// Create XYZL cloud by merging geometry and label data.
///
/// @param labels Label cloud (must match geometry size)
/// @param geometry Geometry cloud (XYZ+RGB)
/// @return Combined XYZL cloud
pcl::PointCloud<pcl::PointXYZL>::Ptr
create_label_cloud(const CloudLPtr &labels, const CloudPtr &geometry) {
  auto label_cloud =
      pcl::PointCloud<pcl::PointXYZL>::Ptr(new pcl::PointCloud<pcl::PointXYZL>);
  label_cloud->resize(geometry->size());

  for (size_t i = 0; i < geometry->size(); ++i) {
    label_cloud->points[i].x = geometry->points[i].x;
    label_cloud->points[i].y = geometry->points[i].y;
    label_cloud->points[i].z = geometry->points[i].z;
    label_cloud->points[i].label = labels->points[i].label;
  }

  return label_cloud;
}

} // namespace

ProjectLoadResult load_from_project_db(const fs::path &path,
                                       rux::VizualizationObserver &observer,
                                       const std::string &filter_expr) {
  spdlog::info("Opening project database: {}", path);
  reusex::ProjectDB db(path, /*readOnly=*/true);

  ProjectLoadResult result;
  CloudPtr first_xyzrgb;

  // Evaluate filter expression if provided
  reusex::IndicesPtr filter_indices;
  if (!filter_expr.empty()) {
    spdlog::info("Evaluating filter expression: '{}'", filter_expr);
    try {
      // Determine expected cloud size - load first XYZRGB cloud temporarily
      auto cloud_names = db.list_point_clouds();
      size_t expected_size = 0;
      for (const auto &name : cloud_names) {
        if (db.point_cloud_type(name) == "PointXYZRGB") {
          auto temp_cloud = db.point_cloud_xyzrgb(name);
          expected_size = temp_cloud->size();
          break;
        }
      }

      if (expected_size == 0) {
        spdlog::error("No XYZRGB clouds found for filtering");
        return result;
      }

      filter_indices =
          rux::filters::evaluate_filter(filter_expr, db, expected_size);

      if (filter_indices->empty()) {
        spdlog::warn("Filter matched 0 points - viewer will be empty");
      } else {
        double pct = 100.0 * filter_indices->size() / expected_size;
        spdlog::info("Filter matched {}/{} points ({:.1f}%)",
                     filter_indices->size(), expected_size, pct);
      }
    } catch (const std::exception &e) {
      spdlog::error("Filter evaluation failed: {}", e.what());
      return result;
    }
  }

  // Load point clouds
  auto cloud_names = db.list_point_clouds();
  for (const auto &name : cloud_names) {
    auto type = db.point_cloud_type(name);

    if (type == "PointXYZRGB") {
      try {
        auto cloud = db.point_cloud_xyzrgb(name);

        // Apply filter if provided
        if (filter_indices && !filter_indices->empty()) {
          pcl::ExtractIndices<PointT> extract;
          extract.setInputCloud(cloud);
          extract.setIndices(filter_indices);
          extract.setNegative(false);

          auto filtered_cloud = std::make_shared<Cloud>();
          extract.filter(*filtered_cloud);

          spdlog::debug("Filtered cloud '{}': {} -> {} points", name,
                        cloud->size(), filtered_cloud->size());
          cloud = filtered_cloud;
        }

        if (!first_xyzrgb)
          first_xyzrgb = cloud;

        LoadedCloud loaded;
        loaded.cloud = cloud;
        loaded.name = fmt::format("cloud_{}", result.clouds.size());

        observer.viewer_enqueue_task(
            [loaded](const rux::VizualizationObserver::ViewerPtr &viewer,
                     const std::vector<int> &viewports) {
              viewer->addPointCloud<PointT>(loaded.cloud, loaded.name,
                                            viewports[0]);
            });

        spdlog::info("Loaded cloud '{}' ({} points)", name, cloud->size());
        result.clouds.push_back(std::move(loaded));
      } catch (const std::exception &e) {
        spdlog::error("Failed to load cloud '{}': {}", name, e.what());
      }
    } else if (type == "Label") {
      // Defer — will merge with first XYZRGB cloud below
      spdlog::debug("Found label cloud '{}', will merge after loading", name);
    } else {
      spdlog::debug("Skipping cloud '{}' (type: {})", name, type);
    }
  }

  // Merge label clouds with first XYZRGB cloud
  if (first_xyzrgb) {
    for (const auto &name : cloud_names) {
      auto type = db.point_cloud_type(name);
      if (type != "Label")
        continue;

      try {
        auto labels = db.point_cloud_label(name);

        // Apply filter if provided (before size check)
        if (filter_indices && !filter_indices->empty()) {
          pcl::ExtractIndices<LabelT> extract;
          extract.setInputCloud(labels);
          extract.setIndices(filter_indices);
          extract.setNegative(false);

          auto filtered_labels = std::make_shared<CloudL>();
          extract.filter(*filtered_labels);
          labels = filtered_labels;
        }

        if (labels->size() != first_xyzrgb->size()) {
          spdlog::error(
              "Label cloud '{}' size ({}) does not match first point cloud "
              "size ({})",
              name, labels->size(), first_xyzrgb->size());
          continue;
        }
        auto cloud = create_label_cloud(labels, first_xyzrgb);

        // Build legend entries from unique labels in the cloud
        auto defs = db.label_definitions(name);
        std::set<uint32_t> unique_labels;
        for (const auto &pt : cloud->points)
          unique_labels.insert(pt.label);

        std::vector<LabelLegendEntry> legend;
        for (uint32_t lbl : unique_labels) {
          auto it = defs.find(static_cast<int>(lbl));
          std::string lbl_name =
              it != defs.end() ? it->second : fmt::format("Label {}", lbl);
          pcl::RGB color = pcl::GlasbeyLUT::at(lbl % pcl::GlasbeyLUT::size());
          legend.push_back({static_cast<int>(lbl), std::move(lbl_name), color});
        }

        result.labels.push_back(
            LabelCloudInfo{cloud, std::string(name), std::move(legend)});
        spdlog::info("Loaded label cloud '{}' ({} unique labels)", name,
                     unique_labels.size());
      } catch (const std::exception &e) {
        spdlog::error("Failed to load label cloud '{}': {}", name, e.what());
      }
    }
  }

  // Load meshes
  auto mesh_names = db.list_meshes();
  if (!mesh_names.empty() && filter_indices && !filter_indices->empty()) {
    spdlog::warn(
        "Filter is active but meshes cannot be filtered - showing full meshes");
  }
  for (const auto &name : mesh_names) {

    LoadedMesh loaded;
    loaded.name = fmt::format("mesh_{}", result.meshes.size());
    bool loaded_successfully = false;

    spdlog::debug("Attempting to load mesh '{}'", name);

    // Try textured mesh first (more specific format)
    try {
      auto texture_mesh = db.texture_mesh(name);

      // Check if any material actually has a texture file
      bool has_textures = false;
      for (const auto &mat : texture_mesh->tex_materials) {
        if (!mat.tex_file.empty()) {
          has_textures = true;
          break;
        }
      }

      if (has_textures) {
        // Render as textured mesh
        loaded.mesh = texture_mesh;
        observer.viewer_enqueue_task(
            [loaded](const rux::VizualizationObserver::ViewerPtr &viewer,
                     const std::vector<int> &viewports) {
              auto textured = std::get<pcl::TextureMesh::Ptr>(loaded.mesh);
              viewer->addTextureMesh(*textured, loaded.name, viewports[0]);
            });
        spdlog::info("Loaded textured mesh '{}' ({} materials) - rendering "
                     "with textures",
                     name, texture_mesh->tex_materials.size());
      } else {
        // No valid textures — render as plain polygon mesh
        auto plain = std::make_shared<pcl::PolygonMesh>();
        plain->cloud = texture_mesh->cloud;
        for (const auto &group : texture_mesh->tex_polygons)
          plain->polygons.insert(plain->polygons.end(), group.begin(),
                                 group.end());
        loaded.mesh = plain;
        observer.viewer_enqueue_task(
            [loaded](const rux::VizualizationObserver::ViewerPtr &viewer,
                     const std::vector<int> &viewports) {
              auto regular = std::get<pcl::PolygonMesh::Ptr>(loaded.mesh);
              viewer->addPolygonMesh(*regular, loaded.name, viewports[0]);
            });
        spdlog::info(
            "Loaded mesh '{}' (no valid textures, rendering as plain mesh)",
            name);
      }

      result.meshes.push_back(std::move(loaded));
      loaded_successfully = true;
    } catch (const std::exception &e) {
      std::string error_msg(e.what());
      // Check if it's a format mismatch (not a textured mesh) vs parsing error
      if (error_msg.find("is not a texture mesh") != std::string::npos) {
        // Format mismatch - try regular mesh loader
        spdlog::trace("Mesh '{}' is not textured format, trying regular loader",
                      name);
      } else {
        // Actual parsing/loading error - don't try regular loader
        spdlog::error("Failed to load textured mesh '{}': {}", name, e.what());
        loaded_successfully =
            true; // Mark as "attempted" to skip regular loader
        continue;
      }
    }

    // Try regular mesh only if it's not a textured mesh
    if (!loaded_successfully) {
      try {
        auto mesh = db.mesh(name);
        loaded.mesh = mesh;

        observer.viewer_enqueue_task(
            [loaded](const rux::VizualizationObserver::ViewerPtr &viewer,
                     const std::vector<int> &viewports) {
              auto regular = std::get<pcl::PolygonMesh::Ptr>(loaded.mesh);
              viewer->addPolygonMesh(*regular, loaded.name, viewports[0]);
            });

        spdlog::info("Loaded mesh '{}' ({} polygons)", name,
                     mesh->polygons.size());
        result.meshes.push_back(std::move(loaded));
      } catch (const std::exception &e) {
        spdlog::error("Failed to load regular mesh '{}': {}", name, e.what());
      }
    }

    // try {
    //   auto mesh = db.mesh(name);
    //   LoadedMesh loaded;
    //   loaded.mesh = mesh;
    //   loaded.name = fmt::format("mesh_{}", result.meshes.size());
    //   observer.viewer_enqueue_task(
    //       [loaded](const rux::VizualizationObserver::ViewerPtr &viewer,
    //                const std::vector<int> &viewports) {
    //         viewer->addPolygonMesh(*loaded.mesh, loaded.name, viewports[0]);
    //       });
    //   spdlog::info("Loaded mesh '{}' ({} polygons)", name,
    //                mesh->polygons.size());
    //   result.meshes.push_back(std::move(loaded));
    // } catch (const std::exception &e) {
    //   spdlog::error("Failed to load mesh '{}': {}", name, e.what());
    // }
  }

  // Load building components (windows, doors, etc.)
  auto component_names = db.list_building_components();
  for (const auto &name : component_names) {
    try {
      auto comp = db.building_component(name);
      spdlog::info("Loaded component '{}' ({}, {} vertices)", name,
                   to_string(comp.type), comp.boundary.vertices.size());
      result.components.push_back(std::move(comp));
    } catch (const std::exception &e) {
      spdlog::error("Failed to load component '{}': {}", name, e.what());
    }
  }

  // Load panoramic image metadata (images loaded on-demand)
  try {
    auto pano_list = db.list_panoramic_images();
    for (const auto &pano : pano_list) {
      PanoramaInfo pi;
      pi.id = pano.id;
      pi.filename = pano.filename;
      pi.node_id = pano.node_id;

      // Prefer the content-aligned pose (`rux align 360`) when present; fall
      // back to the timestamp-matched sensor frame's pose.
      if (pano.has_pose) {
        pi.pose = pano.pose;
        pi.px = pi.pose[3]; // row-major: translation is [3], [7], [11]
        pi.py = pi.pose[7];
        pi.pz = pi.pose[11];
        pi.pose_valid = true;
        spdlog::debug("Panorama '{}' (aligned) at ({:.2f}, {:.2f}, {:.2f})",
                      pi.filename, pi.px, pi.py, pi.pz);
      } else if (pano.node_id >= 0 && db.has_sensor_frame(pano.node_id)) {
        try {
          pi.pose = db.sensor_frame_pose(pano.node_id);
          pi.px = pi.pose[3]; // row-major: translation is [3], [7], [11]
          pi.py = pi.pose[7];
          pi.pz = pi.pose[11];
          pi.pose_valid = true;
          spdlog::debug("Panorama '{}' at ({:.2f}, {:.2f}, {:.2f})",
                        pi.filename, pi.px, pi.py, pi.pz);
        } catch (const std::exception &e) {
          spdlog::warn("Could not load pose for panorama '{}': {}", pi.filename,
                       e.what());
        }
      } else {
        spdlog::debug(
            "Panorama '{}' has no linked sensor frame, skipping sphere",
            pi.filename);
      }

      result.panoramas.push_back(std::move(pi));
    }
  } catch (const std::exception &e) {
    spdlog::debug("Panoramic images not available: {}", e.what());
  }

  return result;
}

} // namespace rux::view
