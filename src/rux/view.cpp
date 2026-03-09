// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "rux/view.hpp"
#include <spdlog/spdlog.h>

#include <pcl/PolygonMesh.h>
#include <pcl/common/common.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <fmt/format.h>
#include <fmt/std.h>

#include <filesystem>
#include <latch>
#include <mutex>
#include <thread>

void setup_subcommand_view(CLI::App &app) {

  auto opt = std::make_shared<SubcommandViewOptions>();
  auto *sub = app.add_subcommand(
      "view",
      "Visualize 3D point clouds, meshes, and optional label overlays.");

  sub->add_option("inputs", opt->input_paths,
                  "Path(s) to input cloud/mesh file(s).")
      ->required()
      ->check(CLI::ExistingFile);

  sub->add_option("-l, --label", opt->label_paths_in,
                  "Path(s) to the label file to overlay on point clouds.");

  sub->callback([opt]() {
    spdlog::trace("calling viewer subcommand");
    return run_subcommand_view(*opt);
  });
}

int run_subcommand_view(SubcommandViewOptions const &opt) {

  spdlog::trace("Visualization thread started");

  // ============================================================================
  // VIEWER SETUP
  // ============================================================================
  auto viewer =
      std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");

  // Create viewport
  std::vector<int> viewports(1);
  for (size_t i = 0; i < viewports.size(); ++i) {
    float left = static_cast<float>(i) / static_cast<float>(viewports.size());
    float right =
        static_cast<float>(i + 1) / static_cast<float>(viewports.size());
    viewer->createViewPort(left, 0.0, right, 1.0, viewports[i]);
    viewer->setBackgroundColor(0, 0, 0, viewports[i]);
    viewer->addCoordinateSystem(1.0, fmt::format("vp{}", i), viewports[i]);
  }

  // Initialize camera with proper orientation (fix upside-down issue)
  viewer->initCameraParameters();
  // viewer->setCameraPosition(0, 0, -10, // Camera position
  //                           0, 0, 0,   // Look at point
  //                           0, -1,
  //                           0); // Up vector (negative Y to fix orientation)

  // ============================================================================
  // DATA LOADING
  // ============================================================================

  // Storage for different data types
  struct LoadedCloud {
    CloudPtr cloud;
    std::string name;
    bool visible = true;
  };

  struct LoadedMesh {
    pcl::PolygonMesh::Ptr mesh;
    std::string name;
    bool visible = true;
  };

  std::vector<LoadedCloud> clouds;
  std::vector<LoadedMesh> meshes;
  std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr> label_clouds;

  // Visibility state
  struct VisibilityState {
    int current_label = -1; // -1 means no label, 0-8 means label index
  };
  auto vis_state = std::make_shared<VisibilityState>();

  // Load all input files
  for (size_t idx = 0; idx < opt.input_paths.size(); ++idx) {
    const auto &input_path = opt.input_paths[idx];
    std::string ext = input_path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    bool is_point_cloud = (ext == ".pcd");
    bool is_mesh =
        (ext == ".ply" || ext == ".obj" || ext == ".stl" || ext == ".vtk");

    if (is_point_cloud) {
      spdlog::trace("Loading point cloud from {}", input_path);
      LoadedCloud loaded;
      loaded.cloud = CloudPtr(new Cloud);
      loaded.name = fmt::format("cloud_{}", idx);
      pcl::io::load<PointT>(input_path.string(), *loaded.cloud);
      viewer->addPointCloud<PointT>(loaded.cloud, loaded.name, viewports[0]);
      clouds.push_back(std::move(loaded));
      spdlog::info("Loaded cloud: {} ({} points)",
                   input_path.filename().string(), clouds.back().cloud->size());

    } else if (is_mesh) {
      spdlog::trace("Loading mesh from {}", input_path);
      LoadedMesh loaded;
      loaded.mesh = pcl::PolygonMesh::Ptr(new pcl::PolygonMesh);
      loaded.name = fmt::format("mesh_{}", idx);

      if (ext == ".ply") {
        pcl::io::loadPLYFile(input_path.string(), *loaded.mesh);
      } else if (ext == ".obj") {
        pcl::io::loadOBJFile(input_path.string(), *loaded.mesh);
      } else if (ext == ".stl") {
        pcl::io::loadPolygonFileSTL(input_path.string(), *loaded.mesh);
      } else if (ext == ".vtk") {
        pcl::io::loadPolygonFileVTK(input_path.string(), *loaded.mesh);
      }

      viewer->addPolygonMesh(*loaded.mesh, loaded.name, viewports[0]);
      meshes.push_back(std::move(loaded));
      spdlog::info("Loaded mesh: {} ({} polygons)",
                   input_path.filename().string(),
                   meshes.back().mesh->polygons.size());

    } else {
      spdlog::error("Unsupported file extension: {}", ext);
      spdlog::error("Supported extensions: .pcd (point cloud), .ply, .obj, "
                    ".stl, .vtk (meshes)");
      continue;
    }
  }

  // Load label clouds if provided (map to first point cloud)
  if (!opt.label_paths_in.empty() && !clouds.empty()) {
    CloudPtr first_cloud = clouds[0].cloud;
    label_clouds.reserve(opt.label_paths_in.size());

    for (const auto &label_path : opt.label_paths_in) {
      spdlog::trace("Loading label cloud from {}", label_path);
      CloudLPtr labels(new CloudL);
      pcl::io::load<LabelT>(label_path.string(), *labels);

      if (labels->size() != first_cloud->size()) {
        spdlog::error(
            "Label cloud size ({}) does not match first point cloud size ({})",
            labels->size(), first_cloud->size());
        continue;
      }

      label_clouds.emplace_back(new pcl::PointCloud<pcl::PointXYZL>);
      label_clouds.back()->resize(first_cloud->size());
      for (size_t i = 0; i < first_cloud->size(); ++i) {
        label_clouds.back()->points[i].x = first_cloud->points[i].x;
        label_clouds.back()->points[i].y = first_cloud->points[i].y;
        label_clouds.back()->points[i].z = first_cloud->points[i].z;
        label_clouds.back()->points[i].label = labels->points[i].label;
      }
      spdlog::info("Loaded label: {}", label_path.filename().string());
    }
  } else if (!opt.label_paths_in.empty() && clouds.empty()) {
    spdlog::warn("Label files provided but no point clouds loaded. Labels will "
                 "be ignored.");
  }

  // ============================================================================
  // KEYBOARD CALLBACKS
  // ============================================================================

  // Toggle individual point clouds (keys 'c' then '0'-'9')
  if (!clouds.empty()) {
    viewer->registerKeyboardCallback(
        [&viewer, &clouds](const pcl::visualization::KeyboardEvent &event) {
          for (size_t i = 0; i < clouds.size() && i < 10; ++i) {
            if (event.getKeySym() == fmt::format("c_{}", i) &&
                event.keyDown()) {
              clouds[i].visible = !clouds[i].visible;
              if (clouds[i].visible) {
                viewer->addPointCloud<PointT>(clouds[i].cloud, clouds[i].name,
                                              0);
                spdlog::info("Cloud {} shown", i);
              } else {
                viewer->removePointCloud(clouds[i].name);
                spdlog::info("Cloud {} hidden", i);
              }
            }
          }
        });
  }

  // Toggle all point clouds (key 'c')
  if (!clouds.empty()) {
    viewer->registerKeyboardCallback(
        [&viewer, &clouds](const pcl::visualization::KeyboardEvent &event) {
          if (event.getKeySym() == "c" && event.keyDown()) {
            bool all_visible = true;
            for (const auto &cloud : clouds) {
              if (!cloud.visible) {
                all_visible = false;
                break;
              }
            }

            // Toggle all
            for (auto &cloud : clouds) {
              if (all_visible) {
                viewer->removePointCloud(cloud.name);
                cloud.visible = false;
              } else {
                viewer->addPointCloud<PointT>(cloud.cloud, cloud.name, 0);
                cloud.visible = true;
              }
            }
            spdlog::info("All clouds {}", all_visible ? "hidden" : "shown");
          }
        });
  }

  // Toggle individual meshes (key 'm' then number)
  if (!meshes.empty()) {
    viewer->registerKeyboardCallback(
        [&viewer, &meshes](const pcl::visualization::KeyboardEvent &event) {
          for (size_t i = 0; i < meshes.size() && i < 10; ++i) {
            if (event.getKeySym() == fmt::format("m_{}", i) &&
                event.keyDown()) {
              meshes[i].visible = !meshes[i].visible;
              if (meshes[i].visible) {
                viewer->addPolygonMesh(*meshes[i].mesh, meshes[i].name, 0);
                spdlog::info("Mesh {} shown", i);
              } else {
                viewer->removePolygonMesh(meshes[i].name);
                spdlog::info("Mesh {} hidden", i);
              }
            }
          }
        });
  }

  // Toggle all meshes (key 'm')
  if (!meshes.empty()) {
    viewer->registerKeyboardCallback(
        [&viewer, &meshes](const pcl::visualization::KeyboardEvent &event) {
          if (event.getKeySym() == "m" && event.keyDown()) {
            bool all_visible = true;
            for (const auto &mesh : meshes) {
              if (!mesh.visible) {
                all_visible = false;
                break;
              }
            }

            // Toggle all
            for (auto &mesh : meshes) {
              if (all_visible) {
                viewer->removePolygonMesh(mesh.name);
                mesh.visible = false;
              } else {
                viewer->addPolygonMesh(*mesh.mesh, mesh.name, 0);
                mesh.visible = true;
              }
            }
            spdlog::info("All meshes {}", all_visible ? "hidden" : "shown");
          }
        });
  }

  // Toggle label views (keys '1'-'9')
  for (size_t i = 0; i < label_clouds.size() && i < 9; ++i) {
    viewer->registerKeyboardCallback([i, &viewer, &label_clouds, &viewports,
                                      vis_state](
                                         const pcl::visualization::KeyboardEvent
                                             &event) {
      if (event.getKeySym() == std::to_string(i + 1) && event.keyDown()) {
        const std::string cloud_name = fmt::format("label_cloud_{}", i);

        // Toggle this label view
        if (static_cast<int>(i) == vis_state->current_label) {
          // Already showing this label, turn it off
          viewer->removePointCloud(cloud_name);
          vis_state->current_label = -1;
          spdlog::info("Label {} hidden", i + 1);
        } else {
          // Remove previous label if any
          if (vis_state->current_label >= 0) {
            viewer->removePointCloud(
                fmt::format("label_cloud_{}", vis_state->current_label));
          }

          // Add new label
          spdlog::info("Showing label {}", i + 1);
          pcl::visualization::PointCloudColorHandlerLabelField<pcl::PointXYZL>
              color_handler(label_clouds[i]);
          viewer->addPointCloud<pcl::PointXYZL>(label_clouds[i], color_handler,
                                                cloud_name, viewports[0]);
          viewer->setPointCloudRenderingProperties(
              pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud_name);
          vis_state->current_label = static_cast<int>(i);
        }
      }
    });
  }

  // Help message (key 'h')
  viewer->registerKeyboardCallback(
      [&clouds, &meshes,
       &label_clouds](const pcl::visualization::KeyboardEvent &event) {
        if (event.getKeySym() == "h" && event.keyDown()) {
          spdlog::info("=== Viewer Keyboard Controls ===");
          if (!clouds.empty()) {
            spdlog::info("  c: Toggle all point clouds");
          }
          if (!meshes.empty()) {
            spdlog::info("  m: Toggle all meshes");
          }
          if (!label_clouds.empty()) {
            spdlog::info("  1-9: Toggle label overlay");
          }
          spdlog::info("  h: Show this help");
          spdlog::info("  q: Quit viewer");
        }
      });

  // Print initial help
  spdlog::info("Press 'h' for keyboard controls");
  spdlog::info("Loaded: {} cloud(s), {} mesh(es), {} label(s)", clouds.size(),
               meshes.size(), label_clouds.size());

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return 0;
}
