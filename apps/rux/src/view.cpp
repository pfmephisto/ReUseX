// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
#include "view.hpp"
#include "processing_observer.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/geometry/BuildingComponent.hpp>

#include <fmt/format.h>
#include <fmt/std.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/common.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <spdlog/spdlog.h>

#include <filesystem>
#include <latch>
#include <mutex>
#include <optional>
#include <thread>

namespace {

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/// A point cloud loaded for visualization with associated metadata.
struct LoadedCloud {
  CloudPtr cloud;      ///< The point cloud data
  std::string name;    ///< Display name in viewer
  bool visible = true; ///< Current visibility state
};

/// A polygon mesh loaded for visualization with associated metadata.
struct LoadedMesh {
  std::variant<pcl::PolygonMesh::Ptr, pcl::TextureMesh::Ptr>
      mesh;            ///< The mesh data (regular or textured)
  std::string name;    ///< Display name in viewer
  bool visible = true; ///< Current visibility state
                       /// Check if this mesh has texture information.
  bool is_textured() const {
    return std::holds_alternative<pcl::TextureMesh::Ptr>(mesh);
  }
};

/// Tracks the currently visible label overlay in the viewer.
struct ViewerState {
  int current_label =
      -1; ///< Index of currently shown label (-1 = none, 0-8 = label index)
};

// ============================================================================
// FILE TYPE DETECTION
// ============================================================================

/// Enumeration of supported file types
enum class FileType {
  PointCloud, ///< .pcd files
  Mesh,       ///< .ply, .obj, .stl, .vtk files
  Project,    ///< .rux project database files
  Unsupported ///< Unrecognized extension
};

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

// ============================================================================
// PROJECT DATABASE LOADING
// ============================================================================

/// Result of loading geometry from a .rux project database.
struct ProjectLoadResult {
  std::vector<LoadedCloud> clouds;
  std::vector<LoadedMesh> meshes;
  std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr> labels;
  std::vector<ReUseX::geometry::BuildingComponent> components;
};

/// Load all viewable geometry from a ReUseX project database.
///
/// @param path Filesystem path to .rux file
/// @param observer Visualization observer for enqueueing viewer tasks
/// @return ProjectLoadResult containing loaded clouds, meshes, and labels
ProjectLoadResult load_from_project_db(const fs::path &path,
                                       rux::VizualizationObserver &observer) {
  spdlog::info("Opening project database: {}", path);
  ReUseX::ProjectDB db(path, /*readOnly=*/true);

  ProjectLoadResult result;
  CloudPtr first_xyzrgb;

  // Load point clouds
  auto cloud_names = db.list_point_clouds();
  for (const auto &name : cloud_names) {
    auto type = db.point_cloud_type(name);

    if (type == "PointXYZRGB") {
      try {
        auto cloud = db.point_cloud_xyzrgb(name);
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
        if (labels->size() != first_xyzrgb->size()) {
          spdlog::error(
              "Label cloud '{}' size ({}) does not match first point cloud "
              "size ({})",
              name, labels->size(), first_xyzrgb->size());
          continue;
        }
        result.labels.push_back(create_label_cloud(labels, first_xyzrgb));
        spdlog::info("Loaded label cloud '{}'", name);
      } catch (const std::exception &e) {
        spdlog::error("Failed to load label cloud '{}': {}", name, e.what());
      }
    }
  }

  // Load meshes
  auto mesh_names = db.list_meshes();
  for (const auto &name : mesh_names) {

    LoadedMesh loaded;
    loaded.name = fmt::format("mesh_{}", result.meshes.size());
    bool loaded_successfully = false;

    spdlog::debug("Attempting to load mesh '{}'", name);

    // Try textured mesh first (more specific format)
    try {
      auto texture_mesh = db.texture_mesh(name);

      // Keep as TextureMesh and render with textures
      loaded.mesh = texture_mesh;

      // observer.viewer_enqueue_task(
      //     [loaded](const rux::VizualizationObserver::ViewerPtr &viewer,
      //              const std::vector<int> &viewports) {
      //       auto textured = std::get<pcl::TextureMesh::Ptr>(loaded.mesh);
      //       viewer->addTextureMesh(*textured, loaded.name, viewports[0]);
      //     });

      spdlog::info(
          "Loaded textured mesh '{}' ({} materials) - rendering with textures",
          name, texture_mesh->tex_materials.size());
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

  return result;
}

// ============================================================================
// TRAITS FOR TEMPLATE-BASED CALLBACK REGISTRATION
// ============================================================================

/// Traits class template for viewer item operations.
/// Specializations provide type-specific operations for clouds and meshes.
template <typename T> struct ItemTraits;

/// Traits specialization for point clouds.
template <> struct ItemTraits<LoadedCloud> {
  static constexpr const char *type_name = "Cloud";
  static constexpr const char *key_prefix = "c";

  static void add_to_viewer(const rux::VizualizationObserver::ViewerPtr &viewer,
                            const LoadedCloud &item, int viewport) {
    viewer->addPointCloud<PointT>(item.cloud, item.name, viewport);
  }

  static void
  remove_from_viewer(const rux::VizualizationObserver::ViewerPtr &viewer,
                     const std::string &name) {
    viewer->removePointCloud(name);
  }
};

/// Traits specialization for polygon meshes.
template <> struct ItemTraits<LoadedMesh> {
  static constexpr const char *type_name = "Mesh";
  static constexpr const char *key_prefix = "m";

  static void add_to_viewer(const rux::VizualizationObserver::ViewerPtr &viewer,
                            const LoadedMesh &item, int viewport) {
    if (item.is_textured()) {
      // // Render textured mesh with materials and UV coordinates
      // auto textured = std::get<pcl::TextureMesh::Ptr>(item.mesh);
      // viewer->addTextureMesh(*textured, item.name, viewport);
    } else {
      // Render regular polygon mesh
      auto regular = std::get<pcl::PolygonMesh::Ptr>(item.mesh);
      viewer->addPolygonMesh(*regular, item.name, viewport);
    }
  }

  static void
  remove_from_viewer(const rux::VizualizationObserver::ViewerPtr &viewer,
                     const std::string &name) {
    viewer->removePolygonMesh(name);
  }
};

// ============================================================================
// KEYBOARD CALLBACK REGISTRATION (TEMPLATE-BASED)
// ============================================================================

/// Register keyboard callbacks for toggling individual items (e.g., c_0, m_0).
///
/// @tparam LoadedItem Type of item (LoadedCloud or LoadedMesh)
/// @param items Vector of loaded items to register toggles for
/// @param observer Visualization observer for enqueueing viewer tasks
template <typename LoadedItem>
void register_individual_toggles(std::vector<LoadedItem> &items,
                                 rux::VizualizationObserver &observer) {
  using Traits = ItemTraits<LoadedItem>;

  if (items.empty())
    return;

  observer.viewer_enqueue_task(
      [&items, &observer](const rux::VizualizationObserver::ViewerPtr &viewer,
                          const std::vector<int> &) {
        viewer->registerKeyboardCallback(
            [&items,
             &observer](const pcl::visualization::KeyboardEvent &event) {
              for (size_t i = 0; i < items.size() && i < 10; ++i) {
                if (event.getKeySym() ==
                        fmt::format("{}_{}", Traits::key_prefix, i) &&
                    event.keyDown()) {
                  items[i].visible = !items[i].visible;
                  if (items[i].visible) {
                    observer.viewer_enqueue_task(
                        [i, &items](
                            const rux::VizualizationObserver::ViewerPtr &viewer,
                            const std::vector<int> &viewports) {
                          Traits::add_to_viewer(viewer, items[i], viewports[0]);
                        });
                    spdlog::info("{} {} shown", Traits::type_name, i);
                  } else {
                    observer.viewer_enqueue_task(
                        [i, &items](
                            const rux::VizualizationObserver::ViewerPtr &viewer,
                            const std::vector<int> &) {
                          Traits::remove_from_viewer(viewer, items[i].name);
                        });
                    spdlog::info("{} {} hidden", Traits::type_name, i);
                  }
                }
              }
            });
      });
}

/// Register keyboard callback for toggling all items at once (key 'c' or 'm').
///
/// @tparam LoadedItem Type of item (LoadedCloud or LoadedMesh)
/// @param items Vector of loaded items to register toggle for
/// @param observer Visualization observer for enqueueing viewer tasks
template <typename LoadedItem>
void register_toggle_all_callback(std::vector<LoadedItem> &items,
                                  rux::VizualizationObserver &observer) {
  using Traits = ItemTraits<LoadedItem>;

  if (items.empty())
    return;

  observer.viewer_enqueue_task(
      [&items, &observer](const rux::VizualizationObserver::ViewerPtr &viewer,
                          const std::vector<int> &) {
        viewer->registerKeyboardCallback(
            [&items,
             &observer](const pcl::visualization::KeyboardEvent &event) {
              if (event.getKeySym() == Traits::key_prefix && event.keyDown()) {
                // Check if all are currently visible
                bool all_visible = true;
                for (const auto &item : items) {
                  if (!item.visible) {
                    all_visible = false;
                    break;
                  }
                }

                //  Toggle all items
                for (size_t i = 0; i < items.size(); ++i) {
                  if (all_visible) {
                    observer.viewer_enqueue_task(
                        [i, &items](
                            const rux::VizualizationObserver::ViewerPtr &viewer,
                            const std::vector<int> &) {
                          Traits::remove_from_viewer(viewer, items[i].name);
                          items[i].visible = false;
                        });
                  } else {
                    observer.viewer_enqueue_task(
                        [i, &items](
                            const rux::VizualizationObserver::ViewerPtr &viewer,
                            const std::vector<int> &viewports) {
                          Traits::add_to_viewer(viewer, items[i], viewports[0]);
                          items[i].visible = true;
                        });
                  }
                }
                spdlog::info("All {}s {}", Traits::type_name,
                             all_visible ? "hidden" : "shown");
              }
            });
      });
}

/// Register keyboard callbacks for toggling label overlays (keys '1'-'9').
///
/// @param label_clouds Vector of label clouds to register toggles for
/// @param state Shared viewer state tracking currently visible label
/// @param observer Visualization observer for enqueueing viewer tasks
void register_label_toggles(
    const std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr> &label_clouds,
    std::shared_ptr<ViewerState> state, rux::VizualizationObserver &observer) {
  for (size_t i = 0; i < label_clouds.size() && i < 9; ++i) {
    observer.viewer_enqueue_task([i, &observer, &label_clouds, state](
                                     const rux::VizualizationObserver::ViewerPtr
                                         &viewer,
                                     const std::vector<int> &) {
      viewer->registerKeyboardCallback(
          [i, &observer, &label_clouds,
           state](const pcl::visualization::KeyboardEvent &event) {
            if (event.getKeySym() == std::to_string(i + 1) && event.keyDown()) {
              const std::string cloud_name = fmt::format("label_cloud_{}", i);

              // Toggle this label view
              if (static_cast<int>(i) == state->current_label) {
                // Already showing this label, turn it off
                observer.viewer_enqueue_task(
                    [cloud_name,
                     state](const rux::VizualizationObserver::ViewerPtr &viewer,
                            const std::vector<int> &) {
                      viewer->removePointCloud(cloud_name);
                      state->current_label = -1;
                    });
                spdlog::info("Label {} hidden", i + 1);
              } else {
                // Remove previous label if any
                if (state->current_label >= 0) {
                  observer.viewer_enqueue_task(
                      [state](
                          const rux::VizualizationObserver::ViewerPtr &viewer,
                          const std::vector<int> &) {
                        viewer->removePointCloud(fmt::format(
                            "label_cloud_{}", state->current_label));
                      });
                }

                // Add new label
                observer.viewer_enqueue_task(
                    [i, &label_clouds, cloud_name,
                     state](const rux::VizualizationObserver::ViewerPtr &viewer,
                            const std::vector<int> &viewports) {
                      pcl::visualization::PointCloudColorHandlerLabelField<
                          pcl::PointXYZL>
                          color_handler(label_clouds[i]);
                      viewer->addPointCloud<pcl::PointXYZL>(
                          label_clouds[i], color_handler, cloud_name,
                          viewports[0]);
                      viewer->setPointCloudRenderingProperties(
                          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,
                          cloud_name);
                      state->current_label = static_cast<int>(i);
                    });
                spdlog::info("Showing label {}", i + 1);
              }
            }
          });
    });
  }
}

// ============================================================================
// BUILDING COMPONENT RENDERING
// ============================================================================

/// Generate all line shape names for a set of building components.
///
/// @param components Vector of building components
/// @return Vector of shape names (one per line segment across all components)
std::vector<std::string> component_line_names(
    const std::vector<ReUseX::geometry::BuildingComponent> &components) {
  std::vector<std::string> names;
  for (size_t ci = 0; ci < components.size(); ++ci) {
    const auto &verts = components[ci].boundary.vertices;
    if (verts.size() < 2)
      continue;
    for (size_t i = 0; i < verts.size(); ++i) {
      names.push_back(fmt::format("comp_{}_{}", ci, i));
    }
  }
  return names;
}

/// Enqueue line-drawing tasks to render building component boundaries.
///
/// Each component's CoplanarPolygon is drawn as a closed loop of lines.
/// Windows are green, doors are cyan, walls are yellow.
///
/// @param components Vector of building components to render
/// @param observer Visualization observer for enqueueing viewer tasks
void add_component_lines(
    const std::vector<ReUseX::geometry::BuildingComponent> &components,
    rux::VizualizationObserver &observer) {
  for (size_t ci = 0; ci < components.size(); ++ci) {
    const auto &comp = components[ci];
    const auto &verts = comp.boundary.vertices;
    if (verts.size() < 2)
      continue;

    // Pick color based on component type
    double r = 0.0, g = 1.0, b = 0.0; // green for windows
    if (comp.type == ReUseX::geometry::ComponentType::door) {
      r = 0.0; g = 1.0; b = 1.0; // cyan for doors
    } else if (comp.type == ReUseX::geometry::ComponentType::wall) {
      r = 1.0; g = 1.0; b = 0.0; // yellow for walls
    }

    for (size_t i = 0; i < verts.size(); ++i) {
      size_t j = (i + 1) % verts.size();
      auto line_name = fmt::format("comp_{}_{}", ci, i);
      auto p1 = verts[i], p2 = verts[j];
      observer.viewer_enqueue_task(
          [p1, p2, line_name, r, g, b](
              const rux::VizualizationObserver::ViewerPtr &viewer,
              const std::vector<int> &viewports) {
            pcl::PointXYZ a(static_cast<float>(p1.x()),
                            static_cast<float>(p1.y()),
                            static_cast<float>(p1.z()));
            pcl::PointXYZ b_pt(static_cast<float>(p2.x()),
                               static_cast<float>(p2.y()),
                               static_cast<float>(p2.z()));
            viewer->addLine(a, b_pt, r, g, b, line_name, viewports[0]);
            viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3.0, line_name);
          });
    }
  }
}

/// Register keyboard callback to toggle building component visibility (key 'w').
///
/// @param components Vector of building components
/// @param components_visible Shared flag tracking visibility state
/// @param observer Visualization observer for enqueueing viewer tasks
void register_component_toggle(
    const std::vector<ReUseX::geometry::BuildingComponent> &components,
    std::shared_ptr<bool> components_visible,
    rux::VizualizationObserver &observer) {
  auto names = component_line_names(components);

  observer.viewer_enqueue_task(
      [&components, names, components_visible,
       &observer](const rux::VizualizationObserver::ViewerPtr &viewer,
                  const std::vector<int> &) {
        viewer->registerKeyboardCallback(
            [&components, names, components_visible,
             &observer](const pcl::visualization::KeyboardEvent &event) {
              if (event.getKeySym() == "w" && event.keyDown()) {
                if (*components_visible) {
                  // Remove all component lines
                  for (const auto &name : names) {
                    observer.viewer_enqueue_task(
                        [name](
                            const rux::VizualizationObserver::ViewerPtr &viewer,
                            const std::vector<int> &) {
                          viewer->removeShape(name);
                        });
                  }
                  *components_visible = false;
                  spdlog::info("Components hidden");
                } else {
                  // Re-add all component lines
                  add_component_lines(components, observer);
                  *components_visible = true;
                  spdlog::info("Components shown");
                }
              }
            });
      });
}

/// Register help keyboard callback (key 'h').
///
/// @param clouds Vector of loaded clouds (for help message)
/// @param meshes Vector of loaded meshes (for help message)
/// @param label_clouds Vector of label clouds (for help message)
/// @param observer Visualization observer for enqueueing viewer tasks
void register_help_callback(
    const std::vector<LoadedCloud> &clouds,
    const std::vector<LoadedMesh> &meshes,
    const std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr> &label_clouds,
    bool has_components, rux::VizualizationObserver &observer) {
  observer.viewer_enqueue_task(
      [&clouds, &meshes, &label_clouds,
       has_components](const rux::VizualizationObserver::ViewerPtr &viewer,
                       const std::vector<int> &) {
        viewer->registerKeyboardCallback(
            [&clouds, &meshes, &label_clouds,
             has_components](const pcl::visualization::KeyboardEvent &event) {
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
                if (has_components) {
                  spdlog::info("  w: Toggle building components");
                }
                spdlog::info("  h: Show this help");
                spdlog::info("  q: Quit viewer");
              }
            });
      });
}

} // anonymous namespace

// ============================================================================
// SUBCOMMAND SETUP
// ============================================================================

void setup_subcommand_view(CLI::App &app,
                           std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandViewOptions>();
  auto *sub = app.add_subcommand("view", "Visualize point clouds and meshes");

  sub->footer(R"(
DESCRIPTION:
  Interactive 3D visualization of point clouds, meshes, and segmentation
  labels stored in ProjectDB. Loads all available geometry and provides
  keyboard controls for toggling visibility, switching label overlays,
  and exploring multi-modal scan data.

EXAMPLES:
  rux view                             # View project at ./project.rux
  rux -p scan.rux view                 # View custom project
  rux -vv view                         # Debug mode with verbose output

KEYBOARD CONTROLS:
  c         Toggle all point clouds on/off
  m         Toggle all meshes on/off
  w         Toggle building components (windows, doors, etc.)
  1-9       Toggle label overlay (planes, rooms, etc.)
  h         Show help message
  q         Quit viewer

WORKFLOW:
  1. rux import rtabmap scan.db        # Import sensor data
  2. rux create clouds                 # Reconstruct geometry
  3. rux create planes                 # Add segmentation
  4. rux view                          # Visualize results

NOTES:
  - Loads all point clouds, meshes, and labels from ProjectDB automatically
  - Label clouds mapped to first XYZRGB cloud for geometry
  - Supports multiple viewports for simultaneous views
  - PCL viewer based on VTK - standard camera controls apply
  - Use global -p/--project flag to specify database path
)");

  // View command loads data exclusively from the project database
  // Use global --project flag to specify database path

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling viewer subcommand");
    return run_subcommand_view(*opt, *global_opt);
  });
}

// ============================================================================
// MAIN SUBCOMMAND FUNCTION
// ============================================================================

int run_subcommand_view([[maybe_unused]] SubcommandViewOptions const &opt,
                        [[maybe_unused]] const RuxOptions &global_opt) {
  spdlog::trace("Visualization thread started");

  // Start the viewer thread
  rux::start_viewer();
  rux::VizualizationObserver &observer = rux::get_processing_observer();

  // === Phase 1: Load Data ===
  std::vector<LoadedCloud> clouds;
  std::vector<LoadedMesh> meshes;
  std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr> labels;

  // Load data from project database
  fs::path project_path = global_opt.project_db;
  spdlog::debug("Loading visualization data from project database: {}",
                project_path.string());

  auto result = load_from_project_db(project_path, observer);
  clouds = std::move(result.clouds);
  meshes = std::move(result.meshes);
  labels = std::move(result.labels);
  auto components = std::move(result.components);

  // === Phase 2: Register Keyboard Callbacks ===
  register_individual_toggles(clouds, observer);
  register_toggle_all_callback(clouds, observer);

  register_individual_toggles(meshes, observer);
  register_toggle_all_callback(meshes, observer);

  if (!labels.empty()) {
    auto state = std::make_shared<ViewerState>();
    register_label_toggles(labels, state, observer);
  }

  // Render building components as line loops and register 'w' toggle
  if (!components.empty()) {
    add_component_lines(components, observer);
    auto components_visible = std::make_shared<bool>(true);
    register_component_toggle(components, components_visible, observer);
  }

  register_help_callback(clouds, meshes, labels, !components.empty(), observer);

  // === Phase 3: Display Summary ===
  spdlog::info("Press 'h' for keyboard controls");
  spdlog::info("Loaded: {} cloud(s), {} mesh(es), {} label(s), {} component(s)",
               clouds.size(), meshes.size(), labels.size(), components.size());

  // We need to wait here otherwise the clouds go out of scope and get destroyed
  // and the viewer segfaults when it tries to access them.
  observer.viewer_wait_for_user();

  return 0;
}
