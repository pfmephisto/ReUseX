// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
#include "view.hpp"
#include "filter_utils.hpp"
#include "processing_observer.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/geometry/BuildingComponent.hpp>

#include <fmt/format.h>
#include <fmt/std.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/colors.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <spdlog/spdlog.h>

#include <opencv2/imgproc.hpp>

#include <vtkCamera.h>
#include <vtkImageData.h>
#include <vtkImageImport.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkPropCollection.h>
#include <vtkSkybox.h>
#include <vtkSmartPointer.h>
#include <vtkTexture.h>

#include <filesystem>
#include <latch>
#include <mutex>
#include <optional>
#include <set>
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
  bool legend_visible = false;           ///< Whether the label legend is shown
  std::vector<std::string> legend_ids;   ///< Shape IDs for legend text elements
};

/// A single entry in the label legend overlay.
struct LabelLegendEntry {
  int label_id;
  std::string name;
  pcl::RGB color;
};

/// A label cloud with its associated metadata for legend display.
struct LabelCloudInfo {
  pcl::PointCloud<pcl::PointXYZL>::Ptr cloud;
  std::string db_name;                          ///< Original cloud name in DB
  std::vector<LabelLegendEntry> legend_entries;  ///< Pre-computed at load time
};

/// Metadata for a panoramic image (pixel data loaded on-demand).
struct PanoramaInfo {
  int id;                        ///< Database row ID
  std::string filename;          ///< Original filename
  int node_id;                   ///< Linked sensor frame (-1 if unmatched)
  bool pose_valid = false;       ///< Whether a 3D pose is available
  double px, py, pz;             ///< Position from sensor frame pose
  std::array<double, 16> pose;   ///< Full 4x4 SE(3) row-major world pose
};

/// State for immersive panorama viewing.
struct PanoramaState {
  bool immersive = false;                   ///< Currently in panorama mode
  int active_index = -1;                    ///< Which panorama is displayed
  std::vector<PanoramaInfo> infos;          ///< All panorama metadata
  vtkSmartPointer<vtkSkybox> skybox;        ///< Reusable skybox actor

  /// Saved viewer state for restoration on exit.
  struct SavedView {
    double cam_pos[3], cam_focal[3], cam_up[3];
    std::vector<vtkSmartPointer<vtkProp>> hidden_props; ///< All props hidden on enter
  } saved;
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
// PANORAMA HELPERS
// ============================================================================

/// Get the VTK renderer for a PCL viewport index.
///
/// PCL creates a default renderer (index 0) in the PCLVisualizer constructor,
/// then createViewPort() adds additional renderers. Geometry is added to the
/// viewport renderer, NOT the default one. GetFirstRenderer() returns the
/// wrong renderer — use this helper instead.
vtkRenderer *
viewport_renderer(const rux::VizualizationObserver::ViewerPtr &viewer,
                  int viewport) {
  auto renderers = viewer->getRendererCollection();
  renderers->InitTraversal();
  vtkRenderer *renderer = nullptr;
  for (int i = 0; i <= viewport; ++i)
    renderer = renderers->GetNextItem();
  return renderer;
}

/// Convert a BGR cv::Mat to vtkImageData (RGB, bottom-up for VTK).
vtkSmartPointer<vtkImageData> cv_mat_to_vtk_image(const cv::Mat &bgr) {
  cv::Mat rgb;
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  cv::flip(rgb, rgb, 0); // VTK expects bottom-up

  auto importer = vtkSmartPointer<vtkImageImport>::New();
  importer->SetDataSpacing(1, 1, 1);
  importer->SetDataOrigin(0, 0, 0);
  importer->SetWholeExtent(0, rgb.cols - 1, 0, rgb.rows - 1, 0, 0);
  importer->SetDataExtentToWholeExtent();
  importer->SetDataScalarTypeToUnsignedChar();
  importer->SetNumberOfScalarComponents(3);
  importer->SetImportVoidPointer(rgb.data, /*save=*/1);
  importer->Update();

  auto image = vtkSmartPointer<vtkImageData>::New();
  image->DeepCopy(importer->GetOutput());
  return image;
}

/// Activate the panorama skybox in the VTK renderer.
///
/// Loads the image from the database, converts to cubemap, and adds the
/// skybox actor to the renderer.
///
/// @param db_path Path to the project database
/// @param info Panorama metadata (used for id)
/// @param skybox Reusable skybox actor
/// @param viewer PCL visualizer (used to access VTK renderer)
void activate_panorama_skybox(
    const fs::path &db_path, const PanoramaInfo &info,
    vtkSmartPointer<vtkSkybox> &skybox,
    const rux::VizualizationObserver::ViewerPtr &viewer, int viewport) {
  reusex::ProjectDB db(db_path, /*readOnly=*/true);
  cv::Mat pano = db.panoramic_image(info.id);

  if (pano.empty()) {
    spdlog::error("Panorama image {} is empty", info.id);
    return;
  }
  spdlog::debug("Panorama image: {}x{} type={}", pano.cols, pano.rows,
                pano.type());

  auto vtk_img = cv_mat_to_vtk_image(pano);

  // Use Sphere projection — the skybox fragment shader samples the 2D
  // equirectangular texture directly via spherical-coordinate mapping,
  // avoiding the GPU-based cubemap conversion which requires a fully
  // connected VTK pipeline (SetInputConnection) to work reliably.
  auto texture = vtkSmartPointer<vtkTexture>::New();
  texture->SetInputData(vtk_img);
  texture->InterpolateOn();
  texture->MipmapOn();

  if (!skybox)
    skybox = vtkSmartPointer<vtkSkybox>::New();
  skybox->SetTexture(texture);
  skybox->SetProjectionToSphere();
  skybox->GammaCorrectOff(); // JPEG is sRGB, not linear

  auto *renderer = viewport_renderer(viewer, viewport);
  renderer->AddActor(skybox);

  // Skybox returns zero bounds, so VTK's auto clipping range calculation
  // produces a degenerate range that clips the skybox geometry away.
  renderer->GetActiveCamera()->SetClippingRange(0.01, 100.0);
}

/// Remove the skybox actor from the VTK renderer.
void deactivate_panorama_skybox(
    const vtkSmartPointer<vtkSkybox> &skybox,
    const rux::VizualizationObserver::ViewerPtr &viewer, int viewport) {
  if (!skybox)
    return;
  auto *renderer = viewport_renderer(viewer, viewport);
  renderer->RemoveActor(skybox);
}

// ============================================================================
// PANORAMA MODE ENTER / EXIT
// ============================================================================

/// Enter immersive panorama mode: hide all scene geometry, show skybox.
///
/// Uses VTK prop visibility to hide everything in the viewport renderer
/// (clouds, meshes, components, coordinate axes, spheres, text — all of it).
/// The skybox is added after hiding, so it remains visible.
void enter_panorama_mode(
    std::shared_ptr<PanoramaState> state, int pano_index,
    const fs::path &project_path,
    const rux::VizualizationObserver::ViewerPtr &viewer,
    const std::vector<int> &viewports) {

  const auto &info = state->infos[static_cast<size_t>(pano_index)];

  // Save camera state — use the viewport renderer, not the default one
  auto *renderer = viewport_renderer(viewer, viewports[0]);
  auto *cam = renderer->GetActiveCamera();
  cam->GetPosition(state->saved.cam_pos);
  cam->GetFocalPoint(state->saved.cam_focal);
  cam->GetViewUp(state->saved.cam_up);

  // Hide all currently visible props in the renderer
  state->saved.hidden_props.clear();
  auto *props = renderer->GetViewProps();
  props->InitTraversal();
  while (auto *prop = props->GetNextProp()) {
    if (prop->GetVisibility()) {
      state->saved.hidden_props.emplace_back(prop);
      prop->VisibilityOff();
    }
  }

  // Activate skybox (added after hiding, so it stays visible)
  activate_panorama_skybox(project_path, info, state->skybox, viewer,
                           viewports[0]);

  // Position camera at panorama origin
  if (info.pose_valid) {
    cam->SetPosition(info.px, info.py, info.pz);
    // Row-major 4x4: column 2 is the Z-axis (forward in camera frame)
    double fx = info.pose[2];
    double fy = info.pose[6];
    double fz = info.pose[10];
    cam->SetFocalPoint(info.px + fx, info.py + fy, info.pz + fz);
    // Up is negative Y-axis of the pose (camera convention)
    cam->SetViewUp(-info.pose[1], -info.pose[5], -info.pose[9]);
    // Keep clipping range valid for skybox rendering
    cam->SetClippingRange(0.01, 100.0);
  }

  state->immersive = true;
  state->active_index = pano_index;
  spdlog::info("Entered panorama mode: {} [{}]", info.filename, pano_index + 1);
}

/// Exit immersive panorama mode: restore scene and camera.
void exit_panorama_mode(
    std::shared_ptr<PanoramaState> state,
    const rux::VizualizationObserver::ViewerPtr &viewer,
    const std::vector<int> &viewports) {

  // Remove skybox
  deactivate_panorama_skybox(state->skybox, viewer, viewports[0]);

  // Restore visibility of all previously visible props
  for (auto &prop : state->saved.hidden_props)
    prop->VisibilityOn();
  state->saved.hidden_props.clear();

  // Restore camera
  auto *renderer = viewport_renderer(viewer, viewports[0]);
  auto *cam = renderer->GetActiveCamera();
  cam->SetPosition(state->saved.cam_pos);
  cam->SetFocalPoint(state->saved.cam_focal);
  cam->SetViewUp(state->saved.cam_up);

  state->immersive = false;
  state->active_index = -1;
  spdlog::info("Exited panorama mode");
}

// ============================================================================
// PROJECT DATABASE LOADING
// ============================================================================

/// Result of loading geometry from a .rux project database.
struct ProjectLoadResult {
  std::vector<LoadedCloud> clouds;
  std::vector<LoadedMesh> meshes;
  std::vector<LabelCloudInfo> labels;
  std::vector<reusex::geometry::BuildingComponent> components;
  std::vector<PanoramaInfo> panoramas;
};

/// Load all viewable geometry from a ReUseX project database.
///
/// @param path Filesystem path to .rux file
/// @param observer Visualization observer for enqueueing viewer tasks
/// @param filter_expr Optional filter expression to limit visualization
/// @return ProjectLoadResult containing loaded clouds, meshes, and labels
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

      filter_indices = rux::filters::evaluate_filter(filter_expr, db, expected_size);

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

          spdlog::debug("Filtered cloud '{}': {} -> {} points",
                        name, cloud->size(), filtered_cloud->size());
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
          std::string lbl_name = it != defs.end()
                                     ? it->second
                                     : fmt::format("Label {}", lbl);
          pcl::RGB color =
              pcl::GlasbeyLUT::at(lbl % pcl::GlasbeyLUT::size());
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
    spdlog::warn("Filter is active but meshes cannot be filtered - showing full meshes");
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
        spdlog::info(
            "Loaded textured mesh '{}' ({} materials) - rendering with textures",
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

      if (pano.node_id >= 0 && db.has_sensor_frame(pano.node_id)) {
        try {
          pi.pose = db.sensor_frame_pose(pano.node_id);
          pi.px = pi.pose[3];  // row-major: translation is [3], [7], [11]
          pi.py = pi.pose[7];
          pi.pz = pi.pose[11];
          pi.pose_valid = true;
          spdlog::debug("Panorama '{}' at ({:.2f}, {:.2f}, {:.2f})",
                         pi.filename, pi.px, pi.py, pi.pz);
        } catch (const std::exception &e) {
          spdlog::warn("Could not load pose for panorama '{}': {}",
                        pi.filename, e.what());
        }
      } else {
        spdlog::debug("Panorama '{}' has no linked sensor frame, skipping sphere",
                       pi.filename);
      }

      result.panoramas.push_back(std::move(pi));
    }
  } catch (const std::exception &e) {
    spdlog::debug("Panoramic images not available: {}", e.what());
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
      viewer->removePolygonMesh(item.name);
      auto textured = std::get<pcl::TextureMesh::Ptr>(item.mesh);
      viewer->addTextureMesh(*textured, item.name, viewport);
    } else {
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

/// Remove the label legend overlay from the viewer.
void remove_label_legend(std::shared_ptr<ViewerState> state,
                         rux::VizualizationObserver &observer) {
  if (state->legend_ids.empty())
    return;
  auto ids = state->legend_ids;
  state->legend_ids.clear();
  observer.viewer_enqueue_task(
      [ids](const rux::VizualizationObserver::ViewerPtr &viewer,
            const std::vector<int> &) {
        for (const auto &id : ids)
          viewer->removeShape(id);
      });
}

/// Add a label legend overlay to the viewer for the given label cloud info.
void add_label_legend(const LabelCloudInfo &info,
                      std::shared_ptr<ViewerState> state,
                      rux::VizualizationObserver &observer) {
  remove_label_legend(state, observer);

  std::vector<std::string> ids;
  constexpr int max_entries = 25;
  int count =
      std::min(static_cast<int>(info.legend_entries.size()), max_entries);

  // Title row
  std::string title_id = "legend_title";
  ids.push_back(title_id);

  // Legend entries
  for (int i = 0; i < count; ++i) {
    ids.push_back(fmt::format("legend_swatch_{}", i));
    ids.push_back(fmt::format("legend_text_{}", i));
  }

  // Overflow trailer
  int overflow =
      static_cast<int>(info.legend_entries.size()) - max_entries;
  std::string trailer_id;
  if (overflow > 0) {
    trailer_id = "legend_overflow";
    ids.push_back(trailer_id);
  }

  state->legend_ids = ids;

  auto legend = info.legend_entries;
  auto db_name = info.db_name;
  observer.viewer_enqueue_task(
      [legend, db_name, count, overflow, trailer_id](
          const rux::VizualizationObserver::ViewerPtr &viewer,
          const std::vector<int> &) {
        // Title
        viewer->addText(db_name, 10, 30, 16, 0.1, 0.1, 0.1, "legend_title");

        // Entries
        for (int i = 0; i < count; ++i) {
          const auto &e = legend[static_cast<size_t>(i)];
          int y = 55 + i * 22;
          viewer->addText(
              "##", 10, y, 16,
              static_cast<double>(e.color.r) / 255.0,
              static_cast<double>(e.color.g) / 255.0,
              static_cast<double>(e.color.b) / 255.0,
              fmt::format("legend_swatch_{}", i));
          viewer->addText(
              fmt::format("{}: {}", e.label_id, e.name), 40, y, 14,
              0.1, 0.1, 0.1,
              fmt::format("legend_text_{}", i));
        }

        // Overflow
        if (overflow > 0) {
          int y = 55 + count * 22;
          viewer->addText(
              fmt::format("... and {} more", overflow), 10, y, 12,
              0.3, 0.3, 0.3, trailer_id);
        }
      });
}

/// Register keyboard callback for toggling label legend (key 'l').
void register_legend_toggle(
    const std::vector<LabelCloudInfo> &label_clouds,
    std::shared_ptr<ViewerState> state, rux::VizualizationObserver &observer) {
  observer.viewer_enqueue_task(
      [&label_clouds, state,
       &observer](const rux::VizualizationObserver::ViewerPtr &viewer,
                  const std::vector<int> &) {
        viewer->registerKeyboardCallback(
            [&label_clouds, state,
             &observer](const pcl::visualization::KeyboardEvent &event) {
              if (event.getKeySym() != "l" || !event.keyDown())
                return;
              if (state->legend_visible) {
                remove_label_legend(state, observer);
                state->legend_visible = false;
                spdlog::info("Label legend hidden");
              } else if (state->current_label >= 0 &&
                         state->current_label <
                             static_cast<int>(label_clouds.size())) {
                add_label_legend(
                    label_clouds[static_cast<size_t>(state->current_label)],
                    state, observer);
                state->legend_visible = true;
                spdlog::info("Label legend shown");
              } else {
                spdlog::info("No label overlay active — press 1-9 first");
              }
            });
      });
}

/// Register keyboard callbacks for toggling label overlays (keys '1'-'9').
///
/// @param label_clouds Vector of label cloud info to register toggles for
/// @param state Shared viewer state tracking currently visible label
/// @param observer Visualization observer for enqueueing viewer tasks
void register_label_toggles(
    const std::vector<LabelCloudInfo> &label_clouds,
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
                // Remove legend if visible
                if (state->legend_visible) {
                  remove_label_legend(state, observer);
                  state->legend_visible = false;
                }
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
                          color_handler(label_clouds[i].cloud);
                      viewer->addPointCloud<pcl::PointXYZL>(
                          label_clouds[i].cloud, color_handler, cloud_name,
                          viewports[0]);
                      viewer->setPointCloudRenderingProperties(
                          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,
                          cloud_name);
                      state->current_label = static_cast<int>(i);
                    });
                // Update legend if visible
                if (state->legend_visible)
                  add_label_legend(label_clouds[i], state, observer);
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
    const std::vector<reusex::geometry::BuildingComponent> &components) {
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
    const std::vector<reusex::geometry::BuildingComponent> &components,
    rux::VizualizationObserver &observer) {
  for (size_t ci = 0; ci < components.size(); ++ci) {
    const auto &comp = components[ci];
    const auto &verts = comp.boundary.vertices;
    if (verts.size() < 2)
      continue;

    // Pick color based on component type
    double r = 0.0, g = 1.0, b = 0.0; // green for windows
    if (comp.type == reusex::geometry::ComponentType::door) {
      r = 0.0; g = 1.0; b = 1.0; // cyan for doors
    } else if (comp.type == reusex::geometry::ComponentType::wall) {
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
    const std::vector<reusex::geometry::BuildingComponent> &components,
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
    const std::vector<LabelCloudInfo> &label_clouds,
    bool has_components, bool has_panoramas,
    rux::VizualizationObserver &observer) {
  observer.viewer_enqueue_task(
      [&clouds, &meshes, &label_clouds, has_components,
       has_panoramas](const rux::VizualizationObserver::ViewerPtr &viewer,
                      const std::vector<int> &) {
        viewer->registerKeyboardCallback(
            [&clouds, &meshes, &label_clouds, has_components,
             has_panoramas](const pcl::visualization::KeyboardEvent &event) {
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
                  spdlog::info("  l: Toggle label color legend");
                }
                if (has_components) {
                  spdlog::info("  w: Toggle building components");
                }
                if (has_panoramas) {
                  spdlog::info("  Shift+Click: Enter 360 panorama (on orange sphere)");
                  spdlog::info("  Escape: Exit panorama mode");
                  spdlog::info("  [/]: Previous/Next panorama (in panorama mode)");
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

  sub->add_option(
         "-f, --filter", opt->filter_expr,
         "Filter expression to limit visualization to specific labeled points.\n"
         "Syntax: <cloud_name> <op> <value(s)>\n"
         "Examples:\n"
         "  -f 'planes in [1, 2, 5]'        # Show only labels 1, 2, 5 from planes cloud\n"
         "  -f 'rooms == 3'                 # Show only room 3\n"
         "  -f 'planes in [1,2] || rooms == 5'  # Combine multiple clouds\n"
         "  -f 'planes >= 10 && planes <= 20'   # Range filter")
      ->default_val("");

  sub->footer(R"(
DESCRIPTION:
  Interactive 3D visualization of point clouds, meshes, and segmentation
  labels stored in ProjectDB. Loads all available geometry and provides
  keyboard controls for toggling visibility, switching label overlays,
  and exploring multi-modal scan data.

EXAMPLES:
  rux view                             # View project at ./project.rux
  rux -p scan.rux view                 # View custom project
  rux view -f 'planes == 3'            # View only plane 3
  rux view -f 'rooms in [1,2]'         # View only rooms 1 and 2
  rux -vv view                         # Debug mode with verbose output

KEYBOARD CONTROLS:
  c           Toggle all point clouds on/off
  m           Toggle all meshes on/off
  w           Toggle building components (windows, doors, etc.)
  1-9         Toggle label overlay (planes, rooms, etc.)
  l           Toggle label color legend
  Shift+Click Enter 360 panorama (click on orange sphere)
  Escape      Exit panorama mode
  [/]         Previous/Next panorama (in panorama mode)
  h           Show help message
  q           Quit viewer

WORKFLOW:
  1. rux import rtabmap scan.db        # Import sensor data
  2. rux create clouds                 # Reconstruct geometry
  3. rux create planes                 # Add segmentation
  4. rux view                          # Visualize results

NOTES:
  - Loads all point clouds, meshes, and labels from ProjectDB automatically
  - Filter applies uniformly to all XYZRGB clouds with corresponding label filtering
  - Meshes and building components are not filtered (shown in full)
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
  std::vector<LabelCloudInfo> labels;

  // Load data from project database
  fs::path project_path = global_opt.project_db;
  spdlog::debug("Loading visualization data from project database: {}",
                project_path.string());

  auto result = load_from_project_db(project_path, observer, opt.filter_expr);
  clouds = std::move(result.clouds);
  meshes = std::move(result.meshes);
  labels = std::move(result.labels);
  auto components = std::move(result.components);

  // Setup panorama state
  auto pano_state = std::make_shared<PanoramaState>();
  pano_state->infos = std::move(result.panoramas);

  // === Phase 2: Register Keyboard Callbacks ===
  register_individual_toggles(clouds, observer);
  register_toggle_all_callback(clouds, observer);

  register_individual_toggles(meshes, observer);
  register_toggle_all_callback(meshes, observer);

  if (!labels.empty()) {
    auto state = std::make_shared<ViewerState>();
    register_label_toggles(labels, state, observer);
    register_legend_toggle(labels, state, observer);
  }

  // Render building components as line loops and register 'w' toggle
  if (!components.empty()) {
    add_component_lines(components, observer);
    auto components_visible = std::make_shared<bool>(true);
    register_component_toggle(components, components_visible, observer);
  }

  // Render panorama position spheres and register callbacks
  bool has_panoramas = false;
  if (!pano_state->infos.empty()) {
    // Add orange spheres at panorama positions
    for (size_t i = 0; i < pano_state->infos.size(); ++i) {
      const auto &pi = pano_state->infos[i];
      if (!pi.pose_valid)
        continue;
      has_panoramas = true;
      auto sphere_name = fmt::format("pano_sphere_{}", i);
      float px = static_cast<float>(pi.px);
      float py = static_cast<float>(pi.py);
      float pz = static_cast<float>(pi.pz);
      observer.viewer_enqueue_task(
          [px, py, pz,
           sphere_name](const rux::VizualizationObserver::ViewerPtr &viewer,
                        const std::vector<int> &viewports) {
            viewer->addSphere(pcl::PointXYZ(px, py, pz), 0.15, 1.0, 0.5, 0.0,
                              sphere_name, viewports[0]);
          });
    }

    // Register Shift+Click point picking callback for panorama entry
    observer.viewer_enqueue_task(
        [pano_state, project_path,
         &observer](const rux::VizualizationObserver::ViewerPtr &viewer,
                    const std::vector<int> &) {
          viewer->registerPointPickingCallback(
              [pano_state, project_path, &observer](
                  const pcl::visualization::PointPickingEvent &event) {
                if (pano_state->immersive)
                  return;
                if (event.getPointIndex() == -1)
                  return;

                float x, y, z;
                event.getPoint(x, y, z);

                // Find nearest panorama sphere
                int best_idx = -1;
                double best_dist = 0.5; // threshold in meters
                for (size_t i = 0; i < pano_state->infos.size(); ++i) {
                  const auto &pi = pano_state->infos[i];
                  if (!pi.pose_valid)
                    continue;
                  double dx = x - pi.px;
                  double dy = y - pi.py;
                  double dz = z - pi.pz;
                  double d = std::sqrt(dx * dx + dy * dy + dz * dz);
                  if (d < best_dist) {
                    best_dist = d;
                    best_idx = static_cast<int>(i);
                  }
                }

                if (best_idx >= 0) {
                  observer.viewer_enqueue_task(
                      [pano_state, best_idx, project_path](
                          const rux::VizualizationObserver::ViewerPtr &v,
                          const std::vector<int> &vp) {
                        enter_panorama_mode(pano_state, best_idx, project_path,
                                            v, vp);
                      });
                }
              });
        });

    // Register Escape key to exit panorama mode
    observer.viewer_enqueue_task(
        [pano_state,
         &observer](const rux::VizualizationObserver::ViewerPtr &viewer,
                    const std::vector<int> &) {
          viewer->registerKeyboardCallback(
              [pano_state,
               &observer](const pcl::visualization::KeyboardEvent &event) {
                if (event.getKeySym() == "Escape" && event.keyDown() &&
                    pano_state->immersive) {
                  observer.viewer_enqueue_task(
                      [pano_state](
                          const rux::VizualizationObserver::ViewerPtr &v,
                          const std::vector<int> &vp) {
                        exit_panorama_mode(pano_state, v, vp);
                      });
                }
              });
        });

    // Register [ / ] keys for cycling panoramas in immersive mode
    observer.viewer_enqueue_task(
        [pano_state, project_path, &clouds, &meshes,
         &observer](const rux::VizualizationObserver::ViewerPtr &viewer,
                    const std::vector<int> &) {
          viewer->registerKeyboardCallback(
              [pano_state, project_path, &clouds, &meshes,
               &observer](const pcl::visualization::KeyboardEvent &event) {
                if (!pano_state->immersive || !event.keyDown())
                  return;

                int delta = 0;
                if (event.getKeySym() == "bracketright")
                  delta = 1;
                else if (event.getKeySym() == "bracketleft")
                  delta = -1;
                else
                  return;

                // Find next panorama with a valid pose
                int count = static_cast<int>(pano_state->infos.size());
                int cur = pano_state->active_index;
                for (int step = 1; step < count; ++step) {
                  int idx = ((cur + delta * step) % count + count) % count;
                  if (pano_state->infos[static_cast<size_t>(idx)].pose_valid) {
                    observer.viewer_enqueue_task(
                        [pano_state, idx, project_path, &clouds,
                         &meshes](
                            const rux::VizualizationObserver::ViewerPtr &v,
                            const std::vector<int> &vp) {
                          deactivate_panorama_skybox(pano_state->skybox, v,
                                                     vp[0]);
                          const auto &info =
                              pano_state->infos[static_cast<size_t>(idx)];
                          activate_panorama_skybox(project_path, info,
                                                   pano_state->skybox, v,
                                                   vp[0]);
                          // Update camera to new panorama position
                          if (info.pose_valid) {
                            auto *renderer = viewport_renderer(v, vp[0]);
                            auto *cam = renderer->GetActiveCamera();
                            cam->SetPosition(info.px, info.py, info.pz);
                            cam->SetFocalPoint(info.px + info.pose[2],
                                               info.py + info.pose[6],
                                               info.pz + info.pose[10]);
                            cam->SetViewUp(-info.pose[1], -info.pose[5],
                                           -info.pose[9]);
                            cam->SetClippingRange(0.01, 100.0);
                          }
                          pano_state->active_index = idx;
                          spdlog::info("Panorama: {} [{}]", info.filename,
                                       idx + 1);
                        });
                    break;
                  }
                }
              });
        });
  }

  register_help_callback(clouds, meshes, labels, !components.empty(),
                         has_panoramas, observer);

  // === Phase 3: Display Summary ===
  spdlog::info("Press 'h' for keyboard controls");
  int pano_count = 0;
  for (const auto &pi : pano_state->infos)
    if (pi.pose_valid)
      ++pano_count;
  spdlog::info(
      "Loaded: {} cloud(s), {} mesh(es), {} label(s), {} component(s), {} "
      "panorama(s)",
      clouds.size(), meshes.size(), labels.size(), components.size(),
      pano_count);

  // We need to wait here otherwise the clouds go out of scope and get destroyed
  // and the viewer segfaults when it tries to access them.
  observer.viewer_wait_for_user();

  return 0;
}
