// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "view/component_renderer.hpp"

#include <fmt/format.h>
#include <spdlog/spdlog.h>

namespace rux::view {

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
      r = 0.0;
      g = 1.0;
      b = 1.0; // cyan for doors
    } else if (comp.type == reusex::geometry::ComponentType::wall) {
      r = 1.0;
      g = 1.0;
      b = 0.0; // yellow for walls
    }

    for (size_t i = 0; i < verts.size(); ++i) {
      size_t j = (i + 1) % verts.size();
      auto line_name = fmt::format("comp_{}_{}", ci, i);
      auto p1 = verts[i], p2 = verts[j];
      observer.viewer_enqueue_task(
          [p1, p2, line_name, r, g,
           b](const rux::VizualizationObserver::ViewerPtr &viewer,
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

void register_help_callback(const std::vector<LoadedCloud> &clouds,
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
                  spdlog::info(
                      "  Shift+Click: Enter 360 panorama (on orange sphere)");
                  spdlog::info("  Escape: Exit panorama mode");
                  spdlog::info(
                      "  [/]: Previous/Next panorama (in panorama mode)");
                }
                spdlog::info("  h: Show this help");
                spdlog::info("  q: Quit viewer");
              }
            });
      });
}

} // namespace rux::view
