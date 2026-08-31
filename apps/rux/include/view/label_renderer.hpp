// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "processing_observer.hpp"
#include "view/viewer_types.hpp"

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>
#include <vector>

namespace rux::view {

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

// ============================================================================
// LABEL OVERLAY / LEGEND
// ============================================================================

/// Remove the label legend overlay from the viewer.
void remove_label_legend(std::shared_ptr<ViewerState> state,
                         rux::VizualizationObserver &observer);

/// Add a label legend overlay to the viewer for the given label cloud info.
void add_label_legend(const LabelCloudInfo &info,
                      std::shared_ptr<ViewerState> state,
                      rux::VizualizationObserver &observer);

/// Register keyboard callback for toggling label legend (key 'l').
void register_legend_toggle(const std::vector<LabelCloudInfo> &label_clouds,
                            std::shared_ptr<ViewerState> state,
                            rux::VizualizationObserver &observer);

/// Register keyboard callbacks for toggling label overlays (keys '1'-'9').
///
/// @param label_clouds Vector of label cloud info to register toggles for
/// @param state Shared viewer state tracking currently visible label
/// @param observer Visualization observer for enqueueing viewer tasks
void register_label_toggles(const std::vector<LabelCloudInfo> &label_clouds,
                            std::shared_ptr<ViewerState> state,
                            rux::VizualizationObserver &observer);

} // namespace rux::view
