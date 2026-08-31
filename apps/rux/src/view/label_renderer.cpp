// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "view/label_renderer.hpp"

#include <fmt/format.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <spdlog/spdlog.h>

#include <algorithm>

namespace rux::view {

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
  int overflow = static_cast<int>(info.legend_entries.size()) - max_entries;
  std::string trailer_id;
  if (overflow > 0) {
    trailer_id = "legend_overflow";
    ids.push_back(trailer_id);
  }

  state->legend_ids = ids;

  auto legend = info.legend_entries;
  auto db_name = info.db_name;
  observer.viewer_enqueue_task(
      [legend, db_name, count, overflow,
       trailer_id](const rux::VizualizationObserver::ViewerPtr &viewer,
                   const std::vector<int> &) {
        // Title
        viewer->addText(db_name, 10, 30, 16, 0.1, 0.1, 0.1, "legend_title");

        // Entries
        for (int i = 0; i < count; ++i) {
          const auto &e = legend[static_cast<size_t>(i)];
          int y = 55 + i * 22;
          viewer->addText("##", 10, y, 16,
                          static_cast<double>(e.color.r) / 255.0,
                          static_cast<double>(e.color.g) / 255.0,
                          static_cast<double>(e.color.b) / 255.0,
                          fmt::format("legend_swatch_{}", i));
          viewer->addText(fmt::format("{}: {}", e.label_id, e.name), 40, y, 14,
                          0.1, 0.1, 0.1, fmt::format("legend_text_{}", i));
        }

        // Overflow
        if (overflow > 0) {
          int y = 55 + count * 22;
          viewer->addText(fmt::format("... and {} more", overflow), 10, y, 12,
                          0.3, 0.3, 0.3, trailer_id);
        }
      });
}

void register_legend_toggle(const std::vector<LabelCloudInfo> &label_clouds,
                            std::shared_ptr<ViewerState> state,
                            rux::VizualizationObserver &observer) {
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

void register_label_toggles(const std::vector<LabelCloudInfo> &label_clouds,
                            std::shared_ptr<ViewerState> state,
                            rux::VizualizationObserver &observer) {
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

} // namespace rux::view
