// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "processing_observer.hpp"
#include "spdmon.hpp"

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>

namespace rux {
namespace {
constexpr int kViewerSpinTimeoutMs = 100;
rux::VizualizationObserver g_processing_observer;
}; // namespace

/* Utility function to calculate viewport bounds for a given index and total
 * count. Uses recursive binary space partitioning to split the viewport area.
 *
 * @param index Index of the viewport (0-based)
 * @param total Total number of viewports
 * @param left Left bound of the current area (default 0.0)
 * @param top Top bound of the current area (default 0.0)
 * @param right Right bound of the current area (default 1.0)
 * @param bottom Bottom bound of the current area (default 1.0)
 * @param split_horizontal Whether to split horizontally first (default true)
 * @return Tuple of (left, top, right, bottom) bounds for the specified viewport
 */
std::tuple<float, float, float, float>
get_viewport_bounds(size_t index, size_t total, float left = 0.0f,
                    float top = 0.0f, float right = 1.0f, float bottom = 1.0f,
                    bool split_horizontal = true) {

  // Ratio used for odd viewport counts: first viewport gets this much space
  constexpr float ODD_VIEWPORT_RATIO = 0.6f;

  // Base case: single viewport gets full bounds
  if (total == 1) {
    return {left, top, right, bottom};
  }

  // Even count: binary split
  if (total % 2 == 0) {
    size_t half = total / 2;

    if (split_horizontal) {
      // Split horizontally (left/right regions)
      float mid = left + (right - left) * 0.5f;

      if (index < half) {
        // First half: left region, recurse with vertical split
        return get_viewport_bounds(index, half, left, top, mid, bottom, false);
      } else {
        // Second half: right region, recurse with vertical split
        return get_viewport_bounds(index - half, half, mid, top, right, bottom,
                                   false);
      }
    } else {
      // Split vertically (top/bottom regions)
      float mid = top + (bottom - top) * 0.5f;

      if (index < half) {
        // First half: top region, recurse with horizontal split
        return get_viewport_bounds(index, half, left, top, right, mid, true);
      } else {
        // Second half: bottom region, recurse with horizontal split
        return get_viewport_bounds(index - half, half, left, mid, right, bottom,
                                   true);
      }
    }
  }

  // Odd count: first viewport larger (60%), rest split recursively
  if (index == 0) {
    // First viewport gets the larger portion
    if (split_horizontal) {
      float mid = left + (right - left) * ODD_VIEWPORT_RATIO;
      return {left, top, mid, bottom};
    } else {
      float mid = top + (bottom - top) * ODD_VIEWPORT_RATIO;
      return {left, top, right, mid};
    }
  }

  // Remaining viewports recursively split the smaller region
  if (split_horizontal) {
    float mid = left + (right - left) * ODD_VIEWPORT_RATIO;
    return get_viewport_bounds(index - 1, total - 1, mid, top, right, bottom,
                               false);
  } else {
    float mid = top + (bottom - top) * ODD_VIEWPORT_RATIO;
    return get_viewport_bounds(index - 1, total - 1, left, mid, right, bottom,
                               true);
  }
}

VizualizationObserver::~VizualizationObserver() { viewer_stop(); }

// ============================================================================
// Viewer Callbacks
// ============================================================================

void VizualizationObserver::viewer_add_plane(std::string_view name,
                                             const Eigen::Vector4d &plane,
                                             ReUseX::core::Stage /*stage*/,
                                             int /*idx*/) {
  if (!viewer_is_active())
    return;

  viewer_enqueue_task([name = std::string(name),
                       plane](const ViewerPtr & /*viewer*/,
                              const std::vector<int> & /*viewports*/) {
    // int vp = viewports.empty() ? 0 : viewports[0];

    // TODO: Implement plane visualization

    // auto sel = [&](size_t idx) {
    //   return std::make_pair(planes[idx], centroids[idx]);
    // };

    // auto vertical_planes = vertical | ranges::views::transform(sel) |
    //                        ranges::to<std::vector>();
    // viewer->addPlanes(vertical_planes, "vertical_planes", vps->at(0));

    // auto horizontal_planes = horizontal | ranges::views::transform(sel) |
    //                          ranges::to<std::vector>();
    // viewer->addPlanes(horizontal_planes, "horizontal_planes",
    // vps->at(0));

    // auto plane_pairs = pairs | ranges::views::transform([&](auto const
    // &p) {
    //                      return std::make_pair(sel(p.first),
    //                      sel(p.second));
    //                    }) |
    //                    ranges::to<std::vector>();
    // viewer->addPlanePairs(plane_pairs, "plain_pairs", vps->at(0));
  });
}

void VizualizationObserver::viewer_add_plane(std::string_view name,
                                             const Pair &plane,
                                             ReUseX::core::Stage stage,
                                             int idx) {
  // Check if viewer is active before proceeding
  if (!viewer_is_active())
    return;

  // Check if we want this to be added to a specific viewport based on stage and
  // index
  int vp = viewports_.empty() ? 0 : viewports_[0];
  if (stage == ReUseX::core::Stage::Default) {
    if (viewports_.size() == 4)
      vp = viewports_[0];
  }

  // Set the color based on the index using Glasbey LUT for better visibility
  const size_t n_colors = pcl::GlasbeyLUT::size();
  auto color = pcl::GlasbeyLUT::at(idx % n_colors);

  pcl::ModelCoefficients coeffs;
  coeffs.values = {
      static_cast<float>(plane.first[0]), static_cast<float>(plane.first[1]),
      static_cast<float>(plane.first[2]), static_cast<float>(plane.first[3])};

  viewer_enqueue_task([name = std::string(name), coeffs, color,
                       vp](const ViewerPtr &viewer, const std::vector<int> &) {
    viewer->addPlane(coeffs, fmt::format("{}_plane", name), vp);
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR,
        static_cast<double>(color.r) / 255.0,
        static_cast<double>(color.g) / 255.0,
        static_cast<double>(color.b) / 255.0, std::string{name}, vp);
  });
}

void VizualizationObserver::viewer_add_plane_pair(std::string_view name,
                                                  const PlanePair &pair,
                                                  ReUseX::core::Stage stage,
                                                  int /*idx*/) {
  if (!viewer_is_active())
    return;

  // Viewport selection logic (same as viewer_add_plane(Pair))
  int vp = viewports_.empty() ? 0 : viewports_[0];
  if (stage == ReUseX::core::Stage::Default) {
    if (viewports_.size() == 4)
      vp = viewports_[0];
  }

  // Copy visualization code from addPair()
  viewer_enqueue_task(
      [name = std::string(name), pair,
       vp](const ViewerPtr &viewer, const std::vector<int> & /*viewports*/) {
        auto [plane_i, origin_i] = pair.first;
        auto [plane_j, origin_j] = pair.second;

        ReUseX::PointT p1, p2;
        p1.getVector3fMap() = origin_i.head<3>().cast<float>();
        p2.getVector3fMap() = origin_j.head<3>().cast<float>();
        viewer->addLine<ReUseX::PointT>(p1, p2, 0.0, 0.0, 1.0,
                                        fmt::format("{}_line", name), vp);
      });
}

void VizualizationObserver::viewer_add_cell_complex(
    std::string_view name,
    const std::shared_ptr<ReUseX::geometry::CellComplex> &cc,
    ReUseX::core::Stage stage, int /*idx*/) {
  if (!viewer_is_active())
    return;

  // Viewport selection logic
  int vp = viewports_.empty() ? 0 : viewports_[0];
  if (stage == ReUseX::core::Stage::Default) {
    if (viewports_.size() == 4)
      vp = viewports_[0];
  }

  // Copy visualization code based on name
  viewer_enqueue_task([name = std::string(name), cc,
                       vp](const ViewerPtr &viewer,
                           const std::vector<int> & /*viewports*/) {
    if (name.find("room_prob") != std::string::npos) {
      // Copy of addRoomProbabilities()
      spdlog::trace("Displaying room probabilities");
      auto c_rp = cc->property_map<ReUseX::geometry::CellComplex::Vertex,
                                   std::vector<double>>("c:room_probabilities");
      // Create a Lut for the room labels
      std::vector<pcl::RGB> lut(cc->n_rooms + 1);
      for (size_t i = 0; i < lut.size(); ++i)
        lut[i] = pcl::GlasbeyLUT::at(i);

      for (auto cit = cc->cells_begin(); cit != cc->cells_end(); ++cit) {
        const auto id = (*cc)[*cit].id;
        const auto &vec = c_rp[*cit];

        // Mix the colors based on the probabilities
        pcl::RGB color{0, 0, 0};
        for (size_t i = 0; i < vec.size(); ++i) {
          const auto c = lut[i];
          color.r += static_cast<uint8_t>(c.r * vec[i]);
          color.g += static_cast<uint8_t>(c.g * vec[i]);
          color.b += static_cast<uint8_t>(c.b * vec[i]);
        }

        std::string name_ = fmt::format("{}_cell_{}-prob", name, id);
        ReUseX::PointT p;
        p.x = (*cc)[*cit].pos[0];
        p.y = (*cc)[*cit].pos[1];
        p.z = (*cc)[*cit].pos[2];

        auto prob_outside = vec[0];
        double r = 0.4 * (1.0 - prob_outside) + 0.1;

        if (prob_outside > 0.9)
          continue; // Skip mostly outside cells

        viewer->addSphere(p, r, static_cast<double>(color.r) / 255.0,
                          static_cast<double>(color.g) / 255.0,
                          static_cast<double>(color.b) / 255.0, name_, vp);
        viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, name_, vp);
      }
    } else if (name.find("support_prob") != std::string::npos) {
      // Copy of addSupportProbabilities()
      spdlog::trace("Displaying face support probabilities");
      auto vertices = ReUseX::CloudPtr(new ReUseX::Cloud);
      vertices->points.resize(cc->num_vertices());
      for (auto vit = cc->vertices_begin(); vit != cc->vertices_end(); ++vit) {
        const auto id = (*cc)[*vit].id;
        const auto pos = (*cc)[*vit].pos;
        vertices->points[id].x = pos[0];
        vertices->points[id].y = pos[1];
        vertices->points[id].z = pos[2];
      }

      auto f_sp =
          cc->property_map<ReUseX::geometry::CellComplex::Vertex, double>(
              "f:support_probability");
      for (auto fit = cc->faces_begin(); fit != cc->faces_end(); ++fit) {
        const auto id = (*cc)[*fit].id;
        const auto prob = f_sp[*fit] == -1.0 ? 1.0 : f_sp[*fit];

        const auto c = f_sp[*fit] == -1.0
                           ? pcl::RGB(255, 0, 0)
                           : pcl::ViridisLUT::at(static_cast<size_t>(
                                 prob * pcl::ViridisLUT::size()));

        std::string name_ = fmt::format("{}_face_{}-prob", name, id);

        // Create face
        pcl::Vertices face{};
        for (auto vit = cc->vertices_begin(*fit); vit != cc->vertices_end(*fit);
             ++vit)
          face.vertices.push_back(static_cast<int>((*cc)[*vit].id));

        // Close the face
        if (!face.vertices.empty())
          face.vertices.push_back(face.vertices[0]);
        else
          continue;

        viewer->addPolygonMesh<ReUseX::PointT>(vertices, {face}, name_, vp);
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR,
            static_cast<double>(c.r) / 255.0, static_cast<double>(c.g) / 255.0,
            static_cast<double>(c.b) / 255.0, name_, vp);

        auto opacity = f_sp[*fit] == -1.0 ? 1.0 : prob * 0.8 + 0.2;
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, name_, vp);

        if (f_sp[*fit] != -1.0)
          continue; // Only display text for unsupported faces

        auto plane_id = std::get<FaceData>((*cc)[*fit].data).plane_id;

        viewer->addText3D(
            fmt::format("P{}", plane_id),
            ReUseX::PointT{static_cast<float>((*cc)[*fit].pos[0]),
                           static_cast<float>((*cc)[*fit].pos[1]),
                           static_cast<float>((*cc)[*fit].pos[2])},
            0.3, 1.0, 1.0, 1.0, fmt::format("text_face_{}", id), vp);
      }
    } else if (name.find("rooms") != std::string::npos) {
      // Note: addRooms() requires a results parameter that we don't have
      // Fall back to basic cell complex visualization
      spdlog::warn(
          "addRooms() requires results parameter, using basic cell complex "
          "visualization for '{}'",
          name);
      // Fall through to default case
    }

    // Default or fallback: basic cell complex visualization (copy of
    // addCellComplex())
    if (name.find("room_prob") == std::string::npos &&
        name.find("support_prob") == std::string::npos) {
      spdlog::trace("Displaying cell complex vertices");
      // INFO: Display vertices
      auto vertices = ReUseX::CloudPtr(new ReUseX::Cloud);
      vertices->points.resize(cc->num_vertices());
      for (auto vit = cc->vertices_begin(); vit != cc->vertices_end(); ++vit) {
        const auto id = (*cc)[*vit].id;
        const auto pos = (*cc)[*vit].pos;
        vertices->points[id].x = pos[0];
        vertices->points[id].y = pos[1];
        vertices->points[id].z = pos[2];
      }
      const std::string v_name = fmt::format("{}_vertices", name);
      pcl::visualization::PointCloudColorHandlerCustom<ReUseX::PointT>
          v_color_handler(vertices, 0, 0, 255);
      viewer->addPointCloud<ReUseX::PointT>(vertices, v_color_handler, v_name,
                                            vp);
      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, v_name, vp);

      // INFO: Display faces
      spdlog::trace("Displaying cell complex faces");
      auto faces = ReUseX::CloudPtr(new ReUseX::Cloud);
      faces->points.resize(cc->num_faces());
      for (auto fit = cc->faces_begin(); fit != cc->faces_end(); ++fit) {
        const auto id = (*cc)[*fit].id;
        const auto pos = (*cc)[*fit].pos;
        faces->points[id].x = pos[0];
        faces->points[id].y = pos[1];
        faces->points[id].z = pos[2];
      }
      const std::string f_name = fmt::format("{}_faces", name);
      pcl::visualization::PointCloudColorHandlerCustom<ReUseX::PointT>
          f_color_handler(faces, 0, 255, 0);
      viewer->addPointCloud<ReUseX::PointT>(faces, f_color_handler, f_name, vp);
      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, f_name, vp);

      // INFO: Display cell centers
      spdlog::trace("Displaying cell complex cell centers");
      auto cells = ReUseX::CloudPtr(new ReUseX::Cloud);
      cells->points.resize(cc->num_cells());
      for (auto cit = cc->cells_begin(); cit != cc->cells_end(); ++cit) {
        const auto id = (*cc)[*cit].id;
        const auto pos = (*cc)[*cit].pos;
        cells->points[id].x = pos[0];
        cells->points[id].y = pos[1];
        cells->points[id].z = pos[2];
      }
      pcl::visualization::PointCloudColorHandlerCustom<ReUseX::PointT>
          c_color_handler(cells, 255, 0, 0);
      const std::string c_name = fmt::format("{}_centers", name);
      viewer->addPointCloud<ReUseX::PointT>(cells, c_color_handler, c_name, vp);
      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, c_name, vp);

      // INFO: Display Cell Face Correspondences
      spdlog::trace("Displaying cell face correspondences");
      std::string cf_name = "correspondences_cf";
      pcl::CorrespondencesPtr cf(new pcl::Correspondences);
      for (auto cit = cc->cells_begin(); cit != cc->cells_end(); ++cit) {
        const auto cid = (*cc)[*cit].id;
        for (auto fit = cc->faces_begin(*cit); fit != cc->faces_end(*cit);
             ++fit) {
          const auto fid = (*cc)[*fit].id;
          cf->emplace_back(static_cast<int>(cid), static_cast<int>(fid), 0.0);
        }
      }
      viewer->addCorrespondences<ReUseX::PointT>(cells, faces, *cf, cf_name,
                                                 vp);
      viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, cf_name, vp);

      // INFO: Display Face Vertex Correspondences
      spdlog::trace("Displaying face vertex correspondences");
      std::string fv_name = "correspondences_fv";
      pcl::CorrespondencesPtr fv(new pcl::Correspondences);
      for (auto fit = cc->faces_begin(); fit != cc->faces_end(); ++fit) {
        const auto fid = (*cc)[*fit].id;
        for (auto vit = cc->vertices_begin(*fit); vit != cc->vertices_end(*fit);
             ++vit) {
          const auto vid = (*cc)[*vit].id;
          fv->emplace_back(static_cast<int>(fid), static_cast<int>(vid), 0.0);
        }
      }
      viewer->addCorrespondences<ReUseX::PointT>(faces, vertices, *fv, fv_name,
                                                 vp);
      viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0.5, fv_name, vp);
    }
  });
}

void VizualizationObserver::viewer_add_cloud(
    std::string_view /*name*/, const ReUseX::CloudConstPtr & /*cloud*/,
    ReUseX::core::Stage /*stage*/, int /*idx*/) {
  if (!viewer_is_active())
    return;

  // viewer_enqueue_task(
  //     [name = std::string(name), cloud,
  //      stage = std::string(stage)](const ViewerPtr &viewer,
  //                                  const std::vector<int> &viewports) {
  //       int vp = viewports.empty() ? 0 : viewports[0];
  //       viewer->addPointCloud(cloud, fmt::format("{}_cloud", name), vp);
  //     });
}

// ============================================================================
// Progress Bar Callbacks
// ===========================================================================
void VizualizationObserver::on_process_started(ReUseX::core::Stage stage,
                                               size_t total) {
  progress_logger_ = std::make_unique<spdmon::LoggerProgress>(
      fmt::format("Processing: {}", ReUseX::core::to_string(stage)), total);
}

void VizualizationObserver::on_process_finished(ReUseX::core::Stage) {
  progress_logger_.reset();
}

void VizualizationObserver::on_process_updated(ReUseX::core::Stage,
                                               size_t increment) {
  *progress_logger_ += increment;
}

void VizualizationObserver::viewer_start() {
  if (running_.load(std::memory_order_acquire)) {
    spdlog::trace("Viewer already running, skipping start");
    return;
  }

  spdlog::trace("Starting visualization observer thread");
  if (viz_thread_.joinable()) {
    spdlog::trace(
        "Visualization thread already running, joining existing thread");
    viz_thread_.join();
  }

  bool expected = false;
  spdlog::trace("Attempting to start visualization thread");
  if (!running_.compare_exchange_strong(expected, true,
                                        std::memory_order_acq_rel,
                                        std::memory_order_acquire)) {
    return;
  }

  spdlog::trace("Visualization thread started successfully");
  stop_requested_.store(false, std::memory_order_release);
  spdlog::trace("Creating latch for viewer initialization");
  auto initialized = std::make_shared<std::latch>(1);
  spdlog::trace("Launching visualization thread");
  viz_thread_ =
      std::thread([this, initialized]() { viewer_loop(*initialized); });
  spdlog::trace("Waiting for viewer initialization");
  initialized->wait();
}

void VizualizationObserver::viewer_stop() {
  running_.store(false, std::memory_order_release);
  stop_requested_.store(true, std::memory_order_release);
  if (viz_thread_.joinable()) {
    viz_thread_.join();
  }
}

bool VizualizationObserver::viewer_is_active() const {
  return running_.load(std::memory_order_acquire);
}

void VizualizationObserver::viewer_wait_for_user() {
  spdlog::trace("Waiting for user to close visualization");
  if (running_.load(std::memory_order_acquire) && viz_thread_.joinable()) {
    spdlog::info("Visualization window open. Close it to exit the program.");
    // Just join the thread - it will exit when user closes window
    // Don't set stop_requested_, let the viewer close naturally
    viz_thread_.join();
  }
}

void VizualizationObserver::viewer_enqueue_task(VizTask task) {
  if (!running_.load(std::memory_order_acquire)) {
    return;
  }
  std::lock_guard<std::mutex> lock(queue_mutex_);
  task_queue_.push(std::move(task));
}

void VizualizationObserver::viewer_request_viewports(size_t num_viewports) {
  if (num_viewports == viewports_.size())
    return;

  bool is_active = viewer_is_active();
  if (is_active)
    viewer_stop();

  viewports_ = std::vector<int>(num_viewports);

  if (is_active)
    viewer_start();
}

void VizualizationObserver::viewer_drain_tasks(const ViewerPtr &viewer) {
  std::queue<VizTask> local_tasks;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    std::swap(local_tasks, task_queue_);
  }

  while (!local_tasks.empty()) {
    local_tasks.front()(viewer, viewports_);
    local_tasks.pop();
  }
}

void VizualizationObserver::viewer_loop(std::latch &initialized) {
  spdlog::trace("Initializing PCLVisualizer in viewer loop");
  ViewerPtr viewer = std::make_shared<pcl::visualization::PCLVisualizer>("rux");

  // Create viewport
  for (size_t i = 0; i < viewports_.size(); ++i) {
    auto const [l, t, r, b] = get_viewport_bounds(i, viewports_.size());
    viewer->createViewPort(l, t, r, b, viewports_[i]);
    viewer->setBackgroundColor(255, 255, 255, viewports_[i]);
    viewer->addCoordinateSystem(1.0, fmt::format("vp{}", i), viewports_[i]);
  }

  // Initialize camera with proper orientation (fix upside-down issue)
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, 10, // Camera position
                            0, 0, 0,  // Look at point
                            0, -1,
                            0); // Up vector (negative Y to fix orientation)

  // Notify that viewer is initialized and ready
  initialized.count_down();

  while (!stop_requested_.load(std::memory_order_acquire) &&
         !viewer->wasStopped()) {
    viewer_drain_tasks(viewer);
    viewer->spinOnce(kViewerSpinTimeoutMs);
  }
  viewer_drain_tasks(viewer);
  viewer->close();
  running_.store(false, std::memory_order_release);
}

void setup_processing_observer() {
  ReUseX::core::set_processing_observer(&g_processing_observer);
}
void start_viewer() { g_processing_observer.viewer_start(); }
void wait_for_viewer() {
  if (g_processing_observer.viewer_is_active()) {
    g_processing_observer.viewer_wait_for_user();
  }
}
VizualizationObserver &get_processing_observer() {
  return g_processing_observer;
}
} // namespace rux
