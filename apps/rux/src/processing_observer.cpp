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

//

// ============================================================================
// Progress Bar Callbacks
// ===========================================================================
void VizualizationObserver::on_process_started(std::string_view process,
                                               size_t total) {
  progress_logger_ = std::make_unique<spdmon::LoggerProgress>(
      fmt::format("Processing: {}", process), total);
}

void VizualizationObserver::on_process_finished(std::string_view) {
  progress_logger_.reset();
}

void VizualizationObserver::on_process_updated(std::string_view,
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
