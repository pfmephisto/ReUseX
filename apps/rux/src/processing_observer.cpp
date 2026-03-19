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

// Utility to calculate viewport bounds for a given index and total number of
// viewports
std::tuple<float, float, float, float> get_viewport_bounds(size_t index,
                                                           size_t total) {
  float left = static_cast<float>(index) / static_cast<float>(total);
  float right = static_cast<float>(index + 1) / static_cast<float>(total);
  return {left, 0.0f, right, 1.0f};
}

VizualizationObserver::~VizualizationObserver() { stop_viewer(); }

// VizualizationObserver
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

void VizualizationObserver::start_viewer() {
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

void VizualizationObserver::stop_viewer() {
  running_.store(false, std::memory_order_release);
  stop_requested_.store(true, std::memory_order_release);
  if (viz_thread_.joinable()) {
    viz_thread_.join();
  }
}

bool VizualizationObserver::is_viewer_active() const {
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

void VizualizationObserver::enqueue_viewer_task(VizTask task) {
  if (!running_.load(std::memory_order_acquire)) {
    return;
  }
  std::lock_guard<std::mutex> lock(queue_mutex_);
  task_queue_.push(std::move(task));
}

void VizualizationObserver::drain_tasks(const ViewerPtr &viewer) {
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
  viewports_ = std::vector<int>(1);
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
    drain_tasks(viewer);
    viewer->spinOnce(kViewerSpinTimeoutMs);
  }
  drain_tasks(viewer);
  running_.store(false, std::memory_order_release);
}

void setup_processing_observer() {
  ReUseX::core::set_processing_observer(&g_processing_observer);
}
void start_viewer() { g_processing_observer.start_viewer(); }
void wait_for_viewer() {
  if (g_processing_observer.is_viewer_active()) {
    g_processing_observer.viewer_wait_for_user();
  }
}
VizualizationObserver &get_processing_observer() {
  return g_processing_observer;
}
} // namespace rux
