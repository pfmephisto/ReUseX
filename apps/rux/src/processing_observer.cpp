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
};

// VizualizationObserver

void VizualizationObserver::on_process_started(std::string_view process,
                                               size_t total) {
  progress_logger_ = std::make_unique<spdmon::LoggerProgress>(
      fmt::format("Processing: {}", process), total);
  // spdlog::info("Processing started: {}", process);
  // enqueue_status(fmt::format("Process: {}", process));
}

void VizualizationObserver::on_process_finished(std::string_view) {
  progress_logger_.reset();
}

void VizualizationObserver::on_process_updated(std::string_view,
                                               size_t increment) {
  *progress_logger_ += increment;
}

void VizualizationObserver::start() {
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

void VizualizationObserver::stop() {
  running_.store(false, std::memory_order_release);
  stop_requested_.store(true, std::memory_order_release);
  if (viz_thread_.joinable()) {
    viz_thread_.join();
  }
}

bool VizualizationObserver::is_active() const {
  return running_.load(std::memory_order_acquire);
}

void VizualizationObserver::wait_for_user() {
  spdlog::trace("Waiting for user to close visualization");
  if (running_.load(std::memory_order_acquire) && viz_thread_.joinable()) {
    spdlog::info("Visualization window open. Close it to exit the program.");
    // Just join the thread - it will exit when user closes window
    // Don't set stop_requested_, let the viewer close naturally
    viz_thread_.join();
  }
}

VizualizationObserver::~VizualizationObserver() { stop(); }

void VizualizationObserver::enqueue_status(std::string message) {
  enqueue_task([message = std::move(message)](const ViewerPtr &viewer) {
    viewer->removeShape("reusex_status");
    viewer->addText(message, 10, 10, 16.0, 1.0, 1.0, 1.0, "reusex_status");
  });
}

void VizualizationObserver::enqueue_progress(std::string message) {
  enqueue_task([message = std::move(message)](const ViewerPtr &viewer) {
    viewer->removeShape("reusex_progress");
    viewer->addText(message, 10, 30, 16.0, 0.6, 0.9, 1.0, "reusex_progress");
  });
}

void VizualizationObserver::enqueue_warning(std::string message) {
  enqueue_task([message = std::move(message)](const ViewerPtr &viewer) {
    viewer->removeShape("reusex_warning");
    viewer->addText(message, 10, 50, 16.0, 1.0, 0.8, 0.0, "reusex_warning");
  });
}

void VizualizationObserver::enqueue_error(std::string message) {
  enqueue_task([message = std::move(message)](const ViewerPtr &viewer) {
    viewer->removeShape("reusex_error");
    viewer->addText(message, 10, 70, 16.0, 1.0, 0.2, 0.2, "reusex_error");
  });
}

void VizualizationObserver::enqueue_task(VizTask task) {
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
    local_tasks.front()(viewer);
    local_tasks.pop();
  }
}

void VizualizationObserver::viewer_loop(std::latch &initialized) {
  spdlog::trace("Initializing PCLVisualizer in viewer loop");
  ViewerPtr viewer = std::make_shared<pcl::visualization::PCLVisualizer>("rux");
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  initialized.count_down();

  while (!stop_requested_.load(std::memory_order_acquire) &&
         !viewer->wasStopped()) {
    drain_tasks(viewer);
    viewer->spinOnce(kViewerSpinTimeoutMs);
  }
  drain_tasks(viewer);
  running_.store(false, std::memory_order_release);
}

} // namespace rux
