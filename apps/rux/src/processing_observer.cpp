// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "processing_observer.hpp"

#include <pcl/visualization/pcl_visualizer.h>
#include <spdlog/spdlog.h>
#include <fmt/format.h>

#include <atomic>
#include <functional>
#include <latch>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

namespace rux {
namespace {
constexpr int kViewerSpinTimeoutMs = 100;

class SpdlogProcessingObserver final : public ReUseX::core::IProcessingObserver {
public:
  using ViewerPtr = std::shared_ptr<pcl::visualization::PCLVisualizer>;
  using VizTask = std::function<void(const ViewerPtr &)>;

  void on_stage_started(std::string_view stage) override {
    spdlog::info("Processing stage: {}", stage);
    enqueue_status(fmt::format("Stage: {}", stage));
  }

  void on_progress(std::string_view stage, float progress) override {
    spdlog::info("Processing progress [{}]: {:.0f}%", stage, progress * 100.0F);
    enqueue_progress(fmt::format("{}: {:.0f}%", stage, progress * 100.0F));
  }

  void on_warning(std::string_view message) override {
    spdlog::warn("Processing warning: {}", message);
    enqueue_warning(std::string(message));
  }

  void on_error(std::string_view message) override {
    spdlog::error("Processing error: {}", message);
    enqueue_error(std::string(message));
  }

  void start() {
    if (viz_thread_.joinable()) {
      viz_thread_.join();
    }

    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true,
                                          std::memory_order_acq_rel,
                                          std::memory_order_acquire)) {
      return;
    }

    stop_requested_.store(false, std::memory_order_release);
    auto initialized = std::make_shared<std::latch>(1);
    viz_thread_ = std::thread([this, initialized]() {
      viewer_loop(*initialized);
    });
    initialized->wait();
  }

  void stop() {
    running_.store(false, std::memory_order_release);
    stop_requested_.store(true, std::memory_order_release);
    if (viz_thread_.joinable()) {
      viz_thread_.join();
    }
  }

  ~SpdlogProcessingObserver() override { stop(); }

private:
  void enqueue_status(std::string message) {
    enqueue_task([message = std::move(message)](const ViewerPtr &viewer) {
      viewer->removeShape("reusex_status");
      viewer->addText(message, 10, 10, 16.0, 1.0, 1.0, 1.0, "reusex_status");
    });
  }

  void enqueue_progress(std::string message) {
    enqueue_task([message = std::move(message)](const ViewerPtr &viewer) {
      viewer->removeShape("reusex_progress");
      viewer->addText(message, 10, 30, 16.0, 0.6, 0.9, 1.0, "reusex_progress");
    });
  }

  void enqueue_warning(std::string message) {
    enqueue_task([message = std::move(message)](const ViewerPtr &viewer) {
      viewer->removeShape("reusex_warning");
      viewer->addText(message, 10, 50, 16.0, 1.0, 0.8, 0.0, "reusex_warning");
    });
  }

  void enqueue_error(std::string message) {
    enqueue_task([message = std::move(message)](const ViewerPtr &viewer) {
      viewer->removeShape("reusex_error");
      viewer->addText(message, 10, 70, 16.0, 1.0, 0.2, 0.2, "reusex_error");
    });
  }

  void enqueue_task(VizTask task) {
    if (!running_.load(std::memory_order_acquire)) {
      return;
    }
    std::lock_guard<std::mutex> lock(queue_mutex_);
    task_queue_.push(std::move(task));
  }

  void drain_tasks(const ViewerPtr &viewer) {
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

  void viewer_loop(std::latch &initialized) {
    ViewerPtr viewer =
        std::make_shared<pcl::visualization::PCLVisualizer>("ReUseX Processing");
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

  std::thread viz_thread_;
  std::queue<VizTask> task_queue_;
  std::mutex queue_mutex_;
  std::atomic_bool stop_requested_{false};
  std::atomic_bool running_{false};
};

SpdlogProcessingObserver g_processing_observer;

} // namespace

void enable_processing_observer(bool enabled) {
  if (enabled) {
    g_processing_observer.start();
    ReUseX::core::set_processing_observer(&g_processing_observer);
    return;
  }
  ReUseX::core::reset_processing_observer();
  g_processing_observer.stop();
}

} // namespace rux
