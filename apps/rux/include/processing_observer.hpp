// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "spdmon.hpp"

#include <pcl/visualization/pcl_visualizer.h>
#include <reusex/core/processing_observer.hpp>

#include <latch>
#include <mutex>
#include <queue>

namespace rux {

class VizualizationObserver final : public ReUseX::core::IProcessingObserver {
    public:
  using ViewerPtr = std::shared_ptr<pcl::visualization::PCLVisualizer>;
  using VizTask = std::function<void(const ViewerPtr &)>;

  void on_process_started(std::string_view process, size_t total) override;
  void on_process_finished(std::string_view) override;
  void on_process_updated(std::string_view, size_t increment) override;

  void start();
  void stop();
  bool is_active() const;
  void wait_for_user();

  ~VizualizationObserver() override;

    private:
  void enqueue_status(std::string message);
  void enqueue_progress(std::string message);
  void enqueue_warning(std::string message);
  void enqueue_error(std::string message);
  void enqueue_task(VizTask task);
  void drain_tasks(const ViewerPtr &viewer);
  void viewer_loop(std::latch &initialized);

  std::thread viz_thread_;
  std::queue<VizTask> task_queue_;
  std::mutex queue_mutex_;
  std::atomic_bool stop_requested_{false};
  std::atomic_bool running_{false};
  std::unique_ptr<spdmon::LoggerProgress> progress_logger_;
};

// void enable_processing_observer(bool enabled);

} // namespace rux
