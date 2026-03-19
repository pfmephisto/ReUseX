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
  using VizTask =
      std::function<void(const ViewerPtr &, const std::vector<int> &viewports)>;

  ~VizualizationObserver() override;

  void on_process_started(std::string_view process, size_t total) override;
  void on_process_finished(std::string_view) override;
  void on_process_updated(std::string_view, size_t increment) override;

  void start_viewer();
  void stop_viewer();
  bool is_viewer_active() const;
  void viewer_wait_for_user();
  void enqueue_viewer_task(VizTask task);

    private:
  void drain_tasks(const ViewerPtr &viewer);
  void viewer_loop(std::latch &initialized);

  std::vector<int> viewports_;
  std::thread viz_thread_;
  std::queue<VizTask> task_queue_;
  std::mutex queue_mutex_;
  std::atomic_bool stop_requested_{false};
  std::atomic_bool running_{false};
  std::unique_ptr<spdmon::LoggerProgress> progress_logger_;
};

void setup_processing_observer();
void start_viewer();
void wait_for_viewer();

// Return non-owning reference to the global processing observer instance
VizualizationObserver &get_processing_observer();
} // namespace rux
