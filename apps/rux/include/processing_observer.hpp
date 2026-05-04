// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "spdmon.hpp"

#include <pcl/visualization/pcl_visualizer.h>
#include <reusex/core/processing_observer.hpp>
#include <reusex/geometry/CellComplex.hpp>
#include <reusex/types.hpp>

#include <latch>
#include <mutex>
#include <queue>

namespace rux {

class VizualizationObserver final : public reusex::core::IVisualObserver,
                                    public reusex::core::IProgressObserver {
    public:
  using ViewerPtr = std::shared_ptr<pcl::visualization::PCLVisualizer>;
  using VizTask =
      std::function<void(const ViewerPtr &, const std::vector<int> &viewports)>;
  using Pair = std::pair<Eigen::Vector4d, Eigen::Vector3d>;
  using PlanePair = std::pair<Pair, Pair>;

  ~VizualizationObserver() override;

  // Viewer callbacks

  // Default implementation does nothing - users can override this method to add
  template <typename T>
  void viewer_add_geometry(std::string_view
                           /*name*/,
                           const T & /*geometry*/, std::string_view /*stage*/,
                           int /*idx*/) {};

  // Override virtual method for planes
  void viewer_add_plane(std::string_view name, const Eigen::Vector4d &plane,
                        reusex::core::Stage stage, int idx) override;
  void viewer_add_plane(std::string_view name, const Pair &plane,
                        reusex::core::Stage stage, int idx) override;
  void viewer_add_plane_pair(std::string_view name, const PlanePair &pair,
                             reusex::core::Stage stage, int idx) override;
  void viewer_add_cell_complex(
      std::string_view name,
      const std::shared_ptr<reusex::geometry::CellComplex> &cc,
      reusex::core::Stage stage, int idx) override;

  void viewer_add_cloud(std::string_view name,
                        const reusex::CloudConstPtr &cloud,
                        reusex::core::Stage stage, int idx) override;

  void viewer_add_camera_frustum(std::string_view name, double focal_x,
                                 double focal_y, int image_width,
                                 int image_height,
                                 const Eigen::Affine3f &pose,
                                 reusex::core::Stage stage, int idx) override;

  void viewer_add_visibility_graph(std::string_view name,
                                   const reusex::CloudLocPtr &disc_points,
                                   const std::shared_ptr<std::vector<pcl::Vertices>> &disc_outlines,
                                   const pcl::CorrespondencesPtr &edges,
                                   reusex::core::Stage stage, int idx) override;

  void viewer_add_labeled_cloud(std::string_view name,
                                const reusex::CloudConstPtr &cloud,
                                const reusex::CloudLConstPtr &labels,
                                reusex::core::Stage stage, int idx) override;

  // Progress bar callbacks
  void on_process_started(reusex::core::Stage stage, size_t total) override;
  void on_process_finished(reusex::core::Stage stage) override;
  void on_process_updated(reusex::core::Stage stage, size_t increment) override;

  // Viewer control methods
  void viewer_start();
  void viewer_stop();
  bool viewer_is_active() const;
  void viewer_wait_for_user();
  void viewer_enqueue_task(VizTask task);
  void viewer_request_viewports(size_t num_viewports);

    private:
  void viewer_drain_tasks(const ViewerPtr &viewer);
  void viewer_loop(std::latch &initialized);

  std::vector<int> viewports_ = std::vector<int>(1);
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
