// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ReUseX/visualize/Visualizer.hpp"
#include "ReUseX/visualize/pcl.hpp"

#include <pcl/visualization/pcl_visualizer.h>
#include <spdlog/spdlog.h>

namespace ReUseX::visualize {

Visualizer::Visualizer(std::shared_ptr<std::vector<int>> vps) : viewports(vps) {
  std::latch barrier(1);
  viz_thread = std::thread(&Visualizer::mainThread, this, std::ref(barrier));

  spdlog::trace("Waiting for visualization thread to initialize");
  // Wait for visualization thread to finish initializing viewports
  barrier.wait(); // blocks until count_down calleD
}

Visualizer::~Visualizer() {
  if (viz_thread.joinable())
    viz_thread.join();
}

void Visualizer::mainThread(std::latch &barrier) {
  spdlog::trace("Visualization thread started");
  VisualizerPtr viewer =
      std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");

  initViewports(viewer);
  initCameraParameters(viewer);

  // Notify the contructor that viewports are ready
  barrier.count_down();

  // Main loop
  // Spin until the viewer is closed
  while (!viewer->wasStopped()) {
    addPendingTasks(viewer);
    viewer->spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}
void Visualizer::initViewports(VisualizerPtr viewer) {

  if (!viewports) {
    viewer->addCoordinateSystem(1.0);
    return;
  }

  size_t n_vps = viewports->size();

  size_t n_cols = static_cast<size_t>(std::ceil(std::sqrt(n_vps)));
  size_t n_rows = static_cast<size_t>(
      std::ceil(static_cast<double>(n_vps) / static_cast<double>(n_cols)));

  for (size_t i = 0; i < n_vps; ++i) {
    int col = i % n_cols;
    int row = i / n_cols;

    double x_min = static_cast<double>(col) / static_cast<double>(n_cols);
    double x_max = static_cast<double>(col + 1) / static_cast<double>(n_cols);
    double y_min =
        1.0 - static_cast<double>(row + 1) / static_cast<double>(n_rows);
    double y_max = 1.0 - static_cast<double>(row) / static_cast<double>(n_rows);

    viewer->createViewPort(x_min, y_min, x_max, y_max, (*viewports)[i]);
    viewer->addCoordinateSystem(
        1.0, fmt::format("reference_vp{}", i + 1).c_str(), (*viewports)[i]);
  }
}

void Visualizer::initCameraParameters(VisualizerPtr viewer) {
  viewer->setBackgroundColor(0, 0, 0, 1);
  viewer->initCameraParameters();
}

void Visualizer::addPendingTasks(VisualizerPtr viewer) {
  std::lock_guard<std::mutex> lock(queue_mutex);
  while (!task_queue.empty()) {
    spdlog::trace("adding pending visualization task");
    task_queue.front()(viewer); // call function
    task_queue.pop();
  }
}

void Visualizer::enqueueTask(std::function<void(VisualizerPtr)> task) {
  std::lock_guard<std::mutex> lock(queue_mutex);
  task_queue.push(std::move(task));
}

void Visualizer::resetCameraViewpoint(const std::string name) {
  enqueueTask([=](VisualizerPtr viewer) {
    spdlog::trace("Resetting camera viewpoint for: '{}'", name);
    viewer->resetCameraViewpoint(name);
  });
}

void Visualizer::addPointCloud(CloudConstPtr cloud, std::string name, int vp) {
  spdlog::trace("Displaying point cloud: {} in vp {} with {} points", name, vp,
                cloud->size());
  enqueueTask([=](VisualizerPtr viewer) {
    spdlog::trace("Displaying point cloud: {} in vp {} with points", name, vp);
    viewer->addPointCloud<PointT>(cloud, name, vp);
  });
}

void Visualizer::addPlane(const Eigen::Vector4d &plane,
                          const Eigen::Vector3d &origin, const pcl::RGB &color,
                          const std::string_view &name, int vp) {
  enqueueTask([=](VisualizerPtr viewer) {
    spdlog::trace("Displaying plane");
    ReUseX::visualize::addPlane(viewer, plane, origin, color, name, vp);
  });
}

void Visualizer::addPlanes(const std::vector<Pair> &vertical_planes,
                           const std::string_view &name, int vp) {
  spdlog::trace("Displaying planes");
  enqueueTask([=](VisualizerPtr viewer) {
    ReUseX::visualize::addPlanes(viewer, vertical_planes, name, vp);
  });
}

void Visualizer::addPair(const PlanePair &pair, const std::string_view &name,
                         int vp) {
  enqueueTask([=](VisualizerPtr viewer) {
    spdlog::trace("Displaying plane pair");
    ReUseX::visualize::addPair(viewer, pair, name, vp);
  });
}

void Visualizer::addPlanePairs(const std::vector<PlanePair> &plane_pairs,
                               const std::string_view &name, int vp) {
  enqueueTask([=](VisualizerPtr viewer) {
    spdlog::trace("Displaying plane pairs");
    ReUseX::visualize::addPlanePairs(viewer, plane_pairs, name, vp);
  });
}

void Visualizer::addFloor(const double height, const Eigen::Vector3d &min,
                          const Eigen::Vector3d &max,
                          const std::string_view &name, int vp) {
  enqueueTask([=](VisualizerPtr viewer) {
    spdlog::trace("Displaying floor at height: {}", height);
    ReUseX::visualize::addFloor(viewer, height, min, max, name, vp);
  });
}

void Visualizer::addFloors(const std::vector<double> &heights,
                           const Eigen::Vector3d &min,
                           const Eigen::Vector3d &max,
                           const std::string_view &name, int vp) {
  enqueueTask([=](VisualizerPtr viewer) {
    spdlog::trace("Displaying floors");
    ReUseX::visualize::addFloors(viewer, heights, min, max, name, vp);
  });
}

void Visualizer::addCellComplex(
    const std::shared_ptr<ReUseX::geometry::CellComplex> &cc,
    const std::string_view &name, int vp) {
  enqueueTask([=](VisualizerPtr viewer) {
    spdlog::trace("Displaying cell complex vertices");
    ReUseX::visualize::addCellComplex(viewer, cc, name, vp);
  });
}

void Visualizer::addRoomProbabilities(
    const std::shared_ptr<ReUseX::geometry::CellComplex> &cc,
    const std::string_view &name, int vp) {
  enqueueTask([=](VisualizerPtr viewer) {
    spdlog::trace("Displaying room probabilities");
    ReUseX::visualize::addRoomProbabilities(viewer, cc, name, vp);
  });
}

void Visualizer::addSupportProbabilities(
    const std::shared_ptr<ReUseX::geometry::CellComplex> &cc,
    const std::string_view &name, int vp) {
  enqueueTask([=](VisualizerPtr viewer) {
    spdlog::trace("Displaying face support probabilities");
    ReUseX::visualize::addSupportProbabilities(viewer, cc, name, vp);
  });
}

void Visualizer::addRooms(
    const std::shared_ptr<ReUseX::geometry::CellComplex> &cc,
    const std::pair<
        std::unordered_map<ReUseX::geometry::CellComplex::Vertex, int>,
        std::unordered_map<ReUseX::geometry::CellComplex::Vertex,
                           std::set<int>>> &results,
    const std::string_view &name, int vp) {
  enqueueTask([=](VisualizerPtr viewer) {
    spdlog::trace("Displaying results");
    ReUseX::visualize::addRooms(viewer, cc, results, name, vp);
  });
}
} // namespace ReUseX::visualize
