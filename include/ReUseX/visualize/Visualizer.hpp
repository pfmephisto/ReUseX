// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <ReUseX/geometry/CellComplex.hpp>
#include <ReUseX/types.hpp>

#include <pcl/visualization/pcl_visualizer.h>

#include <chrono>
#include <functional>
#include <latch>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

namespace ReUseX::visualize {

class Visualizer
/*: pcl::visualization::PCLVisualizer */ {
    private:
  using VisualizerPtr = std::shared_ptr<pcl::visualization::PCLVisualizer>;
  using VizTask = std::function<void(VisualizerPtr)>;

    public:
  Visualizer(std::shared_ptr<std::vector<int>> vps = nullptr);
  ~Visualizer();

  // delete copy constructor and copy assignment operator
  Visualizer(const Visualizer &) = delete;
  Visualizer &operator=(const Visualizer &) = delete;

  // delete move constructor and move assignment operator
  Visualizer(Visualizer &&) = delete;
  Visualizer &operator=(Visualizer &&) = delete;

    public:
  std::shared_ptr<const std::vector<int>> getViewports() const {
    return viewports;
  }
  void resetCameraViewpoint(const std::string name = "");

  void addPointCloud(CloudConstPtr cloud, std::string name = "cloud",
                     int vp = 0);

  void addPlane(const Eigen::Vector4d &plane, const Eigen::Vector3d &origin,
                const pcl::RGB &color, const std::string_view &name = "plane",
                int vp = 0);

  void addPlanes(const std::vector<Pair> &planes,
                 const std::string_view &name = "plane", int vp = 0);

  void addPair(const PlanePair &plane_pair,
               const std::string_view &name = "plane_pair", int vp = 0);

  void addPlanePairs(const std::vector<PlanePair> &plane_pairs,
                     const std::string_view &name = "plane_pair", int vp = 0);

  void addFloor(const double height, const Eigen::Vector3d &min,
                const Eigen::Vector3d &max,
                const std::string_view &name = "floor", int vp = 0);

  void addFloors(const std::vector<double> &heights, const Eigen::Vector3d &min,
                 const Eigen::Vector3d &max,
                 const std::string_view &name = "floor", int vp = 0);

  void addCellComplex(const std::shared_ptr<ReUseX::geometry::CellComplex> &cc,
                      const std::string_view &name = "cell_complex",
                      int vp = 0);

  void addRoomProbabilities(

      const std::shared_ptr<ReUseX::geometry::CellComplex> &cc,
      const std::string_view &name = "room_probabilities", int vp = 0);

  void addSupportProbabilities(

      const std::shared_ptr<ReUseX::geometry::CellComplex> &cc,
      const std::string_view &name = "support_probabilities", int vp = 0);

  void
  addRooms(const std::shared_ptr<ReUseX::geometry::CellComplex> &cc,
           const std::pair<
               std::unordered_map<ReUseX::geometry::CellComplex::Vertex, int>,
               std::unordered_map<ReUseX::geometry::CellComplex::Vertex,
                                  std::set<int>>> &results,
           const std::string_view &name = "rooms", int vp = 0);

    private:
  // methods
  void mainThread(std::latch &barrier);
  void initViewports(VisualizerPtr viewer);
  void initCameraParameters(VisualizerPtr viewer);
  void addPendingTasks(VisualizerPtr viewer);
  void enqueueTask(std::function<void(VisualizerPtr)> task);

    private:
  std::thread viz_thread;
  std::shared_ptr<std::vector<int>> viewports;

  std::mutex queue_mutex;
  std::queue<VizTask> task_queue;
};
} // namespace ReUseX::visualize
