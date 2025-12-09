// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <ReUseX/geometry/CellComplex.hpp>
#include <ReUseX/types.hpp>

#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace ReUseX::visualize {

class Visualizer {
public:
  Visualizer(std::shared_ptr<std::vector<int>> vps = nullptr);
  ~Visualizer();

  // Delete copy constructor and copy assignment operator
  Visualizer(const Visualizer &) = delete;
  Visualizer &operator=(const Visualizer &) = delete;

  // Delete move constructor and move assignment operator
  Visualizer(Visualizer &&) = delete;
  Visualizer &operator=(Visualizer &&) = delete;

public:
  std::shared_ptr<const std::vector<int>> getViewports() const;
  void reset_camera_viewpoint(const std::string &name = "");

  void add_point_cloud(CloudConstPtr cloud, std::string name = "cloud",
                       int vp = 0);

  void add_plane(const Eigen::Vector4d &plane, const Eigen::Vector3d &origin,
                 const pcl::RGB &color, const std::string_view &name = "plane",
                 int vp = 0);

  void add_planes(const std::vector<Pair> &planes,
                  const std::string_view &name = "plane", int vp = 0);

  void add_pair(const PlanePair &plane_pair,
                const std::string_view &name = "plane_pair", int vp = 0);

  void add_plane_pairs(const std::vector<PlanePair> &plane_pairs,
                       const std::string_view &name = "plane_pair", int vp = 0);

  void add_floor(const double height, const Eigen::Vector3d &min,
                 const Eigen::Vector3d &max,
                 const std::string_view &name = "floor", int vp = 0);

  void add_floors(const std::vector<double> &heights, const Eigen::Vector3d &min,
                  const Eigen::Vector3d &max,
                  const std::string_view &name = "floor", int vp = 0);

  void add_cell_complex(
      const std::shared_ptr<ReUseX::geometry::CellComplex> &cc,
      const std::string_view &name = "cell_complex", int vp = 0);

  void add_room_probabilities(

      const std::shared_ptr<ReUseX::geometry::CellComplex> &cc,
      const std::string_view &name = "room_probabilities", int vp = 0);

  void add_support_probabilities(

      const std::shared_ptr<ReUseX::geometry::CellComplex> &cc,
      const std::string_view &name = "support_probabilities", int vp = 0);

  void add_rooms(
      const std::shared_ptr<ReUseX::geometry::CellComplex> &cc,
      const std::pair<
          std::unordered_map<ReUseX::geometry::CellComplex::Vertex, int>,
          std::unordered_map<ReUseX::geometry::CellComplex::Vertex,
                             std::set<int>>> &results,
      const std::string_view &name = "rooms", int vp = 0);

  // Deprecated aliases for backward compatibility
  [[deprecated("Use reset_camera_viewpoint instead")]] void
  resetCameraViewpoint(const std::string &name = "") {
    reset_camera_viewpoint(name);
  }
  [[deprecated("Use add_point_cloud instead")]] void
  addPointCloud(CloudConstPtr cloud, std::string name = "cloud", int vp = 0) {
    add_point_cloud(cloud, name, vp);
  }
  [[deprecated("Use add_plane instead")]] void
  addPlane(const Eigen::Vector4d &plane, const Eigen::Vector3d &origin,
           const pcl::RGB &color, const std::string_view &name = "plane",
           int vp = 0) {
    add_plane(plane, origin, color, name, vp);
  }
  [[deprecated("Use add_planes instead")]] void
  addPlanes(const std::vector<Pair> &planes,
            const std::string_view &name = "plane", int vp = 0) {
    add_planes(planes, name, vp);
  }
  [[deprecated("Use add_pair instead")]] void
  addPair(const PlanePair &plane_pair,
          const std::string_view &name = "plane_pair", int vp = 0) {
    add_pair(plane_pair, name, vp);
  }
  [[deprecated("Use add_plane_pairs instead")]] void
  addPlanePairs(const std::vector<PlanePair> &plane_pairs,
                const std::string_view &name = "plane_pair", int vp = 0) {
    add_plane_pairs(plane_pairs, name, vp);
  }
  [[deprecated("Use add_floor instead")]] void
  addFloor(const double height, const Eigen::Vector3d &min,
           const Eigen::Vector3d &max, const std::string_view &name = "floor",
           int vp = 0) {
    add_floor(height, min, max, name, vp);
  }
  [[deprecated("Use add_floors instead")]] void
  addFloors(const std::vector<double> &heights, const Eigen::Vector3d &min,
            const Eigen::Vector3d &max, const std::string_view &name = "floor",
            int vp = 0) {
    add_floors(heights, min, max, name, vp);
  }
  [[deprecated("Use add_cell_complex instead")]] void
  addCellComplex(const std::shared_ptr<ReUseX::geometry::CellComplex> &cc,
                 const std::string_view &name = "cell_complex", int vp = 0) {
    add_cell_complex(cc, name, vp);
  }
  [[deprecated("Use add_room_probabilities instead")]] void
  addRoomProbabilities(
      const std::shared_ptr<ReUseX::geometry::CellComplex> &cc,
      const std::string_view &name = "room_probabilities", int vp = 0) {
    add_room_probabilities(cc, name, vp);
  }
  [[deprecated("Use add_support_probabilities instead")]] void
  addSupportProbabilities(
      const std::shared_ptr<ReUseX::geometry::CellComplex> &cc,
      const std::string_view &name = "support_probabilities", int vp = 0) {
    add_support_probabilities(cc, name, vp);
  }
  [[deprecated("Use add_rooms instead")]] void
  addRooms(const std::shared_ptr<ReUseX::geometry::CellComplex> &cc,
           const std::pair<
               std::unordered_map<ReUseX::geometry::CellComplex::Vertex, int>,
               std::unordered_map<ReUseX::geometry::CellComplex::Vertex,
                                  std::set<int>>> &results,
           const std::string_view &name = "rooms", int vp = 0) {
    add_rooms(cc, results, name, vp);
  }

private:
  class Impl;
  std::unique_ptr<Impl> pimpl_;
};
} // namespace ReUseX::visualize
