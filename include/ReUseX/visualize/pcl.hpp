// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "ReUseX/geometry/CellComplex.hpp"
#include "ReUseX/types.hpp"

#include <Eigen/Dense>
#include <pcl/common/colors.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>

namespace ReUseX::visualize {

void addPlane(std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
              const Eigen::Vector4d &plane, const Eigen::Vector3d &origin,
              const pcl::RGB &color, const std::string_view &name = "plane",
              int vp = 0);

void addPlanes(std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
               const std::vector<Pair> &planes,
               const std::string_view &name = "plane", int vp = 0);

void addPair(std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
             const PlanePair &plane_pair,
             const std::string_view &name = "plane_pair", int vp = 0);

void addPlanePairs(std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                   const std::vector<PlanePair> &plane_pairs,
                   const std::string_view &name = "plane_pair", int vp = 0);

void addFloor(std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
              const double height, const Eigen::Vector3d &min,
              const Eigen::Vector3d &max,
              const std::string_view &name = "floor", int vp = 0);

void addFloors(std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
               const std::vector<double> &heights, const Eigen::Vector3d &min,
               const Eigen::Vector3d &max,
               const std::string_view &name = "floor", int vp = 0);

void addCellComplex(std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                    const std::shared_ptr<ReUseX::geometry::CellComplex> &cc,
                    const std::string_view &name = "cell_complex", int vp = 0);

void addRoomProbabilities(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const std::shared_ptr<ReUseX::geometry::CellComplex> &cc,
    const std::string_view &name = "room_probabilities", int vp = 0);

void addSupportProbabilities(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const std::shared_ptr<ReUseX::geometry::CellComplex> &cc,
    const std::string_view &name = "support_probabilities", int vp = 0);

void addRooms(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const std::shared_ptr<ReUseX::geometry::CellComplex> &cc,
    const std::pair<
        std::unordered_map<ReUseX::geometry::CellComplex::Vertex, int>,
        std::unordered_map<ReUseX::geometry::CellComplex::Vertex,
                           std::set<int>>> &results,
    const std::string_view &name = "rooms", int vp = 0);

void addCameraFrustum(std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                      pcl::TextureMapping<pcl::PointXYZ>::Camera &cam,
                      const std::string_view &name = "camera", int vp = 0);

/** \brief Display a 3D representation showing the a cloud and a list of camera
 * with their 6DOf poses */
void addCameraFrustums(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    pcl::texture_mapping::CameraVector cams,
    const std::string_view &name = "camera", int vp = 0);

} // namespace ReUseX::visualize
