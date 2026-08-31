// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// PCL/Eigen-typed visualization observer payloads. This header carries the
// heavy point-cloud/geometry types that the visualization side needs, and is
// deliberately kept OUT of the core interface (core/processing_observer.hpp):
// core headers stay PCL-free (STANDARDS §1) and only see IVisualObserver as an
// opaque forward-declared pointer through the observer registry. Only
// geometry/vision/visualize consumers that actually emit visualization events
// include this header.

#include "reusex/core/logging.hpp"
#include "reusex/core/processing_observer.hpp"
#include "reusex/core/stages.hpp"
#include "reusex/types.hpp"

#include <pcl/Vertices.h>
#include <pcl/correspondence.h>

#include <Eigen/Geometry>
#include <fmt/format.h>
#include <memory>
#include <string_view>
#include <type_traits>
#include <typeinfo>
#include <utility>
#include <vector>

// Forward declaration to avoid circular dependency
namespace reusex::geometry {
class CellComplex;
} // namespace reusex::geometry

namespace reusex::core {

class IVisualObserver : IObserver {

    public:
  virtual ~IVisualObserver() = default;
  using Pair = std::pair<Eigen::Vector4d, Eigen::Vector3d>;
  using PlanePair = std::pair<Pair, Pair>;

  // Viewer callbacks
  template <typename T>
  void viewer_add_geometry(std::string_view name,
                           [[maybe_unused]] const T &geometry, Stage stage,
                           int idx = 0) {
    // Special case: if T is Eigen::Vector4d, use virtual dispatch
    if constexpr (std::is_same_v<T, Eigen::Vector4d>) {
      viewer_add_plane(name, geometry, stage, idx);
    } else if constexpr (std::is_same_v<
                             T, std::pair<Eigen::Vector4d, Eigen::Vector3d>>) {
      viewer_add_plane(name, geometry, stage, idx);
    } else if constexpr (std::is_same_v<T, PlanePair>) {
      viewer_add_plane_pair(name, geometry, stage, idx);
    } else if constexpr (std::is_same_v<
                             T, std::shared_ptr<geometry::CellComplex>>) {
      viewer_add_cell_complex(name, geometry, stage, idx);
    } else if constexpr (std::is_same_v<T, CloudConstPtr>) {
      viewer_add_cloud(name, geometry, stage, idx);
    } else {
      // Default implementation: log that geometry type is not handled
      // This prevents linker errors while providing runtime visibility
      core::debug("viewer_add_geometry<{}> called for '{}' at stage '{}' "
                  "(no handler registered)",
                  typeid(T).name(), name, to_string(stage));
    }
  }

  template <typename T>
  void viewer_add_geometries(std::string_view name, const T &geometries,
                             Stage stage) {
    for (size_t i = 0; i < geometries.size(); ++i) {
      viewer_add_geometry(fmt::format("{}_{}", name, i), geometries[i], stage,
                          i);
    }
  }

  // Virtual method for specific geometry types (can be overridden)
  virtual void viewer_add_plane(std::string_view name,
                                [[maybe_unused]] const Eigen::Vector4d &plane,
                                Stage stage, int /*idx*/ = 0) {
    // Default: log that geometry type is not handled
    core::debug("viewer_add_plane called for '{}' at stage '{}' "
                "(no handler registered)",
                name, to_string(stage));
  }

  virtual void viewer_add_plane(
      std::string_view name,
      [[maybe_unused]] const std::pair<Eigen::Vector4d, Eigen::Vector3d> &plane,
      Stage stage, int /*idx*/ = 0) {
    // Default: log that geometry type is not handled
    core::debug("viewer_add_plane (with origin) called for '{}' at stage '{}' "
                "(no handler registered)",
                name, to_string(stage));
  }

  virtual void viewer_add_plane_pair(std::string_view name,
                                     [[maybe_unused]] const PlanePair &pair,
                                     Stage stage, int /*idx*/ = 0) {
    // Default: log that geometry type is not handled
    core::debug("viewer_add_plane_pair called for '{}' at stage '{}' "
                "(no handler registered)",
                name, to_string(stage));
  }

  virtual void viewer_add_cell_complex(
      std::string_view name,
      [[maybe_unused]] const std::shared_ptr<reusex::geometry::CellComplex> &cc,
      Stage stage, int /*idx*/ = 0) {
    // Default: log that geometry type is not handled
    core::debug("viewer_add_cell_complex called for '{}' at stage '{}' "
                "(no handler registered)",
                name, to_string(stage));
  }

  virtual void viewer_add_cloud(std::string_view name,
                                [[maybe_unused]] const CloudConstPtr &cloud,
                                Stage stage, int /*idx*/ = 0) {
    // Default: log that geometry type is not handled
    core::debug("viewer_add_cloud called for '{}' at stage '{}' "
                "(no handler registered)",
                name, to_string(stage));
  }

  virtual void viewer_add_camera_frustum(
      std::string_view name, [[maybe_unused]] double focal_x,
      [[maybe_unused]] double focal_y, [[maybe_unused]] int image_width,
      [[maybe_unused]] int image_height,
      [[maybe_unused]] const Eigen::Affine3f &pose, Stage stage,
      int /*idx*/ = 0) {
    core::debug("viewer_add_camera_frustum called for '{}' at stage '{}' "
                "(no handler registered)",
                name, to_string(stage));
  }

  virtual void viewer_add_visibility_graph(
      std::string_view name, [[maybe_unused]] const CloudLocPtr &disc_points,
      [[maybe_unused]] const std::shared_ptr<std::vector<pcl::Vertices>>
          &disc_outlines,
      [[maybe_unused]] const pcl::CorrespondencesPtr &edges, Stage stage,
      int /*idx*/ = 0) {
    core::debug("viewer_add_visibility_graph called for '{}' at stage '{}' "
                "(no handler registered)",
                name, to_string(stage));
  }

  virtual void
  viewer_add_labeled_cloud(std::string_view name,
                           [[maybe_unused]] const CloudConstPtr &cloud,
                           [[maybe_unused]] const CloudLConstPtr &labels,
                           Stage stage, int /*idx*/ = 0) {
    core::debug("viewer_add_labeled_cloud called for '{}' at stage '{}' "
                "(no handler registered)",
                name, to_string(stage));
  }
};

} // namespace reusex::core
