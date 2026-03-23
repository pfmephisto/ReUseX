// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/core/logging.hpp"
#include "reusex/types.hpp"

#include <fmt/format.h>
#include <string_view>
#include <type_traits>
#include <typeinfo>

// Forward declaration to avoid circular dependency
namespace ReUseX::geometry {
class CellComplex;
} // namespace ReUseX::geometry

namespace ReUseX::core {

enum class EventType {
  Process,
  Progress,
  Visualization
  //
};

class IObserver {};
class IVisualObserver : IObserver {};
class IProgressObserver_new : IObserver {};

// TODO: Refactor to separate visualization and progress observers
class [[deprecated("Migrate to using IVisulaObserver and "
                   "IProgressObserver_new")]] IProcessingObserver {
    public:
  virtual ~IProcessingObserver() = default;

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
      [[maybe_unused]] const std::shared_ptr<ReUseX::geometry::CellComplex> &cc,
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

  // Progress bar callbacks
  virtual void on_process_started(Stage, size_t) {}
  virtual void on_process_finished(Stage) {}
  virtual void on_process_updated(Stage, size_t) {}
};

class NullProcessingObserver final : public IProcessingObserver {};

class ProgressObserver {
    public:
  ProgressObserver(Stage stage, size_t total = 0);
  ~ProgressObserver();

  void update(size_t progress = 1);

  inline void operator++() { update(1); };
  inline void operator+=(size_t increment) { update(increment); };

    private:
  Stage stage_;
  size_t total_ = 0;
};

// Register a global processing observer. The caller retains ownership and must
// keep the observer alive until reset or replacement. Passing nullptr clears
// it.

void set_processing_observer(IProcessingObserver *observer);
void reset_processing_observer();
auto get_processing_observer() -> IProcessingObserver *;

} // namespace ReUseX::core
