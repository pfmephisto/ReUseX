// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Injectable view-renderer interface for evidence renders (Kortlægning,
// #265 Phase 2 Task 8).
//
// LAYERING: ruxd_api_lib must not link reusex_visualize (VTK); ruxd's main
// layer registers a concrete renderer; without one the endpoint answers 503.

#pragma once

#include <reusex/core/ProjectDB.hpp>

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace ruxd::api {

/// Parsed and validated GET /api/v1/renders query.
struct RenderRequest {
  std::string view = "plan"; // plan | top | front | orbit
  int orbit_index = 0;       // with view == orbit, of 8
  std::vector<std::string> layers{"cloud"};
  std::optional<std::string> highlight_cloud;
  std::optional<std::uint32_t> highlight_instance;
  int width = 960;
  int height = 720;
};

/// This server has no view renderer (built without the visualize module, or
/// the machine has no usable offscreen OpenGL context).
class RenderUnavailable : public std::runtime_error {
    public:
  using std::runtime_error::runtime_error;
};

/// View-renderer hook injected into the GUI server by ruxd's main.
class IViewRenderer {
    public:
  virtual ~IViewRenderer() = default;

  /// Render one view of @p db as PNG bytes.
  ///
  /// @throws RenderUnavailable    no usable offscreen OpenGL context.
  /// @throws std::invalid_argument the request itself is bad.
  /// @throws std::runtime_error   the project lacks the data; the message
  ///         names the stage to run first.
  ///
  /// Implementations must serialise internally: render_view() drives VTK,
  /// which is not safe to run concurrently in one process.
  virtual std::vector<std::uint8_t> render_png(const reusex::ProjectDB &db,
                                               const RenderRequest &req) = 0;
};

} // namespace ruxd::api
