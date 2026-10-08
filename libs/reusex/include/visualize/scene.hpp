// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The scene builder shared by `rux render` (render_view, headless) and the
// Qt client's 3D workspace (an interactive QVTKOpenGLNativeWidget).
//
// render_view() used to build its VTK scene inline, so the only way for an
// interactive viewer to show "the same thing rux render shows" was to copy
// that code. populate_scene() is the one definition instead: it loads a
// project's layers and adds them to a caller's vtkRenderer, and the camera and
// cut-plane helpers below place the camera the way the presets do. Both front
// ends call them; render_view() is now validation + populate_scene() + camera
// + an offscreen render window.
//
// Header hygiene (STANDARDS §2): VTK types are forward-declared, so a consumer
// that only needs SceneOptions/SceneInfo compiles no VTK header.

#include "reusex/visualize/render_view.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

class vtkActor;
class vtkRenderer;

namespace reusex::visualize {

/// Axis-aligned bounds of the geometry a scene draws.
struct SceneBounds {
  std::array<double, 3> min{0, 0, 0};
  std::array<double, 3> max{0, 0, 0};
  bool valid = false;

  void add(double x, double y, double z);
  std::array<double, 3> center() const;
  double extent(int axis) const { return max[axis] - min[axis]; }
  double diagonal() const;
};

/// What populate_scene() draws, and how.
struct SceneOptions {
  /// Layers in draw order. Empty is an error.
  std::vector<Layer> layers{Layer::cloud};
  /// Named geometry cloud supplying point positions for every point layer.
  std::string cloud_name = "cloud";
  /// Named mesh drawn by Layer::mesh.
  std::string mesh_name = "mesh";
  /// Point sprite size in pixels. Must be positive.
  double point_size = 2.0;
  /// Paint one instance and dim the rest (see render_view.hpp).
  std::optional<InstanceHighlight> highlight;
};

/// The actors one layer added, so an interactive viewer can toggle it.
struct SceneLayer {
  Layer layer = Layer::cloud;
  /// Every actor of the layer (one for a point layer, one per component).
  std::vector<vtkActor *> actors;
  std::size_t points = 0; ///< points drawn
  std::size_t faces = 0;  ///< mesh faces drawn
};

/// What populate_scene() added.
struct SceneInfo {
  SceneBounds bounds;
  std::size_t drawn_points = 0;
  std::size_t drawn_faces = 0;
  std::vector<SceneLayer> layers;
};

/// Load @p opts' layers from @p db and add them to @p renderer.
///
/// Does not touch the camera, the background or any render window.
/// @throws std::runtime_error when a requested layer's data is missing or out
///         of sync (the message names the stage to run), or when the layers
///         produced no drawable geometry.
SceneInfo populate_scene(vtkRenderer *renderer, const ProjectDB &db,
                         const SceneOptions &opts);

/// A resolved horizontal cut: where the plane sits and how that was decided.
struct CutPlane {
  double z = 0.0;           ///< world z of the plane
  double floor_z = 0.0;     ///< the reference it was measured up from
  double height = 0.0;      ///< metres above @ref floor_z
  bool from_planes = false; ///< floor came from the segmentation, not the bbox
};

/// Decide where to cut @p bounds: @p height (or the default) above the floor
/// detected by `rux create planes`, or above the bounding box's lower face.
/// Warns when the cut is above or below everything drawn (STANDARDS §5).
CutPlane resolve_cut_plane(const ProjectDB &db, const SceneBounds &bounds,
                           std::optional<double> height);

/// Clip every actor of @p renderer to the half-space below z = @p z.
void apply_cut_plane(vtkRenderer *renderer, double z);

/// Remove every clipping plane from @p renderer's actors.
void clear_cut_planes(vtkRenderer *renderer);

/// How a preset camera frames the scene.
struct CameraFraming {
  ViewPreset view = ViewPreset::top; ///< never ViewPreset::explicit_camera
  int orbit_count = 8;
  int orbit_index = 0;
  double orbit_elevation_deg = 25.0;
  double margin = 1.08;
  double aspect = 4.0 / 3.0; ///< viewport width / height
};

/// Place @p renderer's active camera for a preset view of @p bounds.
/// Deterministic: a pure function of the bounds (STANDARDS §6).
void place_preset_camera(vtkRenderer *renderer, const CameraFraming &framing,
                         const SceneBounds &bounds);

/// Place @p renderer's active camera at an explicit pose + pinhole camera,
/// for an image of @p width x @p height pixels.
/// @throws std::runtime_error on non-positive focal lengths.
void place_explicit_camera(vtkRenderer *renderer, const CameraSpec &spec,
                           int width, int height, const SceneBounds &bounds);

/// Route VTK's diagnostics into ReUseX logging, once per process (see
/// render_view()).
void install_vtk_log_bridge();

} // namespace reusex::visualize
