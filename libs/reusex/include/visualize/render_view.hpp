// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Headless render-to-image (#294).
//
// Renders the contents of a .rux project to an in-memory image without a
// display server. The backend is VTK's offscreen render window: the flake's
// VTK is built with VTK_OPENGL_HAS_EGL, and VTK transparently falls back from
// vtkXOpenGLRenderWindow to vtkEGLRenderWindow when no X/Wayland session is
// reachable, so `env -u DISPLAY rux render ...` renders on the GPU exactly as
// it would with a session attached.
//
// Keep this header light (STANDARDS §2): cv::Mat and ProjectDB are
// forward-declared, and no VTK header is exposed to consumers.

#include <array>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace cv {
class Mat;
} // namespace cv

namespace reusex {

class ProjectDB;

namespace visualize {

/// One drawable body of content in a rendered view.
///
/// `cloud` draws the geometry cloud with its stored RGB colour; the four
/// label layers draw the same geometry recoloured by the corresponding
/// index-aligned `Label` cloud (CONTRACTS.md); `mesh` draws a stored mesh and
/// `components` draws building-component outlines.
enum class Layer {
  cloud,      ///< `cloud` (PointXYZRGB), stored per-point colour
  labels,     ///< `labels` (Label) — semantic classes
  planes,     ///< `planes` (Label) — planar segments
  rooms,      ///< `rooms` (Label) — room partition
  instances,  ///< `instances` (Label) — spatial instances
  mesh,       ///< a stored mesh from the `meshes` table
  components, ///< building-component outlines (windows / doors / walls)
};

/// Parse a layer name as accepted by `rux render --layers`.
/// @return std::nullopt if @p name is not a layer.
std::optional<Layer> layer_from_string(std::string_view name);

/// The canonical name of @p layer (the inverse of layer_from_string).
std::string_view to_string(Layer layer);

/// Every layer, in draw order — the vocabulary `--layers` accepts.
const std::vector<Layer> &all_layers();

/// How the camera is placed.
enum class ViewPreset {
  /// Orthographic floor plan looking straight down (-Z), north up.
  top,
  /// Orthographic elevation looking along +Y.
  front,
  /// Perspective view on a ring around the scene; see
  /// RenderOptions::orbit_index / orbit_count / orbit_elevation_deg.
  orbit,
  /// Use RenderOptions::camera verbatim.
  explicit_camera,
};

/// An explicit camera: rigid pose plus pinhole intrinsics.
///
/// The layout deliberately matches what ProjectDB hands out for a captured
/// frame — `ProjectDB::sensor_frame_pose()` (row-major, camera-to-world) and
/// `SensorIntrinsics` — so rendering the scene from a recorded viewpoint is a
/// copy, not a conversion. Axes follow the OpenCV camera convention used
/// throughout the pipeline: +x right, +y down, +z forward along the view.
struct CameraSpec {
  /// Row-major 4x4 camera-to-world transform.
  std::array<double, 16> pose{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  double fx = 0.0; ///< focal length in pixels (x)
  double fy = 0.0; ///< focal length in pixels (y)
  double cx = 0.0; ///< principal point in pixels (x)
  double cy = 0.0; ///< principal point in pixels (y)
};

/// Everything render_view() can be told to do.
///
/// This struct is the single definition of every render default; `rux render`
/// mirrors these values into its `--help` and never redefines them
/// (STANDARDS §4).
struct RenderOptions {
  /// Layers to draw, back to front. Empty is an error.
  std::vector<Layer> layers{Layer::cloud};

  /// Named geometry cloud supplying point positions for every point layer.
  std::string cloud_name = "cloud";
  /// Named mesh drawn by Layer::mesh.
  std::string mesh_name = "mesh";

  ViewPreset view = ViewPreset::top;

  /// Number of viewpoints on the orbit ring (`--view orbit:N`).
  int orbit_count = 8;
  /// Which of the @ref orbit_count viewpoints to render, in [0, orbit_count).
  int orbit_index = 0;
  /// Height of the orbit ring above the scene centre, in degrees.
  double orbit_elevation_deg = 25.0;

  /// Camera used when view == ViewPreset::explicit_camera.
  std::optional<CameraSpec> camera;

  int width = 1600;
  int height = 1200;

  /// Background colour as linear RGB in [0, 1].
  std::array<double, 3> background{0.11, 0.12, 0.14};

  /// Point sprite size in pixels.
  double point_size = 2.0;

  /// Framing slack around the scene bounding box (1.0 = tight fit).
  double margin = 1.08;

  /// Draw a subtle ground grid and axes for scale reference.
  bool show_axes = false;
};

/// Render one view of @p db to an image.
///
/// Deterministic: every preset camera is derived from the bounding box of the
/// geometry actually drawn, so two runs over the same project frame the same
/// shot (STANDARDS §6).
///
/// @param db    An open project. Read-only access is sufficient.
/// @param opts  What and how to draw.
/// @return An 8-bit 3-channel BGR image of size opts.width x opts.height,
///         ready for `cv::imwrite`.
/// @throws std::runtime_error if the project is not open, the options are
///         invalid, a requested layer's data is missing (the message names the
///         stage to run first), or the GPU/EGL context cannot be created.
cv::Mat render_view(ProjectDB &db, const RenderOptions &opts);

} // namespace visualize
} // namespace reusex
