// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "visualize/render_view.hpp"

#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"
#include "geometry/BuildingComponent.hpp"
#include "geometry/component_persistence.hpp"
#include "geometry/transform_utils.hpp"
#include "types/point_types.hpp"
#include "visualize/offscreen_gl.hpp"

#include <opencv2/core.hpp>

#include <pcl/PolygonMesh.h>
#include <pcl/common/colors.h>
#include <pcl/conversions.h>

#include <vtkActor.h>
#include <vtkActorCollection.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkImageData.h>
#include <vtkLogger.h>
#include <vtkMapper.h>
#include <vtkNew.h>
#include <vtkPlane.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkUnsignedCharArray.h>
#include <vtkWindowToImageFilter.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numbers>
#include <stdexcept>
#include <string>

namespace reusex::visualize {

namespace {

// ── VTK diagnostics -> ReUseX logging ────────────────────────────────────────
//
// Two reasons to reroute VTK's own output: library code must log through
// core/logging (CLAUDE.md), and the headless path always emits one cosmetic
// warning — vtkXOpenGLRenderWindow complaining that it cannot reach an X
// server — immediately before VTK falls back to the EGL render window and
// succeeds. That message is expected on every display-less run, so it is
// demoted to debug while every other VTK warning still surfaces at warn.
//
// The interception point is vtkLogger rather than vtkOutputWindow. In VTK 9
// the vtkWarningMacro family routes through vtkOutputWindow's file/line-aware
// virtuals into vtkLogger, and some call sites reach vtkLogger directly;
// subclassing vtkOutputWindow catches only part of that traffic (verified: the
// X-connection warning still reached stderr). A logger callback catches all of
// it, and switching the stderr sink off routes VTK through our handler instead
// of duplicating every message.

/// The X-server probe that precedes the (successful) EGL fallback.
bool is_expected_headless_fallback(const char *text) {
  return text != nullptr &&
         std::strstr(text, "bad X server connection") != nullptr;
}

void vtk_log_callback(void * /*user_data*/, const vtkLogger::Message &message) {
  const char *text = message.message != nullptr ? message.message : "";
  const char *file = message.filename != nullptr ? message.filename : "?";

  if (message.verbosity <= vtkLogger::VERBOSITY_ERROR) {
    core::error("VTK [{}:{}]: {}", file, message.line, text);
  } else if (message.verbosity == vtkLogger::VERBOSITY_WARNING) {
    if (is_expected_headless_fallback(text)) {
      core::debug("VTK: no X server; falling back to offscreen EGL rendering "
                  "(expected when headless)");
    } else {
      core::warn("VTK [{}:{}]: {}", file, message.line, text);
    }
  } else {
    core::debug("VTK [{}:{}]: {}", file, message.line, text);
  }
}

/// Route VTK's diagnostics into ReUseX logging, once per process.
void install_vtk_log_bridge() {
  static const bool installed = [] {
    vtkLogger::SetStderrVerbosity(vtkLogger::VERBOSITY_OFF);
    vtkLogger::AddCallback("reusex", &vtk_log_callback, nullptr,
                           vtkLogger::VERBOSITY_INFO);
    return true;
  }();
  (void)installed;
}

// ── Bounding box accumulation ────────────────────────────────────────────────

struct Bounds {
  double min[3]{0, 0, 0};
  double max[3]{0, 0, 0};
  bool valid = false;

  void add(double x, double y, double z) {
    if (!valid) {
      min[0] = max[0] = x;
      min[1] = max[1] = y;
      min[2] = max[2] = z;
      valid = true;
      return;
    }
    min[0] = std::min(min[0], x);
    min[1] = std::min(min[1], y);
    min[2] = std::min(min[2], z);
    max[0] = std::max(max[0], x);
    max[1] = std::max(max[1], y);
    max[2] = std::max(max[2], z);
  }

  void center(double c[3]) const {
    for (int i = 0; i < 3; ++i)
      c[i] = 0.5 * (min[i] + max[i]);
  }
  double extent(int axis) const { return max[axis] - min[axis]; }
  double diagonal() const {
    const double dx = extent(0), dy = extent(1), dz = extent(2);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }
};

// ── Colour helpers ───────────────────────────────────────────────────────────

/// Colour for a label, matching what the interactive viewer shows.
///
/// PCL's PointCloudColorHandlerLabelField (used by `rux view`) maps labels
/// through the Glasbey LUT modulo its size, and the legend in
/// apps/rux/src/view/project_loader.cpp does the same — so a render and the
/// viewer agree on colours. Label 0 is "unlabeled" per STANDARDS §3 and is
/// drawn as a muted grey instead of a Glasbey colour, so unsegmented geometry
/// is visible but never mistaken for a class.
void label_color(std::uint32_t label, unsigned char rgb[3]) {
  if (label == 0) {
    rgb[0] = rgb[1] = rgb[2] = 90;
    return;
  }
  const pcl::RGB c = pcl::GlasbeyLUT::at(label % pcl::GlasbeyLUT::size());
  rgb[0] = c.r;
  rgb[1] = c.g;
  rgb[2] = c.b;
}

void component_color(geometry::ComponentType type, double rgb[3]) {
  // Same convention as the interactive viewer (view/component_renderer.cpp).
  switch (type) {
  case geometry::ComponentType::door:
    rgb[0] = 0.0, rgb[1] = 1.0, rgb[2] = 1.0;
    return;
  case geometry::ComponentType::wall:
    rgb[0] = 1.0, rgb[1] = 1.0, rgb[2] = 0.0;
    return;
  case geometry::ComponentType::window:
  default:
    rgb[0] = 0.0, rgb[1] = 1.0, rgb[2] = 0.0;
    return;
  }
}

// ── Data loading ─────────────────────────────────────────────────────────────

/// Name of the pipeline stage that produces a layer's data, for error messages.
std::string producing_command(Layer layer) {
  switch (layer) {
  case Layer::cloud:
    return "rux create clouds";
  case Layer::labels:
    return "rux create annotate && rux create project";
  case Layer::planes:
    return "rux create planes";
  case Layer::rooms:
    return "rux create rooms";
  case Layer::instances:
    return "rux create instances";
  case Layer::mesh:
    return "rux create mesh";
  case Layer::components:
    return "rux create windows";
  }
  return "the corresponding rux create subcommand";
}

/// Load the geometry cloud shared by every point layer, failing loudly.
CloudPtr load_geometry_cloud(const ProjectDB &db, const RenderOptions &opts) {
  if (!db.has_point_cloud(opts.cloud_name)) {
    throw std::runtime_error("render: project has no point cloud named '" +
                             opts.cloud_name + "' — run `" +
                             producing_command(Layer::cloud) + "` first");
  }
  CloudPtr cloud = db.point_cloud_xyzrgb(opts.cloud_name);
  if (!cloud || cloud->empty()) {
    throw std::runtime_error("render: point cloud '" + opts.cloud_name +
                             "' is empty — nothing to draw; re-run `" +
                             producing_command(Layer::cloud) + "`");
  }
  return cloud;
}

/// The label cloud a label layer draws with.
std::string label_cloud_name(Layer layer) {
  switch (layer) {
  case Layer::labels:
    return "labels";
  case Layer::planes:
    return "planes";
  case Layer::rooms:
    return "rooms";
  case Layer::instances:
    return "instances";
  default:
    return {};
  }
}

// ── VTK geometry construction ────────────────────────────────────────────────

/// Wrap positions + per-point colours in a renderable vertex poly-data.
vtkSmartPointer<vtkPolyData>
make_point_polydata(const Cloud &cloud,
                    const std::vector<unsigned char> &colors) {
  const vtkIdType n = static_cast<vtkIdType>(cloud.size());

  vtkNew<vtkPoints> points;
  points->SetDataTypeToFloat();
  points->SetNumberOfPoints(n);

  vtkNew<vtkUnsignedCharArray> scalars;
  scalars->SetNumberOfComponents(3);
  scalars->SetName("colors");
  scalars->SetNumberOfTuples(n);

  vtkNew<vtkCellArray> verts;
  verts->AllocateEstimate(n, 1);

  for (vtkIdType i = 0; i < n; ++i) {
    const auto &p = cloud[static_cast<std::size_t>(i)];
    points->SetPoint(i, p.x, p.y, p.z);
    scalars->SetTypedComponent(i, 0,
                               colors[3 * static_cast<std::size_t>(i) + 0]);
    scalars->SetTypedComponent(i, 1,
                               colors[3 * static_cast<std::size_t>(i) + 1]);
    scalars->SetTypedComponent(i, 2,
                               colors[3 * static_cast<std::size_t>(i) + 2]);
    verts->InsertNextCell(1, &i);
  }

  auto poly = vtkSmartPointer<vtkPolyData>::New();
  poly->SetPoints(points);
  poly->SetVerts(verts);
  poly->GetPointData()->SetScalars(scalars);
  return poly;
}

vtkSmartPointer<vtkActor>
add_points_actor(vtkRenderer *renderer, vtkPolyData *poly, double point_size) {
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(poly);
  mapper->SetScalarModeToUsePointData();
  mapper->SetColorModeToDirectScalars();

  auto actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetPointSize(point_size);
  // Points carry their own colour; lighting would tint them.
  actor->GetProperty()->SetLighting(false);
  renderer->AddActor(actor);
  return actor;
}

/// Convert a stored PolygonMesh into a shaded surface actor.
void add_mesh_actor(vtkRenderer *renderer, const pcl::PolygonMesh &mesh,
                    Bounds &bounds) {
  pcl::PointCloud<pcl::PointXYZ> vertices;
  pcl::fromPCLPointCloud2(mesh.cloud, vertices);

  vtkNew<vtkPoints> points;
  points->SetDataTypeToFloat();
  points->SetNumberOfPoints(static_cast<vtkIdType>(vertices.size()));
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    const auto &v = vertices[i];
    points->SetPoint(static_cast<vtkIdType>(i), v.x, v.y, v.z);
    bounds.add(v.x, v.y, v.z);
  }

  vtkNew<vtkCellArray> polys;
  for (const auto &face : mesh.polygons) {
    if (face.vertices.size() < 3)
      continue;
    std::vector<vtkIdType> ids;
    ids.reserve(face.vertices.size());
    for (const auto idx : face.vertices)
      ids.push_back(static_cast<vtkIdType>(idx));
    polys->InsertNextCell(static_cast<vtkIdType>(ids.size()), ids.data());
  }

  vtkNew<vtkPolyData> poly;
  poly->SetPoints(points);
  poly->SetPolys(polys);

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(poly);

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(0.82, 0.80, 0.76);
  actor->GetProperty()->SetAmbient(0.25);
  actor->GetProperty()->SetDiffuse(0.75);
  actor->GetProperty()->SetSpecular(0.05);
  renderer->AddActor(actor);
}

/// Draw building-component boundaries as closed polylines.
void add_component_actors(
    vtkRenderer *renderer,
    const std::vector<geometry::BuildingComponent> &components,
    Bounds &bounds) {
  for (const auto &comp : components) {
    const auto &verts = comp.boundary.vertices;
    if (verts.size() < 2)
      continue;

    vtkNew<vtkPoints> points;
    points->SetDataTypeToFloat();
    vtkNew<vtkCellArray> lines;
    const vtkIdType n = static_cast<vtkIdType>(verts.size());
    for (vtkIdType i = 0; i < n; ++i) {
      const auto &v = verts[static_cast<std::size_t>(i)];
      points->InsertNextPoint(v.x(), v.y(), v.z());
      bounds.add(v.x(), v.y(), v.z());
    }
    for (vtkIdType i = 0; i < n; ++i) {
      const vtkIdType seg[2] = {i, (i + 1) % n};
      lines->InsertNextCell(2, seg);
    }

    vtkNew<vtkPolyData> poly;
    poly->SetPoints(points);
    poly->SetLines(lines);

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(poly);

    double rgb[3];
    component_color(comp.type, rgb);

    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(rgb[0], rgb[1], rgb[2]);
    actor->GetProperty()->SetLineWidth(3.0);
    actor->GetProperty()->SetLighting(false);
    renderer->AddActor(actor);
  }
}

// ── Horizontal cut plane (#306) ──────────────────────────────────────────────
//
// `--view top` on a real interior renders the ceiling, because that is what is
// topmost. A floor plan is the same camera with everything above waist height
// clipped away, which is what `--view plan` adds.
//
// Two decisions worth stating. The cut is applied per *mapper*, not by
// filtering the geometry, so every layer is cut by the same plane with no copy
// of the point data and no change to what the layers mean. And the camera is
// still framed on the *uncut* bounding box, so a plan and a top view of the
// same project cover the same ground — the cut changes what is visible, never
// where the shot is aimed (STANDARDS §6).

/// How far below the drawn geometry a detected floor plane may sit and still
/// be believed, in metres. A plane centroid further down than this belongs to
/// a stale per-plane cloud from a different run, not to this scene's floor.
constexpr double kFloorPlaneSlackM = 0.5;

/// World z of the floor, from the plane segmentation if the project has one.
///
/// The lowest horizontal plane, matching how the real-scan fixture test
/// identifies floor and ceiling. Nothing in an interior scan segments as a
/// horizontal plane below the floor, so "lowest" is the floor; a table top or
/// a windowsill is horizontal too but always above it.
std::optional<double> detect_floor_z(const ProjectDB &db) {
  if (!db.has_point_cloud("plane_centroids") ||
      !db.has_point_cloud("plane_normals"))
    return std::nullopt;

  const auto centroids = db.point_cloud_xyz("plane_centroids");
  const auto normals = db.point_cloud_normal("plane_normals");
  const std::size_t n_centroids = centroids ? centroids->size() : 0;
  const std::size_t n_normals = normals ? normals->size() : 0;
  if (n_centroids == 0 || n_centroids != n_normals) {
    core::warn("render: 'plane_centroids' ({}) and 'plane_normals' ({}) are "
               "empty or out of sync; placing the cut plane from the bounding "
               "box instead — re-run `rux create planes`",
               n_centroids, n_normals);
    return std::nullopt;
  }

  double lowest = 0.0;
  bool found = false;
  int horizontal = 0;
  for (std::size_t i = 0; i < n_centroids; ++i) {
    if (std::abs(static_cast<double>(normals->points[i].normal_z)) <
        kHorizontalNormalZ)
      continue;
    ++horizontal;
    const double z = static_cast<double>(centroids->points[i].z);
    if (!found || z < lowest) {
      lowest = z;
      found = true;
    }
  }
  if (!found) {
    core::warn("render: none of the {} segmented planes is horizontal "
               "(|n_z| >= {}), so no floor could be identified; placing the "
               "cut plane from the bounding box instead",
               n_centroids, kHorizontalNormalZ);
    return std::nullopt;
  }
  core::debug("render: floor at z={:.3f} m, from {} horizontal plane(s) of {}",
              lowest, horizontal, n_centroids);
  return lowest;
}

/// A resolved cut: where the plane sits and how that was decided.
struct CutPlane {
  double z = 0.0;           ///< world z of the plane
  double floor_z = 0.0;     ///< the reference it was measured up from
  double height = 0.0;      ///< metres above @ref floor_z
  bool from_planes = false; ///< floor came from the segmentation, not the bbox
};

/// Decide where to cut, and say so loudly when the answer is degenerate.
CutPlane resolve_cut_plane(const ProjectDB &db, const RenderOptions &opts,
                           const Bounds &bounds) {
  CutPlane cut;

  const std::optional<double> floor = detect_floor_z(db);
  if (floor && *floor >= bounds.min[2] - kFloorPlaneSlackM &&
      *floor <= bounds.max[2]) {
    cut.floor_z = *floor;
    cut.from_planes = true;
  } else {
    if (floor) {
      core::warn("render: the detected floor plane (z={:.3f}) lies outside the "
                 "drawn geometry (z in [{:.3f}, {:.3f}]); the per-plane clouds "
                 "are stale — placing the cut plane from the bounding box",
                 *floor, bounds.min[2], bounds.max[2]);
    }
    cut.floor_z = bounds.min[2];
  }

  cut.height = opts.cut_height.value_or(
      cut.from_planes ? kDefaultCutHeightM
                      : kDefaultCutBboxFraction * bounds.extent(2));
  cut.z = cut.floor_z + cut.height;

  // Neither of these is fatal — the render still means something — but a plan
  // that quietly turned back into a top view, or into an empty frame, must not
  // pass unremarked (STANDARDS §5).
  if (cut.z >= bounds.max[2]) {
    core::warn("render: the cut plane (z={:.3f}) is above everything drawn "
               "(z <= {:.3f}); nothing is cut away and this is a plain top "
               "view",
               cut.z, bounds.max[2]);
  } else if (cut.z <= bounds.min[2]) {
    core::warn("render: the cut plane (z={:.3f}) is below everything drawn "
               "(z >= {:.3f}); the frame will be empty — lower --cut-height or "
               "check the floor",
               cut.z, bounds.min[2]);
  }
  return cut;
}

/// Clip every actor in @p renderer to the half-space below z = @p z.
///
/// vtkPlane keeps the side its normal points to, so the normal points down.
/// Clipping planes live on the mapper and are evaluated in the vertex shader,
/// which is what makes this work uniformly for point sprites, mesh triangles
/// and component polylines alike.
void apply_cut_plane(vtkRenderer *renderer, double z) {
  vtkNew<vtkPlane> plane;
  plane->SetOrigin(0.0, 0.0, z);
  plane->SetNormal(0.0, 0.0, -1.0);

  vtkActorCollection *actors = renderer->GetActors();
  actors->InitTraversal();
  while (vtkActor *actor = actors->GetNextActor()) {
    if (vtkMapper *mapper = actor->GetMapper())
      mapper->AddClippingPlane(plane);
  }
}

// ── Offscreen OpenGL availability (#313) ─────────────────────────────────────

/// Refuse to render, with a diagnosis, when there is no offscreen GL here.
///
/// Called after every option and data check and immediately before the first
/// Render(): VTK's EGL render window crashes rather than failing when it has
/// no device, so this is the last point at which the condition can still be
/// reported. Placing it here (rather than at the top of render_view()) also
/// keeps the cheaper, more common diagnostics — a missing cloud, a stale label
/// layer — as the message a caller sees, on a GPU-less machine as much as
/// anywhere else.
void require_offscreen_gl() {
  // One probe per process. The answer cannot change while it runs, and an
  // orbit sweep would otherwise pay for it once per frame.
  static const OffscreenGlProbe probe = probe_offscreen_gl();

  if (probe.status == OffscreenGlStatus::usable) {
    core::debug("render: offscreen GL available — {}", probe.detail);
    return;
  }
  if (probe.status == OffscreenGlStatus::unknown) {
    // Not a verdict. VTK may be using OSMesa or a windowed GL path that never
    // goes through EGL; SupportsOpenGL() after the render still guards those.
    core::debug("render: offscreen GL probe inconclusive — {}", probe.detail);
    return;
  }

  if (display_configured()) {
    // VTK tries the windowed path first here and only falls back to EGL if it
    // cannot reach the display. An EGL that will not initialise says nothing
    // about GLX on a working X server, so this must not be fatal.
    core::warn("render: no usable EGL device ({}), but DISPLAY/WAYLAND_DISPLAY "
               "is set — continuing on VTK's windowed GL path",
               probe.detail);
    return;
  }

  throw OffscreenGlUnavailable(
      "render: this machine cannot create an offscreen OpenGL context, so "
      "there is nothing to render into — " +
      probe.detail +
      ". Headless rendering needs a GPU reachable through EGL (a DRM render "
      "node under /dev/dri, or a vendor driver), a software rasteriser (Mesa's "
      "llvmpipe EGL driver), or an X/Wayland display to fall back on. Set "
      "REUSEX_SKIP_EGL_PROBE=1 to bypass this check and let VTK try anyway.");
}

// ── Camera placement ─────────────────────────────────────────────────────────

/// Parallel scale (half the viewport height in world units) that exactly fits
/// an @p across x @p up rectangle into an image of the given aspect ratio.
///
/// vtkRenderer::ResetCamera fits the bounding *sphere*, which for an
/// axis-aligned view wastes the whole depth extent — a 3.6 x 4.0 m room 2.9 m
/// tall ends up rendered at ~65 % of the size it could be. For the orthographic
/// presets the projected extents are known exactly, so compute the fit instead.
double parallel_scale_for(double across, double up, double aspect,
                          double margin) {
  const double half_up =
      std::max(0.5 * up, 0.5 * across / std::max(aspect, 1e-9));
  return std::max(half_up, 1e-6) * margin;
}

/// Place the camera for a preset view, deterministically from @p bounds.
///
/// The view direction, up vector and projection type are always set
/// explicitly. Orthographic presets also get an exact parallel scale; the
/// perspective orbit delegates its distance to vtkRenderer::ResetCamera, whose
/// bounding-sphere fit is the right behaviour there because it keeps the scene
/// the same size at every point on the ring. Either way the result is a pure
/// function of the geometry, so the same project always frames the same shot
/// (STANDARDS §6).
void place_preset_camera(vtkRenderer *renderer, const RenderOptions &opts,
                         const Bounds &bounds) {
  double center[3];
  bounds.center(center);
  const double reach = std::max(bounds.diagonal(), 1.0);
  const double aspect =
      static_cast<double>(opts.width) / static_cast<double>(opts.height);

  vtkCamera *cam = renderer->GetActiveCamera();
  cam->SetFocalPoint(center);

  switch (opts.view) {
  case ViewPreset::top:
  case ViewPreset::plan:
    // Straight down -Z with +Y up the page. `plan` differs from `top` only in
    // the cut plane applied to the actors, so the two frame identically.
    cam->ParallelProjectionOn();
    cam->SetPosition(center[0], center[1], center[2] + reach);
    cam->SetViewUp(0.0, 1.0, 0.0);
    cam->SetParallelScale(parallel_scale_for(bounds.extent(0), bounds.extent(1),
                                             aspect, opts.margin));
    renderer->ResetCameraClippingRange();
    return;

  case ViewPreset::front:
    // Orthographic elevation looking along +Y, world Z up.
    cam->ParallelProjectionOn();
    cam->SetPosition(center[0], center[1] - reach, center[2]);
    cam->SetViewUp(0.0, 0.0, 1.0);
    cam->SetParallelScale(parallel_scale_for(bounds.extent(0), bounds.extent(2),
                                             aspect, opts.margin));
    renderer->ResetCameraClippingRange();
    return;

  case ViewPreset::orbit: {
    cam->ParallelProjectionOff();
    cam->SetViewAngle(40.0);
    const double azimuth = 2.0 * std::numbers::pi *
                           static_cast<double>(opts.orbit_index) /
                           static_cast<double>(opts.orbit_count);
    const double elevation =
        opts.orbit_elevation_deg * std::numbers::pi / 180.0;
    cam->SetPosition(
        center[0] + reach * std::cos(azimuth) * std::cos(elevation),
        center[1] + reach * std::sin(azimuth) * std::cos(elevation),
        center[2] + reach * std::sin(elevation));
    cam->SetViewUp(0.0, 0.0, 1.0);
    break;
  }

  case ViewPreset::explicit_camera:
    return; // handled by place_explicit_camera
  }

  renderer->ResetCamera();
  cam->Zoom(1.0 / opts.margin); // margin is validated positive
  renderer->ResetCameraClippingRange();
}

/// Place the camera from an explicit pose + pinhole intrinsics.
void place_explicit_camera(vtkRenderer *renderer, const RenderOptions &opts,
                           const Bounds &bounds) {
  const CameraSpec &spec = *opts.camera;
  if (spec.fy <= 0.0 || spec.fx <= 0.0) {
    throw std::runtime_error(
        "render: explicit camera needs positive fx/fy (got fx=" +
        std::to_string(spec.fx) + ", fy=" + std::to_string(spec.fy) + ")");
  }

  const Eigen::Affine3f c2w = geometry::to_affine(spec.pose);
  const Eigen::Vector3f eye = c2w.translation();
  // OpenCV camera axes: +x right, +y down, +z along the view direction.
  const Eigen::Vector3f forward = c2w.linear() * Eigen::Vector3f::UnitZ();
  const Eigen::Vector3f down = c2w.linear() * Eigen::Vector3f::UnitY();

  vtkCamera *cam = renderer->GetActiveCamera();
  cam->ParallelProjectionOff();
  cam->SetPosition(eye.x(), eye.y(), eye.z());
  const Eigen::Vector3f target =
      eye + forward * static_cast<float>(std::max(bounds.diagonal(), 1.0));
  cam->SetFocalPoint(target.x(), target.y(), target.z());
  cam->SetViewUp(-down.x(), -down.y(), -down.z());

  // Vertical field of view from fy and the image height.
  const double view_angle =
      2.0 * std::atan2(0.5 * static_cast<double>(opts.height), spec.fy) *
      180.0 / std::numbers::pi;
  cam->SetViewAngle(view_angle);

  if (std::abs(spec.fx - spec.fy) > 1e-3 * spec.fy) {
    core::debug("render: anisotropic intrinsics (fx={:.2f}, fy={:.2f}); VTK "
                "renders square pixels, using fy",
                spec.fx, spec.fy);
  }

  // Principal-point offset. VTK's window center is a normalized shift of the
  // projection center with +x right and +y up, opposite in sign to an image
  // principal point measured from the top-left. Only the centered case is
  // covered by tests; see #294.
  if (spec.cx > 0.0 || spec.cy > 0.0) {
    const double wcx = -2.0 *
                       (spec.cx - 0.5 * static_cast<double>(opts.width)) /
                       static_cast<double>(opts.width);
    const double wcy = 2.0 *
                       (spec.cy - 0.5 * static_cast<double>(opts.height)) /
                       static_cast<double>(opts.height);
    if (std::abs(wcx) > 1e-6 || std::abs(wcy) > 1e-6)
      cam->SetWindowCenter(wcx, wcy);
  }

  renderer->ResetCameraClippingRange();
}

// ── Image readback ───────────────────────────────────────────────────────────

/// Copy the rendered frame into a BGR cv::Mat, flipping VTK's bottom-up rows.
cv::Mat to_bgr_image(vtkImageData *image, int width, int height) {
  int dims[3] = {0, 0, 0};
  image->GetDimensions(dims);
  if (dims[0] <= 0 || dims[1] <= 0) {
    throw std::runtime_error(
        "render: the offscreen framebuffer came back empty (" +
        std::to_string(dims[0]) + "x" + std::to_string(dims[1]) + ")");
  }

  cv::Mat out(dims[1], dims[0], CV_8UC3);
  const auto *src =
      static_cast<const unsigned char *>(image->GetScalarPointer());
  const std::size_t stride = static_cast<std::size_t>(dims[0]) * 3;
  for (int y = 0; y < dims[1]; ++y) {
    const unsigned char *row = src + static_cast<std::size_t>(y) * stride;
    auto *dst =
        out.ptr<cv::Vec3b>(dims[1] - 1 - y); // VTK origin is bottom-left
    for (int x = 0; x < dims[0]; ++x) {
      dst[x] = cv::Vec3b(row[3 * x + 2], row[3 * x + 1], row[3 * x + 0]);
    }
  }

  if (dims[0] != width || dims[1] != height) {
    // render_view() documents an exact output size; silently handing back a
    // different one would corrupt anything that lays images out (STANDARDS §5).
    throw std::runtime_error(
        "render: the offscreen framebuffer came back " +
        std::to_string(dims[0]) + "x" + std::to_string(dims[1]) + " but " +
        std::to_string(width) + "x" + std::to_string(height) +
        " was requested — the GPU may be clamping the offscreen buffer size");
  }
  return out;
}

void validate(const RenderOptions &opts) {
  if (opts.layers.empty())
    throw std::runtime_error(
        "render: no layers selected — pass at least one of "
        "cloud, labels, planes, rooms, instances, mesh, "
        "components");
  if (opts.width <= 0 || opts.height <= 0)
    throw std::runtime_error("render: image size must be positive (got " +
                             std::to_string(opts.width) + "x" +
                             std::to_string(opts.height) + ")");
  if (opts.view == ViewPreset::orbit) {
    if (opts.orbit_count <= 0)
      throw std::runtime_error("render: orbit count must be >= 1 (got " +
                               std::to_string(opts.orbit_count) + ")");
    if (opts.orbit_index < 0 || opts.orbit_index >= opts.orbit_count)
      throw std::runtime_error(
          "render: orbit index " + std::to_string(opts.orbit_index) +
          " is out of range for a ring of " + std::to_string(opts.orbit_count));
  }
  if (opts.view == ViewPreset::explicit_camera && !opts.camera)
    throw std::runtime_error(
        "render: view is 'explicit_camera' but no camera was supplied");
  if (opts.margin <= 0.0)
    throw std::runtime_error("render: margin must be positive (got " +
                             std::to_string(opts.margin) + ")");
  if (opts.point_size <= 0.0)
    throw std::runtime_error("render: point size must be positive (got " +
                             std::to_string(opts.point_size) + ")");
  if (opts.cut_height && *opts.cut_height <= 0.0)
    throw std::runtime_error(
        "render: cut height must be positive — it is measured upward from the "
        "floor (got " +
        std::to_string(*opts.cut_height) + ")");
  // At +/-90 degrees the orbit view direction is parallel to the (0,0,1) up
  // vector and the camera basis collapses.
  if (opts.view == ViewPreset::orbit &&
      std::abs(opts.orbit_elevation_deg) > 89.0)
    throw std::runtime_error(
        "render: orbit elevation must be within +/-89 degrees (got " +
        std::to_string(opts.orbit_elevation_deg) + ")");
}

} // namespace

std::optional<Layer> layer_from_string(std::string_view name) {
  for (const Layer layer : all_layers()) {
    if (to_string(layer) == name)
      return layer;
  }
  return std::nullopt;
}

std::string_view to_string(Layer layer) {
  switch (layer) {
  case Layer::cloud:
    return "cloud";
  case Layer::labels:
    return "labels";
  case Layer::planes:
    return "planes";
  case Layer::rooms:
    return "rooms";
  case Layer::instances:
    return "instances";
  case Layer::mesh:
    return "mesh";
  case Layer::components:
    return "components";
  }
  return "unknown";
}

std::string_view to_string(ViewPreset view) {
  switch (view) {
  case ViewPreset::top:
    return "top";
  case ViewPreset::plan:
    return "plan";
  case ViewPreset::front:
    return "front";
  case ViewPreset::orbit:
    return "orbit";
  case ViewPreset::explicit_camera:
    return "frame";
  }
  return "unknown";
}

std::optional<ViewPreset> view_preset_from_string(std::string_view name) {
  for (const ViewPreset view :
       {ViewPreset::top, ViewPreset::plan, ViewPreset::front, ViewPreset::orbit,
        ViewPreset::explicit_camera}) {
    if (to_string(view) == name)
      return view;
  }
  return std::nullopt;
}

const std::vector<Layer> &all_layers() {
  static const std::vector<Layer> layers{
      Layer::cloud,     Layer::labels, Layer::planes,    Layer::rooms,
      Layer::instances, Layer::mesh,   Layer::components};
  return layers;
}

cv::Mat render_view(const ProjectDB &db, const RenderOptions &opts) {
  validate(opts);
  if (!db.is_open())
    throw std::runtime_error("render: project database is not open");

  install_vtk_log_bridge();
  const stopwatch timer;

  vtkNew<vtkRenderer> renderer;
  renderer->SetBackground(opts.background[0], opts.background[1],
                          opts.background[2]);

  Bounds bounds;
  CloudPtr geometry; // loaded lazily; shared by every point layer
  std::size_t drawn_points = 0;
  std::size_t drawn_faces = 0;

  const auto ensure_geometry = [&]() -> const Cloud & {
    if (!geometry) {
      geometry = load_geometry_cloud(db, opts);
      for (const auto &p : *geometry)
        bounds.add(p.x, p.y, p.z);
    }
    return *geometry;
  };

  for (const Layer layer : opts.layers) {
    switch (layer) {
    case Layer::cloud: {
      const Cloud &cloud = ensure_geometry();
      std::vector<unsigned char> colors(cloud.size() * 3);
      for (std::size_t i = 0; i < cloud.size(); ++i) {
        colors[3 * i + 0] = cloud[i].r;
        colors[3 * i + 1] = cloud[i].g;
        colors[3 * i + 2] = cloud[i].b;
      }
      add_points_actor(renderer, make_point_polydata(cloud, colors),
                       opts.point_size);
      drawn_points += cloud.size();
      break;
    }

    case Layer::labels:
    case Layer::planes:
    case Layer::rooms:
    case Layer::instances: {
      const std::string name = label_cloud_name(layer);
      if (!db.has_point_cloud(name)) {
        throw std::runtime_error("render: layer '" +
                                 std::string(to_string(layer)) +
                                 "' needs the label cloud '" + name +
                                 "', which this project does not have — run `" +
                                 producing_command(layer) + "` first");
      }
      const Cloud &cloud = ensure_geometry();
      const CloudLPtr labels = db.point_cloud_label(name);
      if (!labels || labels->empty()) {
        throw std::runtime_error("render: label cloud '" + name +
                                 "' is empty — re-run `" +
                                 producing_command(layer) + "`");
      }
      if (labels->size() != cloud.size()) {
        // The per-scan clouds are index-aligned by contract (STANDARDS §3.2);
        // a mismatch means one of them is stale.
        throw std::runtime_error(
            "render: '" + name + "' has " + std::to_string(labels->size()) +
            " labels but '" + opts.cloud_name + "' has " +
            std::to_string(cloud.size()) +
            " points — the clouds are out of sync; re-run `" +
            producing_command(layer) + "`");
      }

      std::vector<unsigned char> colors(cloud.size() * 3);
      std::size_t unlabeled = 0;
      for (std::size_t i = 0; i < cloud.size(); ++i) {
        const std::uint32_t label = (*labels)[i].label;
        if (label == 0)
          ++unlabeled;
        label_color(label, &colors[3 * i]);
      }
      if (unlabeled == cloud.size()) {
        core::warn("render: every point in '{}' is unlabeled (0/{} labeled); "
                   "the '{}' layer will be uniformly grey",
                   name, cloud.size(), to_string(layer));
      }
      add_points_actor(renderer, make_point_polydata(cloud, colors),
                       opts.point_size);
      drawn_points += cloud.size();
      break;
    }

    case Layer::mesh: {
      if (!db.has_mesh(opts.mesh_name)) {
        throw std::runtime_error("render: project has no mesh named '" +
                                 opts.mesh_name + "' — run `" +
                                 producing_command(Layer::mesh) + "` first");
      }
      const auto mesh = db.mesh(opts.mesh_name);
      if (!mesh || mesh->polygons.empty()) {
        throw std::runtime_error("render: mesh '" + opts.mesh_name +
                                 "' has no faces — re-run `" +
                                 producing_command(Layer::mesh) + "`");
      }
      add_mesh_actor(renderer, *mesh, bounds);
      drawn_faces += mesh->polygons.size();
      break;
    }

    case Layer::components: {
      const auto names = db.list_building_components();
      if (names.empty()) {
        throw std::runtime_error(
            "render: project has no building components — run `" +
            producing_command(Layer::components) + "` first");
      }
      std::vector<geometry::BuildingComponent> components;
      components.reserve(names.size());
      for (const auto &name : names)
        components.push_back(geometry::building_component(db, name));
      add_component_actors(renderer, components, bounds);
      break;
    }
    }
  }

  if (!bounds.valid) {
    throw std::runtime_error(
        "render: the selected layers produced no drawable geometry");
  }

  // A plan view is a top view plus the cut; an explicit `cut` cuts any view.
  if (opts.cut || opts.view == ViewPreset::plan) {
    const CutPlane cut = resolve_cut_plane(db, opts, bounds);
    apply_cut_plane(renderer, cut.z);
    core::info("render: cut plane at z={:.3f} m — {:.2f} m above the {} floor "
               "at z={:.3f}",
               cut.z, cut.height, cut.from_planes ? "detected" : "bounding-box",
               cut.floor_z);
  }

  vtkNew<vtkRenderWindow> window;
  // The whole point of this function: never touch a window manager. With the
  // flake's EGL-enabled VTK this succeeds with no DISPLAY set (#294).
  window->SetOffScreenRendering(1);
  window->SetMultiSamples(0); // removes MSAA sample-order variance
  window->AddRenderer(renderer);
  window->SetSize(opts.width, opts.height);

  if (opts.view == ViewPreset::explicit_camera) {
    place_explicit_camera(renderer, opts, bounds);
  } else {
    place_preset_camera(renderer, opts, bounds);
  }

  // Everything above this line is checked without touching the GPU, so a
  // GPU-less machine still gets the more useful diagnosis when the request
  // itself is wrong. From here on VTK owns the process: its EGL render window
  // crashes rather than fails when it has no device, so the question has to be
  // asked before the first Render() (#313).
  require_offscreen_gl();

  window->Render();
  core::debug("render: window class {}", window->GetClassName());

  // A render window with no usable OpenGL implementation does not fail — it
  // hands back a correctly sized frame of pure background. Nothing downstream
  // can tell that apart from a legitimately empty scene, so check it here
  // rather than let a black PNG be reported as success (STANDARDS §5). The
  // probe above cannot replace this: it answers for EGL, and VTK may have
  // taken a windowed or OSMesa path instead.
  if (window->SupportsOpenGL() == 0) {
    throw OffscreenGlUnavailable(
        "render: the render window has no usable OpenGL implementation "
        "(window class " +
        std::string(window->GetClassName()) +
        "); with no display this needs an EGL-capable VTK and a rendering "
        "device");
  }

  vtkNew<vtkWindowToImageFilter> capture;
  capture->SetInput(window);
  capture->SetInputBufferTypeToRGB();
  capture->ReadFrontBufferOff();
  // Update() re-renders by default; the frame is already drawn, and an orbit
  // sweep would otherwise pay for 2 renders per view.
  capture->ShouldRerenderOff();
  capture->Update();

  cv::Mat image = to_bgr_image(capture->GetOutput(), opts.width, opts.height);

  // Loud, but not fatal: a uniform frame with geometry in the scene means
  // nothing reached the framebuffer. It is legitimate only for an explicit
  // camera the caller aimed away from the geometry.
  double lo = 0.0;
  double hi = 0.0;
  cv::minMaxLoc(image.reshape(1), &lo, &hi);
  if (lo == hi && (drawn_points + drawn_faces) > 0) {
    core::warn("render: the frame is a uniform colour despite {} points and {} "
               "faces in the scene — the camera may be pointed away from the "
               "geometry, or the GL context produced nothing",
               drawn_points, drawn_faces);
  }

  core::info("render: {} points, {} faces -> {}x{} in {:.2f}s", drawn_points,
             drawn_faces, image.cols, image.rows, timer.elapsed());

  // vtkNew would finalize on destruction anyway; doing it here releases the
  // EGL surface before the (potentially large) image is copied out.
  window->Finalize();

  return image;
}

CameraSpec camera_from_sensor_frame(const ProjectDB &db, int node_id, int width,
                                    int height) {
  if (width <= 0 || height <= 0)
    throw std::runtime_error(
        "render: camera_from_sensor_frame needs a positive output size (got " +
        std::to_string(width) + "x" + std::to_string(height) + ")");

  const core::SensorIntrinsics intr = db.sensor_frame_intrinsics(node_id);
  if (intr.fx <= 0.0 || intr.fy <= 0.0 || intr.width <= 0 || intr.height <= 0) {
    throw std::runtime_error(
        "render: sensor frame " + std::to_string(node_id) +
        " has no usable intrinsics (fx=" + std::to_string(intr.fx) +
        ", fy=" + std::to_string(intr.fy) + ", " + std::to_string(intr.width) +
        "x" + std::to_string(intr.height) + ")");
  }

  // Unlike the pipeline stages, there is nothing to skip here: this function
  // returns exactly one camera. Falling through to `sensor_frame_pose()`'s
  // identity fallback would render the scene from the world origin and hand
  // back a PNG that looks like a real answer (#336), so refuse the same way
  // the unusable-intrinsics check above does.
  if (!db.has_sensor_frame_pose(node_id)) {
    throw std::runtime_error(
        "render: sensor frame " + std::to_string(node_id) +
        " has no usable stored pose (missing, non-finite, or degenerate "
        "transform) — pick another frame, or run 'rux optimize' to give the "
        "scan poses");
  }

  // The stored pose is body-to-world; the camera sits at pose * local_transform
  // — the same composition segmentation/reconstruct.cpp uses to back-project
  // depth, which is what makes a render line up with the captured frame.
  const Eigen::Affine3f c2w =
      geometry::to_affine(db.sensor_frame_pose(node_id)) *
      geometry::to_affine(intr.local_transform);

  // Intrinsics describe the captured frame; rescale them to the output size.
  const double sx = static_cast<double>(width) / intr.width;
  const double sy = static_cast<double>(height) / intr.height;

  CameraSpec spec;
  spec.pose = geometry::to_array16(c2w);
  spec.fx = intr.fx * sx;
  spec.fy = intr.fy * sy;
  spec.cx = intr.cx * sx;
  spec.cy = intr.cy * sy;
  return spec;
}

} // namespace reusex::visualize
