// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "view/panorama_handler.hpp"

#include <reusex/core/ProjectDB.hpp>

#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

#include <vtkCamera.h>
#include <vtkImageData.h>
#include <vtkImageImport.h>
#include <vtkPropCollection.h>
#include <vtkRendererCollection.h>
#include <vtkTexture.h>

namespace rux::view {

vtkRenderer *
viewport_renderer(const rux::VizualizationObserver::ViewerPtr &viewer,
                  int viewport) {
  auto renderers = viewer->getRendererCollection();
  renderers->InitTraversal();
  vtkRenderer *renderer = nullptr;
  for (int i = 0; i <= viewport; ++i)
    renderer = renderers->GetNextItem();
  return renderer;
}

namespace {

/// Convert a BGR cv::Mat to vtkImageData (RGB, bottom-up for VTK).
vtkSmartPointer<vtkImageData> cv_mat_to_vtk_image(const cv::Mat &bgr) {
  cv::Mat rgb;
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  cv::flip(rgb, rgb, 0); // VTK expects bottom-up

  auto importer = vtkSmartPointer<vtkImageImport>::New();
  importer->SetDataSpacing(1, 1, 1);
  importer->SetDataOrigin(0, 0, 0);
  importer->SetWholeExtent(0, rgb.cols - 1, 0, rgb.rows - 1, 0, 0);
  importer->SetDataExtentToWholeExtent();
  importer->SetDataScalarTypeToUnsignedChar();
  importer->SetNumberOfScalarComponents(3);
  importer->SetImportVoidPointer(rgb.data, /*save=*/1);
  importer->Update();

  auto image = vtkSmartPointer<vtkImageData>::New();
  image->DeepCopy(importer->GetOutput());
  return image;
}

} // namespace

void activate_panorama_skybox(
    const fs::path &db_path, const PanoramaInfo &info,
    vtkSmartPointer<vtkSkybox> &skybox,
    const rux::VizualizationObserver::ViewerPtr &viewer, int viewport) {
  reusex::ProjectDB db(db_path, /*readOnly=*/true);
  cv::Mat pano = db.panoramic_image(info.id);

  if (pano.empty()) {
    spdlog::error("Panorama image {} is empty", info.id);
    return;
  }
  spdlog::debug("Panorama image: {}x{} type={}", pano.cols, pano.rows,
                pano.type());

  auto vtk_img = cv_mat_to_vtk_image(pano);

  // Use Sphere projection — the skybox fragment shader samples the 2D
  // equirectangular texture directly via spherical-coordinate mapping,
  // avoiding the GPU-based cubemap conversion which requires a fully
  // connected VTK pipeline (SetInputConnection) to work reliably.
  auto texture = vtkSmartPointer<vtkTexture>::New();
  texture->SetInputData(vtk_img);
  texture->InterpolateOn();
  texture->MipmapOn();

  if (!skybox)
    skybox = vtkSmartPointer<vtkSkybox>::New();
  skybox->SetTexture(texture);
  skybox->SetProjectionToSphere();
  skybox->GammaCorrectOff(); // JPEG is sRGB, not linear

  auto *renderer = viewport_renderer(viewer, viewport);
  renderer->AddActor(skybox);

  // Skybox returns zero bounds, so VTK's auto clipping range calculation
  // produces a degenerate range that clips the skybox geometry away.
  renderer->GetActiveCamera()->SetClippingRange(0.01, 100.0);
}

void deactivate_panorama_skybox(
    const vtkSmartPointer<vtkSkybox> &skybox,
    const rux::VizualizationObserver::ViewerPtr &viewer, int viewport) {
  if (!skybox)
    return;
  auto *renderer = viewport_renderer(viewer, viewport);
  renderer->RemoveActor(skybox);
}

void enter_panorama_mode(std::shared_ptr<PanoramaState> state, int pano_index,
                         const fs::path &project_path,
                         const rux::VizualizationObserver::ViewerPtr &viewer,
                         const std::vector<int> &viewports) {

  const auto &info = state->infos[static_cast<size_t>(pano_index)];

  // Save camera state — use the viewport renderer, not the default one
  auto *renderer = viewport_renderer(viewer, viewports[0]);
  auto *cam = renderer->GetActiveCamera();
  cam->GetPosition(state->saved.cam_pos);
  cam->GetFocalPoint(state->saved.cam_focal);
  cam->GetViewUp(state->saved.cam_up);

  // Hide all currently visible props in the renderer
  state->saved.hidden_props.clear();
  auto *props = renderer->GetViewProps();
  props->InitTraversal();
  while (auto *prop = props->GetNextProp()) {
    if (prop->GetVisibility()) {
      state->saved.hidden_props.emplace_back(prop);
      prop->VisibilityOff();
    }
  }

  // Activate skybox (added after hiding, so it stays visible)
  activate_panorama_skybox(project_path, info, state->skybox, viewer,
                           viewports[0]);

  // Position camera at panorama origin
  if (info.pose_valid) {
    cam->SetPosition(info.px, info.py, info.pz);
    // Row-major 4x4: column 2 is the Z-axis (forward in camera frame)
    double fx = info.pose[2];
    double fy = info.pose[6];
    double fz = info.pose[10];
    cam->SetFocalPoint(info.px + fx, info.py + fy, info.pz + fz);
    // Up is negative Y-axis of the pose (camera convention)
    cam->SetViewUp(-info.pose[1], -info.pose[5], -info.pose[9]);
    // Keep clipping range valid for skybox rendering
    cam->SetClippingRange(0.01, 100.0);
  }

  state->immersive = true;
  state->active_index = pano_index;
  spdlog::info("Entered panorama mode: {} [{}]", info.filename, pano_index + 1);
}

void exit_panorama_mode(std::shared_ptr<PanoramaState> state,
                        const rux::VizualizationObserver::ViewerPtr &viewer,
                        const std::vector<int> &viewports) {

  // Remove skybox
  deactivate_panorama_skybox(state->skybox, viewer, viewports[0]);

  // Restore visibility of all previously visible props
  for (auto &prop : state->saved.hidden_props)
    prop->VisibilityOn();
  state->saved.hidden_props.clear();

  // Restore camera
  auto *renderer = viewport_renderer(viewer, viewports[0]);
  auto *cam = renderer->GetActiveCamera();
  cam->SetPosition(state->saved.cam_pos);
  cam->SetFocalPoint(state->saved.cam_focal);
  cam->SetViewUp(state->saved.cam_up);

  state->immersive = false;
  state->active_index = -1;
  spdlog::info("Exited panorama mode");
}

} // namespace rux::view
