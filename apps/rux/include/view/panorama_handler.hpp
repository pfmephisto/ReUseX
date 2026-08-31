// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "processing_observer.hpp"
#include "view/viewer_types.hpp"

#include <vtkProp.h>
#include <vtkRenderer.h>
#include <vtkSkybox.h>
#include <vtkSmartPointer.h>

#include <filesystem>
#include <memory>
#include <vector>

namespace rux::view {

namespace fs = std::filesystem;

/// State for immersive panorama viewing.
struct PanoramaState {
  bool immersive = false;            ///< Currently in panorama mode
  int active_index = -1;             ///< Which panorama is displayed
  std::vector<PanoramaInfo> infos;   ///< All panorama metadata
  vtkSmartPointer<vtkSkybox> skybox; ///< Reusable skybox actor

  /// Saved viewer state for restoration on exit.
  struct SavedView {
    double cam_pos[3], cam_focal[3], cam_up[3];
    std::vector<vtkSmartPointer<vtkProp>>
        hidden_props; ///< All props hidden on enter
  } saved;
};

/// Get the VTK renderer for a PCL viewport index.
///
/// PCL creates a default renderer (index 0) in the PCLVisualizer constructor,
/// then createViewPort() adds additional renderers. Geometry is added to the
/// viewport renderer, NOT the default one. GetFirstRenderer() returns the
/// wrong renderer — use this helper instead.
vtkRenderer *
viewport_renderer(const rux::VizualizationObserver::ViewerPtr &viewer,
                  int viewport);

/// Activate the panorama skybox in the VTK renderer.
///
/// Loads the image from the database, converts to cubemap, and adds the
/// skybox actor to the renderer.
void activate_panorama_skybox(
    const fs::path &db_path, const PanoramaInfo &info,
    vtkSmartPointer<vtkSkybox> &skybox,
    const rux::VizualizationObserver::ViewerPtr &viewer, int viewport);

/// Remove the skybox actor from the VTK renderer.
void deactivate_panorama_skybox(
    const vtkSmartPointer<vtkSkybox> &skybox,
    const rux::VizualizationObserver::ViewerPtr &viewer, int viewport);

/// Enter immersive panorama mode: hide all scene geometry, show skybox.
void enter_panorama_mode(std::shared_ptr<PanoramaState> state, int pano_index,
                         const fs::path &project_path,
                         const rux::VizualizationObserver::ViewerPtr &viewer,
                         const std::vector<int> &viewports);

/// Exit immersive panorama mode: restore scene and camera.
void exit_panorama_mode(std::shared_ptr<PanoramaState> state,
                        const rux::VizualizationObserver::ViewerPtr &viewer,
                        const std::vector<int> &viewports);

} // namespace rux::view
